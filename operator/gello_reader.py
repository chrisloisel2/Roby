#!/usr/bin/env python3
"""GELLO leader arm reader (Arduino + 7x AS5600L magnetic encoders).

Standalone reimplementation of lerobot's `GelloAs5600Leader` teleoperator
(see `~/03_JelloSoft/rebot_lerobot/lerobot/src/lerobot/teleoperators/
gello_as5600_leader/` on the robot PC) that doesn't depend on the lerobot
package -- that package pulls in heavy deps (torch etc.) that have no
business being on the operator PC just to read a serial port. This file
ports only the reading + calibration math, byte for byte, from that class.

The GELLO firmware streams plain ASCII over serial, one line every ~16ms:

    t<ms> J1:<deg> J2:<deg> ... J7:<deg>

<deg> is signed in [-180, 180), relative to the firmware's own zero ("arm
fully straight") -- unrelated to the follower's calibrated zero, which is
why the offset/scale/direction transform below exists. A sensor read
failure prints "ERR" for that joint; we keep the last valid value instead of
snapping to 0.

`gello_calibration.json` (this directory) is a copy of the calibration file
already generated on the robot PC by GelloAs5600Leader.calibrate() (3-step
process: firmware zero, range-of-motion sweep, alignment with the
follower's resting pose). It's tied to the physical GELLO unit and its
firmware EEPROM zero, not to which machine reads the serial port, so it
stays valid unchanged even though the reading moved from the robot PC to
here (the operator PC).
"""
import json
import os
import re
import threading
import time
from pathlib import Path

import serial

PORT_ENV = "GELLO_PORT"
BAUDRATE = 115200
CALIBRATION_PATH = Path(__file__).resolve().parent / "gello_calibration.json"

# Follower joint name -> leader sensor ID (firmware J<n>). Matches
# config_gello_as5600_leader.py on the robot PC.
JOINT_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_yaw": 5,
    "wrist_roll": 6,
    "gripper": 7,
}

# Sign applied to each sensor's raw angle to match the follower's
# convention. Determined empirically on this hardware (config_gello_as5600_
# leader.py); do not change without re-verifying against the robot PC.
JOINT_DIRECTIONS = {
    "shoulder_pan": -1,
    "shoulder_lift": -1,
    "elbow_flex": -1,
    "wrist_flex": 1,
    "wrist_yaw": -1,
    "wrist_roll": -1,
    "gripper": -1,
}

# follower_deg = scale * leader_deg. 1.0 everywhere except the gripper: its
# trigger travels less than the follower gripper's range (~116deg measured
# -> 270deg), so the scale amplifies it.
JOINT_SCALES = {
    "shoulder_pan": 1.0,
    "shoulder_lift": 1.0,
    "elbow_flex": 1.0,
    "wrist_flex": 1.0,
    "wrist_yaw": 1.0,
    "wrist_roll": 1.0,
    "gripper": 3.4,
}

LEADER_SMOOTH = 0.15  # exponential smoothing, anti-tremor (0=very smooth/laggy, 1=raw)
RANGE_SAFETY_MARGIN_DEG = 5.0  # clip raw sensor glitches to measured range +/- this margin
FIRMWARE_RESET_SETTLE_SEC = 2.5  # opening the port resets the Arduino (DTR)

_JOINT_LINE_RE = re.compile(r"J(\d+):(-?\d+\.\d+|ERR)")


def _load_calibration() -> dict:
    with open(CALIBRATION_PATH) as f:
        return json.load(f)


class GelloReader:
    """Background serial reader + lerobot-compatible calibration transform.

    Reproduces GelloAs5600Leader.get_action() (clip to measured range ->
    exponential smoothing -> sign -> scale -> offset) so the numbers match
    what the follower already expects, without needing a live connection to
    the follower itself (the offset was fixed once at calibration time).
    """

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self._calibration = _load_calibration()
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.2)
        self._lock = threading.Lock()
        self._raw: dict[int, float | None] = dict.fromkeys(JOINT_IDS.values(), None)
        self._filtered: dict[str, float | None] = dict.fromkeys(JOINT_IDS.keys(), None)
        self._stop = threading.Event()

        # Opening the port resets the Arduino (DTR); let the firmware boot,
        # then answer "n" to its recalibrate prompt so we don't wait out its
        # ~30s timeout. Only the reading side moved machines -- the physical
        # GELLO and its firmware EEPROM zero didn't -- so we deliberately
        # keep the existing calibration instead of re-triggering it.
        time.sleep(FIRMWARE_RESET_SETTLE_SEC)
        self.ser.reset_input_buffer()
        self.ser.write(b"n\n")

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                continue
            if not raw:
                continue
            text = raw.decode("ascii", errors="ignore")
            matches = _JOINT_LINE_RE.findall(text)
            if not matches:
                continue
            with self._lock:
                for jid_s, val_s in matches:
                    if val_s == "ERR":
                        continue
                    self._raw[int(jid_s)] = float(val_s)

    def get_action(self) -> dict[str, float]:
        """Return {joint_name: calibrated_follower_degrees} for whichever
        joints already have a valid reading. Right after startup this is
        naturally a partial dict -- an omitted key means "no update yet",
        never a fabricated 0.0."""
        action: dict[str, float] = {}
        for name, joint_id in JOINT_IDS.items():
            with self._lock:
                raw = self._raw.get(joint_id)
            calib = self._calibration.get(name)

            if raw is not None and calib is not None:
                lo, hi = calib["range_min"], calib["range_max"]
                raw = max(lo - RANGE_SAFETY_MARGIN_DEG, min(hi + RANGE_SAFETY_MARGIN_DEG, raw))

            prev = self._filtered[name]
            if raw is None:
                filtered = prev
            elif prev is None:
                filtered = raw
            else:
                filtered = prev + LEADER_SMOOTH * (raw - prev)
            self._filtered[name] = filtered

            if filtered is None or calib is None:
                continue

            direction = JOINT_DIRECTIONS[name]
            scale = JOINT_SCALES.get(name, 1.0)
            offset_deg = calib["homing_offset"] / 100.0  # stored in centidegrees
            action[name] = scale * (direction * filtered + offset_deg)

        return action

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.ser.close()


_reader: GelloReader | None = None
_init_failed = False


def read_gello() -> dict | None:
    """Entry point imported by input_agent.py.

    Returns {"joints": {name: deg, ...}, "gripper": deg, "mode":
    "joint_position"}, with only the joints that have a valid reading so far
    (a subset right after startup is normal), or None if the GELLO isn't
    usable at all: GELLO_PORT unset, or the port failed to open. That
    failure is sticky for this process's lifetime (no retry loop) -- restart
    input_agent.py once the GELLO is actually plugged in.
    """
    global _reader, _init_failed
    if _init_failed:
        return None
    if _reader is None:
        port = os.environ.get(PORT_ENV)
        if not port:
            print(f"[gello_reader] {PORT_ENV} not set -- GELLO disabled.")
            _init_failed = True
            return None
        try:
            _reader = GelloReader(port)
            print(f"[gello_reader] connected on {port}")
        except Exception as exc:
            print(f"[gello_reader] failed to open {port}: {exc}")
            _init_failed = True
            return None

    action = _reader.get_action()
    if not action:
        return None
    gripper = action.pop("gripper", None)
    result: dict = {"joints": action, "mode": "joint_position"}
    if gripper is not None:
        result["gripper"] = gripper
    return result


if __name__ == "__main__":
    print(f"gello_reader standalone test ({PORT_ENV}={os.environ.get(PORT_ENV)!r}) -- "
          "move the GELLO, Ctrl+C to stop.")
    try:
        while True:
            action = read_gello()
            if action is None:
                print("no reading yet...                                            ", end="\r")
            else:
                joints = " ".join(f"{k}={v:+7.2f}" for k, v in action["joints"].items())
                gripper = action.get("gripper")
                gripper_str = f"  gripper={gripper:+7.2f}" if gripper is not None else ""
                print(f"{joints}{gripper_str}    ", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped.")
