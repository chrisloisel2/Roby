#!/usr/bin/env python3
"""Operator-side input agent.

Reads the joystick (and GELLO, if present) and publishes robot commands on
Zenoh. The deadman button on the joystick gates all motion: when it is
released, we publish a zero command and deadman="false" so the robot stops.

Direction is SNAPPED to 8 zones of 45° (mecanum: forward/back, strafe
left/right, 4 diagonals) rather than read as a raw continuous stick value --
same pattern already proven on this robot in
~/catkin_ws/u2canfd/mecanum_control.py. Two deliberate reasons, not just
style:

  1. A raw stick reading with no deadzone lets any analog noise near center
     bleed into vx/vy/wz continuously, which the robot then tries to track
     at 100 Hz -- this is what caused the wheels to shake/buzz on deadman
     press with the previous continuous-axis version of this file.
  2. Cheap sticks rarely return to *exact* (0, 0); a snap-to-nearest-sector
     with a deadzone radius makes "centered" reliably mean "stopped".

Base command contract: vx / vy / wz are normalized to [-1, 1]; vx=forward,
vy=lateral (right +), wz=rotation. The robot agent mixes these into mecanum
wheel targets and scales by its own MAX_VEL/ROT_VEL.

GELLO data goes DIRECTLY to robot/arm_agent.py's own WebSocket
(ws://<robot-ip>:8767), NOT over Zenoh -- same change as the browser's own
GELLO path (operator/web/static/js/armLink.js + gello.js), for the same
reason: one fewer hop, and arm_agent.py no longer has a robot/cmd/arm
Zenoh subscriber at all. What's relayed is the RAW, unprocessed GELLO
serial line -- gello_reader.py does no calibration math (see its module
docstring for why: a hand-ported reimplementation of that math had a real
bug once already). arm_agent.py runs the actual lerobot
GelloAs5600RawLeader class server-side instead. Requires ROBOT_IP (the
robot PC's address, not Zenoh-routed) -- only enforced once a GELLO is
actually detected, so joystick-only base control still works with no
GELLO and no ROBOT_IP set.
"""
import json
import math
import os
import time
from pathlib import Path

# Must be set before `import pygame`: SDL2's HIDAPI joystick backend segfaults
# on macOS for some devices (Thrustmaster T.Flight Stick X confirmed) as soon
# as a HID input report comes in (crash in libSDL2's hid_report_callback,
# reached from pygame.event.pump() — see macOS crash report, faulting thread
# through IOHIDDeviceInputReportApplier). Forcing the legacy IOKit joystick
# backend avoids that codepath entirely.
os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "0")

import pygame
import zenoh
from websockets.sync.client import connect as ws_connect

PUBLISH_PERIOD = 0.02  # 50 Hz
DEADMAN_BUTTON = 0     # joystick button index used as deadman

ARM_WS_PORT = 8767            # robot/arm_agent.py's own WebSocket (not Zenoh)
ARM_RECONNECT_INTERVAL = 2.0  # don't retry more often than this while the arm link is down
ARM_WS_OPEN_TIMEOUT = 1.0     # bound the worst-case stall of the 50Hz loop on a dead robot IP

# Axes (Thrustmaster T.Flight Stick X: 0=X, 1=Y, 2=twist/rudder, 3=throttle).
AXIS_STICK_X = 0
AXIS_STICK_Y = 1
AXIS_ROTATION = 2
STICK_X_INVERT = False
DEADZONE = 0.25           # stick: below this magnitude -> STOP, not drift
ROTATION_DEADZONE = 0.15  # twist axis: below this -> no rotation, not drift

# GELLO integration point. Provide a reader that returns the latest raw
# GELLO firmware line (str, untouched) or None when no GELLO is connected.
try:
    from gello_reader import read_gello_raw_line  # type: ignore
except ImportError:
    def read_gello_raw_line():
        return None


# 8 snapped directions, 45° apart: (forward, lateral, label). Matches
# mecanum_control.py's SNAP_TABLE exactly (there named vy=forward,
# vx=lateral; renamed here to this project's vx=forward/vy=lateral
# convention -- same physical mapping, no sign changes).
_D = 1 / math.sqrt(2)
SNAP_TABLE = [
    (  0.0, +1.0, "strafe droite"),    # secteur 0 -> E    0°
    ( +_D,  +_D,  "diag av-droite"),   # secteur 1 -> NE  45°
    ( +1.0,  0.0, "avant"),            # secteur 2 -> N   90°
    ( +_D,  -_D,  "diag av-gauche"),   # secteur 3 -> NW 135°
    (  0.0, -1.0, "strafe gauche"),    # secteur 4 -> W  180°
    ( -_D,  -_D,  "diag ar-gauche"),   # secteur 5 -> SW 225°
    ( -1.0,  0.0, "recule"),           # secteur 6 -> S  270°
    ( -_D,  +_D,  "diag ar-droite"),   # secteur 7 -> SE 315°
]


def snap_direction(joy: pygame.joystick.JoystickType) -> tuple[float, float, str]:
    """Quantize the stick angle into one of the 8 sectors above.

    Returns (0.0, 0.0, "stop") inside the deadzone.
    """
    sx = -joy.get_axis(AXIS_STICK_X) if STICK_X_INVERT else joy.get_axis(AXIS_STICK_X)
    sy = -joy.get_axis(AXIS_STICK_Y)

    if math.hypot(sx, sy) < DEADZONE:
        return 0.0, 0.0, "stop"

    angle = math.degrees(math.atan2(sy, sx)) % 360
    sector = round(angle / 45) % 8
    return SNAP_TABLE[sector]


def read_rotation(joy: pygame.joystick.JoystickType) -> float:
    raw = joy.get_axis(AXIS_ROTATION)
    return 0.0 if abs(raw) < ROTATION_DEADZONE else raw


def load_config() -> zenoh.Config:
    path = Path(__file__).resolve().parent.parent / "config" / "operator_zenoh.json5"
    return zenoh.Config.from_file(str(path))


class ArmLink:
    """Direct WebSocket connection to robot/arm_agent.py, replacing the old
    Zenoh robot/cmd/arm publisher. Connects lazily (only once a GELLO
    reading actually needs sending) and reconnects on failure, throttled to
    ARM_RECONNECT_INTERVAL so a robot that's down/unreachable can't stall
    this process's 50Hz loop (bounded by ARM_WS_OPEN_TIMEOUT per attempt,
    not the default ~10s) -- a dropped/failed arm link degrades to "GELLO
    commands are silently not delivered", never to "the base stops
    responding too".
    """

    def __init__(self, robot_ip: str):
        self.url = f"ws://{robot_ip}:{ARM_WS_PORT}"
        self._ws = None
        self._last_attempt = 0.0

    def send_raw_line(self, line: str) -> None:
        if self._ws is None:
            now = time.time()
            if now - self._last_attempt < ARM_RECONNECT_INTERVAL:
                return
            self._last_attempt = now
            try:
                self._ws = ws_connect(self.url, open_timeout=ARM_WS_OPEN_TIMEOUT, close_timeout=0.5)
                print(f"input_agent: connected to arm link at {self.url}")
            except Exception as exc:
                print(f"[input_agent] arm link connect failed ({self.url}): {exc}")
                return
        try:
            self._ws.send(json.dumps({"raw": line}))
        except Exception as exc:
            print(f"[input_agent] arm link send failed: {exc}")
            self.close()

    def close(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None


def main() -> None:
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        # Not fatal: the browser UI (Gamepad API + Web Serial, see
        # operator/web/index.html) can drive both the base and the GELLO on
        # its own now. start_operator.sh backgrounds this process precisely
        # so that having nothing to do here doesn't take zenohd/web_server
        # down with it -- exit clean (not an exception) so that shows up as
        # a plain informational line, not a traceback.
        print("input_agent: no joystick detected -- nothing to do (drive from the browser "
              "instead). Exiting.")
        return
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()}")

    arm_link: ArmLink | None = None
    warned_no_robot_ip = False

    with zenoh.open(load_config()) as session:
        pub_base = session.declare_publisher("robot/cmd/base")
        pub_deadman = session.declare_publisher("operator/deadman")
        pub_joystick = session.declare_publisher("operator/input/joystick")

        print("input_agent running. Hold the deadman button to enable motion.")
        try:
            while True:
                pygame.event.pump()
                deadman = bool(joy.get_button(DEADMAN_BUTTON))

                # GELLO arm teleop is intentionally independent of the base's
                # joystick deadman -- operating a 7-DOF leader arm needs both
                # hands, so requiring the joystick button held at the same
                # time isn't workable. Safety for the arm instead comes from
                # the robot-side watchdog on the arm link's freshness.
                gello_line = read_gello_raw_line()
                if gello_line is not None:
                    if arm_link is None:
                        robot_ip = os.environ.get("ROBOT_IP")
                        if robot_ip:
                            arm_link = ArmLink(robot_ip)
                        elif not warned_no_robot_ip:
                            print("[input_agent] GELLO detected but ROBOT_IP is not set -- "
                                  "arm commands will be dropped. Set ROBOT_IP=<robot ip> to "
                                  "enable arm teleop.", flush=True)
                            warned_no_robot_ip = True
                    if arm_link is not None:
                        arm_link.send_raw_line(gello_line)

                if not deadman:
                    pub_base.put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
                    pub_deadman.put("false")
                    time.sleep(PUBLISH_PERIOD)
                    continue

                pub_deadman.put("true")

                vx, vy, label = snap_direction(joy)
                wz = read_rotation(joy)

                base_cmd = {"vx": float(vx), "vy": float(vy), "wz": float(wz),
                            "ts": time.time()}
                pub_base.put(json.dumps(base_cmd))
                pub_joystick.put(json.dumps(base_cmd))
                print(f"\r{label:16s} vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}   ", end="")

                time.sleep(PUBLISH_PERIOD)
        except KeyboardInterrupt:
            pass
        finally:
            # Best effort: tell the robot to stop as we leave.
            pub_base.put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
            pub_deadman.put("false")
            if arm_link is not None:
                arm_link.close()
            print("\ninput_agent stopped.")


if __name__ == "__main__":
    main()
