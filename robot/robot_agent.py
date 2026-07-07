#!/usr/bin/env python3
"""Robot-side agent.

Receives Zenoh commands, applies them to the robot, and enforces the LOCAL
safety watchdog. This process is the last line of defense: the robot must
stop whenever it loses fresh, deadman-authorized commands, regardless of what
the operator PC or web server are doing.

Drives the 4 mecanum-wheel DAMIAO DMS2325 motors directly over USB-CAN via
the vendor `dmcan`/`damiao` SDK (libusb, no ROS2) -- the same driver path
validated on this robot's hardware (real spin test, real feedback) in
~/catkin_ws/u2canfd/mecanum_control.py. `damiao.py` and `dlls/libdm_device.so`
are vendored alongside this file; `dmcan_sdk` and `pyusb` are pip packages
(requirements.txt).

Command contract
----------------
robot/cmd/base    JSON {"vx","vy","wz"}, each normalized to [-1, 1].
                  vx = forward/back, vy = lateral (right +), wz = rotation
                  (CCW +). Mixed into 4 wheel velocities (mecanum kinematics)
                  scaled by MAX_VEL / ROT_VEL (empirically tuned on this
                  robot, matching mecanum_control.py -- not a physical
                  m/s/rad/s unit conversion; wheel radius was never reliably
                  measured, see catkin_ws/u2canfd/CLAUDE.md).
robot/cmd/arm     JSON {"joints":[...], "gripper": float, "mode": str}
                  -- no arm/mast hardware exists yet: no-op, kept as a
                  documented extension point.
robot/cmd/gripper JSON {"gripper": float in [0, 1]} -- no gripper hardware
                  yet: no-op, kept as a documented extension point.
robot/cmd/stop    Any payload -> latching emergency stop (requires restart).
                  Edge-triggered hard motor disable_all(), matching the
                  proven emergency-stop behavior in mecanum_control.py.
operator/deadman  "true" / "false" -- must be "true" and fresh to move.

Publishes
---------
robot/heartbeat   Liveness beacon, ~5 Hz.
robot/state       JSON status snapshot, ~5 Hz.
"""
import json
import os
import sys
import threading
import time
from pathlib import Path

import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dmcan import dmcan_device_type
from damiao import DM_Motor_Type, Control_Mode, DmActData, Motor_Control

# --- Safety parameters -------------------------------------------------------
CMD_TIMEOUT_SEC = 0.3      # stop if no fresh base command within this window
DEADMAN_TIMEOUT_SEC = 0.3  # stop if no fresh deadman "true" within this window
CONTROL_PERIOD = 0.01      # 100 Hz control loop
HEARTBEAT_PERIOD = 0.2     # 5 Hz heartbeat / state

# --- Motor driver (real hardware, mecanum) -----------------------------------
# Same values validated on this robot's hardware (real spin test, 2026-07-06):
# see ~/catkin_ws/u2canfd/mecanum_control.py / CLAUDE.md.
SN = "14AA044B241402B10DDBDAFE448040BB"
#         FR     FL     RL     RR
CAN_IDS = [0x01, 0x02, 0x03, 0x04]
MST_IDS = [0x11, 0x12, 0x13, 0x14]
INVERT  = [+1,   -1,   -1,   +1]
KD = 3.0
MAX_VEL = 60.0   # rad/s -- translation wheel-speed scale (empirically tuned)
ROT_VEL = 50.0   # rad/s -- rotation wheel-speed scale (empirically tuned)

_ctrl: Motor_Control | None = None
_motors: list | None = None


def load_config() -> zenoh.Config:
    """Load the robot client config, honoring an OPERATOR_IP env override."""
    path = Path(__file__).resolve().parent.parent / "config" / "robot_zenoh.json5"
    config = zenoh.Config.from_file(str(path))
    operator_ip = os.environ.get("OPERATOR_IP")
    if operator_ip:
        config.insert_json5("connect/endpoints", json.dumps([f"tcp/{operator_ip}:7447"]))
    return config


class State:
    """Latest inputs, guarded by a lock and stamped with arrival time."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.base_cmd = None
        self.arm_cmd = None
        self.gripper_cmd = None
        self.deadman = False
        self.last_base_ts = 0.0
        self.last_deadman_ts = 0.0
        self.estop = False  # latching


state = State()


# --- Robot hardware interface -------------------------------------------------

def _init_motors() -> None:
    global _ctrl, _motors
    init_data = [
        DmActData(motorType=DM_Motor_Type.DMS2325, mode=Control_Mode.MIT_MODE,
                  can_id=can_id, mst_id=mst_id)
        for can_id, mst_id in zip(CAN_IDS, MST_IDS)
    ]
    # NOT a `with` block: Motor_Control.close() / DmCanContext.__del__ crash
    # the process (libusb assertion, native lib bug) -- see shutdown in
    # main(): disable_all() then os._exit() skips this entirely. Confirmed
    # safe: disable_all() always completes before the native cleanup path
    # would run.
    _ctrl = Motor_Control(
        1_000_000, 1_000_000, SN, init_data,
        device_type=dmcan_device_type.USB2CANFD, canfd=False, brs=False,
    )
    _motors = [_ctrl.getMotor(can_id) for can_id in CAN_IDS]


def _mecanum_targets(vx: float, vy: float, wz: float) -> list[float]:
    """[-1,1] vx/vy/wz -> [FR, FL, RL, RR] wheel rad/s, before INVERT."""
    tx, ty, tz = vx * MAX_VEL, vy * MAX_VEL, wz * ROT_VEL
    return [tx - ty - tz, tx + ty + tz, tx - ty + tz, tx + ty - tz]


def stop_robot() -> None:
    # Idempotent, safe every control tick: hold zero velocity (motors stay
    # enabled). Called whenever `moving` is False for any reason (deadman
    # released, stale command). See on_stop() for the harder E-stop path.
    if _ctrl is None or _motors is None:
        return
    for motor in _motors:
        try:
            _ctrl.control_mit(motor, 0.0, KD, 0.0, 0.0, 0.0)
        except Exception as exc:
            print(f"[stop_robot] control_mit failed: {exc}")


def apply_base_command(vx: float, vy: float, wz: float) -> None:
    if _ctrl is None or _motors is None:
        return
    targets = _mecanum_targets(vx, vy, wz)
    for motor, target, inv in zip(_motors, targets, INVERT):
        try:
            _ctrl.control_mit(motor, 0.0, KD, 0.0, target * inv, 0.0)
        except Exception as exc:
            print(f"[apply_base_command] control_mit failed: {exc}")


def apply_arm_command(_cmd: dict) -> None:
    pass  # No arm/mast hardware yet -- extension point.


def apply_gripper_command(_value: float) -> None:
    pass  # No gripper hardware yet -- extension point.


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


# --- Zenoh subscribers (callbacks) ------------------------------------------

def on_base(sample) -> None:
    try:
        cmd = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return
    with state.lock:
        state.base_cmd = cmd
        state.last_base_ts = time.time()


def on_arm(sample) -> None:
    try:
        cmd = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return
    with state.lock:
        state.arm_cmd = cmd


def on_gripper(sample) -> None:
    try:
        cmd = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return
    with state.lock:
        state.gripper_cmd = float(cmd.get("gripper", 0.0))


def on_stop(_sample) -> None:
    with state.lock:
        already = state.estop
        state.estop = True
    if not already:
        # Edge-triggered hard stop: disable_all() (0xFD x5/moteur) is more
        # reliable than a zero-velocity command -- matches the proven
        # emergency-stop behavior in mecanum_control.py. Runs on the Zenoh
        # callback thread; disable_all() has no dependency on the control
        # loop's own state so this is safe to call directly here.
        if _ctrl is not None:
            try:
                _ctrl.disable_all()
            except Exception as exc:
                print(f"[STOP] disable_all failed: {exc}")
    print("[STOP] Emergency stop latched. Restart robot_agent.py to clear.")


def on_reset(_sample) -> None:
    # Clears the E-stop latch and re-enables the motors. Does NOT itself
    # cause motion: `moving` still requires a fresh, deadman-authorized
    # command (see main loop) -- mirrors "reset re-arms, deadman still
    # required to move" rather than any auto-resume.
    with state.lock:
        state.estop = False
    if _ctrl is not None:
        try:
            _ctrl.enable_all()
            print("[RESET] Motors re-enabled, estop cleared.")
        except Exception as exc:
            print(f"[RESET] enable_all failed: {exc}")


def on_deadman(sample) -> None:
    value = sample.payload.to_bytes().decode("utf-8").strip().lower()
    with state.lock:
        state.deadman = value == "true"
        state.last_deadman_ts = time.time()


def main() -> None:
    print("robot_agent: initializing motors...")
    _init_motors()
    print("robot_agent: motors ready.")

    try:
        with zenoh.open(load_config()) as session:
            session.declare_subscriber("robot/cmd/base", on_base)
            session.declare_subscriber("robot/cmd/arm", on_arm)
            session.declare_subscriber("robot/cmd/gripper", on_gripper)
            session.declare_subscriber("robot/cmd/stop", on_stop)
            session.declare_subscriber("robot/cmd/reset", on_reset)
            session.declare_subscriber("operator/deadman", on_deadman)

            pub_heartbeat = session.declare_publisher("robot/heartbeat")
            pub_state = session.declare_publisher("robot/state")

            print("robot_agent running. Waiting for deadman-authorized commands...")
            last_beat = 0.0

            try:
                while True:
                    now = time.time()
                    with state.lock:
                        base = state.base_cmd
                        arm = state.arm_cmd
                        grip = state.gripper_cmd
                        estop = state.estop
                        fresh_cmd = (now - state.last_base_ts) < CMD_TIMEOUT_SEC
                        deadman_ok = state.deadman and (
                            now - state.last_deadman_ts
                        ) < DEADMAN_TIMEOUT_SEC

                    moving = deadman_ok and fresh_cmd and not estop and base is not None
                    if moving:
                        vx = _clamp(float(base.get("vx", 0.0)), 1.0)
                        vy = _clamp(float(base.get("vy", 0.0)), 1.0)
                        wz = _clamp(float(base.get("wz", 0.0)), 1.0)
                        apply_base_command(vx, vy, wz)
                        if arm is not None:
                            apply_arm_command(arm)
                        if grip is not None:
                            apply_gripper_command(grip)
                    else:
                        stop_robot()

                    if now - last_beat >= HEARTBEAT_PERIOD:
                        last_beat = now
                        pub_heartbeat.put(str(now))
                        pub_state.put(json.dumps({
                            "moving": moving,
                            "estop": estop,
                            "deadman_ok": deadman_ok,
                            "fresh_cmd": fresh_cmd,
                            "ts": now,
                        }))

                    time.sleep(CONTROL_PERIOD)
            except KeyboardInterrupt:
                pass
            finally:
                stop_robot()
                print("\nrobot_agent stopped, robot commanded to halt.")
        # Zenoh session cleanly closed above (its own __exit__ is not the
        # buggy one) before we touch motor shutdown below.
    finally:
        try:
            if _ctrl is not None:
                _ctrl.disable_all()
        except Exception as exc:
            print(f"[shutdown] disable_all failed: {exc}")
        # os._exit(): skips Motor_Control/DmCanContext.__del__, which
        # crashes the process (libusb assertion in the native lib's cleanup
        # path) -- confirmed on this hardware that disable_all() above
        # always completes first, so motors are already safely stopped.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
