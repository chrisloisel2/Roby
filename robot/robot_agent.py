#!/usr/bin/env python3
"""Robot-side agent.

Receives Zenoh commands, applies them to the robot, and enforces the LOCAL
safety watchdog. This process is the last line of defense: the robot must
stop whenever it loses fresh, deadman-authorized commands, regardless of what
the operator PC or web server are doing.

Command contract
----------------
robot/cmd/base    JSON {"vx","vy","wz"} with each axis normalized to [-1, 1].
                  This agent clamps to [-1, 1] then scales by MAX_LINEAR /
                  MAX_ANGULAR before actuation.
robot/cmd/arm     JSON {"joints":[...], "gripper": float, "mode": str}
robot/cmd/gripper JSON {"gripper": float in [0, 1]}
robot/cmd/stop    Any payload -> latching emergency stop (requires restart).
operator/deadman  "true" / "false" -- must be "true" and fresh to move.

Publishes
---------
robot/heartbeat   Liveness beacon, ~5 Hz.
robot/state       JSON status snapshot, ~5 Hz.
"""
import json
import os
import threading
import time
from pathlib import Path

import zenoh

# --- Safety parameters -------------------------------------------------------
CMD_TIMEOUT_SEC = 0.3      # stop if no fresh base command within this window
DEADMAN_TIMEOUT_SEC = 0.3  # stop if no fresh deadman "true" within this window
CONTROL_PERIOD = 0.01      # 100 Hz control loop
HEARTBEAT_PERIOD = 0.2     # 5 Hz heartbeat / state
MAX_LINEAR = 0.6           # m/s   clamp for base translation
MAX_ANGULAR = 1.5          # rad/s clamp for base rotation


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


# --- Robot hardware interface (stubs) ---------------------------------------
# Replace the bodies below with your real robot driver calls.

def stop_robot() -> None:
    # Idempotent: safe to call every control tick. Send zero velocity and
    # disable dangerous actuators.
    pass  # TODO: robot.set_base_velocity(0, 0, 0); robot.hold_arm()


def apply_base_command(vx: float, vy: float, wz: float) -> None:
    # vx, vy in m/s; wz in rad/s -- already clamped/scaled by the caller.
    pass  # TODO: robot.set_base_velocity(vx, vy, wz)


def apply_arm_command(cmd: dict) -> None:
    pass  # TODO: robot.command_arm(cmd["joints"], cmd.get("gripper"), cmd.get("mode"))


def apply_gripper_command(value: float) -> None:
    # value in [0, 1]: 0 = open, 1 = closed.
    pass  # TODO: robot.set_gripper(value)


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
        state.estop = True
    print("[STOP] Emergency stop latched. Restart robot_agent.py to clear.")


def on_deadman(sample) -> None:
    value = sample.payload.to_bytes().decode("utf-8").strip().lower()
    with state.lock:
        state.deadman = value == "true"
        state.last_deadman_ts = time.time()


def main() -> None:
    with zenoh.open(load_config()) as session:
        session.declare_subscriber("robot/cmd/base", on_base)
        session.declare_subscriber("robot/cmd/arm", on_arm)
        session.declare_subscriber("robot/cmd/gripper", on_gripper)
        session.declare_subscriber("robot/cmd/stop", on_stop)
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
                    vx = _clamp(float(base.get("vx", 0.0)), 1.0) * MAX_LINEAR
                    vy = _clamp(float(base.get("vy", 0.0)), 1.0) * MAX_LINEAR
                    wz = _clamp(float(base.get("wz", 0.0)), 1.0) * MAX_ANGULAR
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


if __name__ == "__main__":
    main()
