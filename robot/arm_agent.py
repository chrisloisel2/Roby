#!/usr/bin/env python3
"""Arm-side agent: drives the reBot B601 follower arm.

Deliberately a SEPARATE process from robot_agent.py (mecanum base), never
imported into it. `RebotB601Follower` (reused as-is from the `lerobot`
project already proven on this exact arm -- see
`~/03_JelloSoft/rebot_lerobot/lerobot/src/lerobot/robots/rebot_b601_follower/`
on this machine) pulls in the full lerobot package and its dependency chain
(torch, etc.), which has no business anywhere near robot_agent.py's lean
`.venv --system-site-packages`, already-validated-on-hardware 100Hz base
control loop. This file MUST run with the `lerobot` conda env's python (see
scripts/start_arm.sh), not the project's own .venv.

Command contract
----------------
robot/cmd/arm     JSON {"joints": {name: deg, ...}, "gripper": float, "mode": str}.
                  Joints are ALREADY leader-calibrated into the follower's
                  frame by operator/gello_reader.py -- this process applies
                  them close to directly (soft joint-limit clip and a
                  max_relative_target safety cap happen inside
                  RebotB601Follower.send_action(), nothing else). Ignored
                  unless "mode" == "joint_position".
robot/cmd/stop    Any payload -> latching emergency stop. Shared topic with
                  robot_agent.py: one E-stop button/command kills both the
                  base and the arm.
robot/cmd/reset   Clears the E-stop latch and re-enables the arm motors.
                  Shared topic with robot_agent.py, same "reset re-arms,
                  doesn't itself cause motion" semantics.

Publishes
---------
robot/arm/state   JSON status snapshot, ~5 Hz:
                  {connected, moving, fresh_cmd, estop, joints, ts}.

Safety
------
Independent of the base's deadman by design: teleoperating a 7-DOF leader
arm needs both hands, so requiring the base's joystick button held down at
the same time isn't workable (see operator/input_agent.py). Motion instead
gates on robot/cmd/arm freshness (ARM_CMD_TIMEOUT_SEC) -- this process has
its own watchdog, entirely independent of robot_agent.py's. A stale/missing
command means "stop sending new targets", not "actively zero" -- the Damiao
MIT/POS_VEL modes already hold their last commanded position on their own,
so there is nothing analogous to the base's stop_robot() ramp-to-zero here.
"""
import json
import os
import threading
import time

import zenoh

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerRobotConfig

# --- Safety parameters -------------------------------------------------------
ARM_CMD_TIMEOUT_SEC = 0.3  # stop sending new targets if no fresh robot/cmd/arm
CONTROL_PERIOD = 0.02      # 50 Hz -- matches the hz already proven on this arm
                            # (see ~/03_JelloSoft/rebot_lerobot/scripts/gello_follow.py)
HEARTBEAT_PERIOD = 0.2     # 5 Hz heartbeat / state, incl. a present-position read

# --- Arm connection -----------------------------------------------------------
ARM_PORT = os.environ.get("ARM_PORT", "/dev/ttyACM0")
ARM_ID = "follower"  # must match the calibration file already generated on this
                       # machine (~/.cache/huggingface/lerobot/calibration/robots/
                       # rebot_b601_follower/follower.json)

# Extra software safety net for this arm's first time being driven through
# THIS code path (on top of the smoothing already applied leader-side in
# gello_reader.py): caps how far a single send_action() call may move any
# joint from its last observed position. Catches a bad/glitched leader
# reading (e.g. a large jump right after connecting, before the leader's own
# filter has settled) rather than translating it into a fast physical move.
# Not meant to be the primary safety mechanism -- GELLO's own leader_smooth
# is -- just a ceiling.
ARM_MAX_RELATIVE_TARGET_DEG = 3.0


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.arm_cmd: dict | None = None
        self.last_arm_ts = 0.0
        self.estop = False  # latching


state = State()


def load_config() -> zenoh.Config:
    from pathlib import Path

    path = Path(__file__).resolve().parent.parent / "config" / "robot_zenoh.json5"
    config = zenoh.Config.from_file(str(path))
    operator_ip = os.environ.get("OPERATOR_IP")
    if operator_ip:
        config.insert_json5("connect/endpoints", json.dumps([f"tcp/{operator_ip}:7447"]))
    return config


def _build_action(cmd: dict) -> dict[str, float]:
    action = {f"{name}.pos": float(deg) for name, deg in cmd.get("joints", {}).items()}
    gripper = cmd.get("gripper")
    if gripper is not None:
        action["gripper.pos"] = float(gripper)
    return action


# --- Zenoh subscribers (callbacks) ------------------------------------------

def on_arm(sample) -> None:
    try:
        cmd = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return
    if cmd.get("mode") != "joint_position":
        return
    with state.lock:
        state.arm_cmd = cmd
        state.last_arm_ts = time.time()


def on_stop(_sample, follower: RebotB601Follower) -> None:
    with state.lock:
        already = state.estop
        state.estop = True
    if not already:
        try:
            follower.disable_torque()
        except Exception as exc:
            print(f"[STOP] disable_torque failed: {exc}")
    print("[STOP] Arm emergency stop latched (robot/cmd/reset to clear).")


def on_reset(_sample, follower: RebotB601Follower) -> None:
    with state.lock:
        state.estop = False
    try:
        follower.configure()  # re-enables torque and re-applies control modes
        print("[RESET] Arm motors re-enabled, estop cleared.")
    except Exception as exc:
        print(f"[RESET] configure() failed: {exc}")


def main() -> None:
    print(f"arm_agent: connecting to reBot B601 follower on {ARM_PORT} (id={ARM_ID})...")
    config = RebotB601FollowerRobotConfig(
        port=ARM_PORT,
        id=ARM_ID,
        max_relative_target=ARM_MAX_RELATIVE_TARGET_DEG,
    )
    follower = RebotB601Follower(config)
    follower.connect(calibrate=False)  # never block on stdin in a headless service
    if not follower.is_calibrated:
        raise RuntimeError(
            f"reBot B601 follower has no calibration file matching id={ARM_ID!r} -- "
            "refusing to run uncalibrated. Run `lerobot-calibrate "
            f"--robot.type=rebot_b601_follower --robot.port={ARM_PORT} --robot.id={ARM_ID}` "
            "once first (see ~/03_JelloSoft/rebot_lerobot/scripts/README.md)."
        )
    print("arm_agent: follower connected and calibrated.")

    try:
        with zenoh.open(load_config()) as session:
            session.declare_subscriber("robot/cmd/arm", on_arm)
            session.declare_subscriber("robot/cmd/stop", lambda s: on_stop(s, follower))
            session.declare_subscriber("robot/cmd/reset", lambda s: on_reset(s, follower))

            pub_state = session.declare_publisher("robot/arm/state")

            print("arm_agent running. Waiting for robot/cmd/arm...")
            last_beat = 0.0

            try:
                while True:
                    now = time.time()
                    with state.lock:
                        cmd = state.arm_cmd
                        estop = state.estop
                        fresh_cmd = (now - state.last_arm_ts) < ARM_CMD_TIMEOUT_SEC

                    moving = fresh_cmd and not estop and cmd is not None
                    if moving:
                        try:
                            follower.send_action(_build_action(cmd))
                        except Exception as exc:
                            print(f"[arm_agent] send_action failed: {exc}")

                    if now - last_beat >= HEARTBEAT_PERIOD:
                        last_beat = now
                        try:
                            joints = {
                                k.removesuffix(".pos"): v
                                for k, v in follower.get_observation().items()
                            }
                        except Exception as exc:
                            print(f"[arm_agent] get_observation failed: {exc}")
                            joints = {}
                        pub_state.put(json.dumps({
                            "connected": follower.is_connected,
                            "moving": moving,
                            "fresh_cmd": fresh_cmd,
                            "estop": estop,
                            "joints": joints,
                            "ts": now,
                        }))

                    time.sleep(CONTROL_PERIOD)
            except KeyboardInterrupt:
                pass
    finally:
        try:
            follower.disconnect()
            print("\narm_agent stopped, arm torque disabled.")
        except Exception as exc:
            print(f"[shutdown] disconnect failed: {exc}")


if __name__ == "__main__":
    main()
