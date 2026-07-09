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

Deliberately mirrors `~/03_JelloSoft/rebot_lerobot/scripts/start_teleoperation.py`
(this project's own known-good GELLO teleoperation script) as closely as
possible: `make_robot_from_config()` to build the follower, and every tick
runs the SAME `make_default_processors()` pipeline that script does --
`obs = follower.get_observation()`, `teleop_action_processor((raw_action, obs))`,
`robot_action_processor((teleop_action, obs))`, `follower.send_action(robot_action)`.
The only thing that differs is where `raw_action` comes from: that script
reads it from `teleop.get_action()` (a GELLO leader plugged into THIS
machine's own serial port); here it comes from the WebSocket instead (the
leader is read remotely -- browser Web Serial or input_agent.py -- and
already produces the same `{name.pos: deg, ...}` shape
`GelloAs5600RawLeader.get_action()` does, so it drops into `raw_action`
unchanged). No teleoperator object is constructed here for that reason --
there is no local leader to connect to.

Command contract
----------------
WebSocket ws://<robot-ip>:ARM_WS_PORT   JSON text messages:
                  {"joints": {name: deg, ...}, "gripper": float, "mode": str}.
                  Sent DIRECTLY by the browser (operator/web/static/js/armLink.js),
                  bypassing Zenoh + web_server.py's /ws/control relay --
                  same "direct WebSocket, one fewer hop" pattern as
                  camera_pub.py's video, but its OWN connection/port, not
                  shared with the camera: this process needs the `lerobot`
                  conda env, whose own opencv-python has NO GStreamer
                  support (confirmed empirically 2026-07-09 -- same failure
                  mode already documented in requirements.txt/README for a
                  generic PyPI opencv-python wheel), so it can't share a
                  process with camera_pub.py (which needs system cv2 with
                  GStreamer) without breaking one of the two.

                  Joints are ALREADY leader-calibrated into the follower's
                  frame by the browser's own GELLO/Web Serial reader (a
                  port of operator/gello_reader.py's math -- see
                  index.html) -- this process applies them close to
                  directly (soft joint-limit clip and a max_relative_target
                  safety cap happen inside RebotB601Follower.send_action(),
                  nothing else). Ignored unless "mode" == "joint_position".
                  This is the exact same contract robot/cmd/arm carried
                  before -- only the transport changed, Zenoh -> WebSocket.
robot/cmd/stop    (Zenoh, UNCHANGED) Any payload -> latching emergency stop.
                  Shared topic with robot_agent.py: one E-stop
                  button/command kills both the base and the arm. Left on
                  Zenoh deliberately -- moving it to this WebSocket would
                  decouple that shared E-stop.
robot/cmd/reset   (Zenoh, UNCHANGED) Clears the E-stop latch and re-enables
                  the arm motors. Shared topic with robot_agent.py, same
                  "reset re-arms, doesn't itself cause motion" semantics.

Publishes
---------
robot/arm/state   (Zenoh, UNCHANGED) JSON status snapshot, ~5 Hz:
                  {connected, moving, fresh_cmd, estop, joints, ts}.

Safety
------
Independent of the base's deadman by design: teleoperating a 7-DOF leader
arm needs both hands, so requiring the base's joystick button held down at
the same time isn't workable (see operator/input_agent.py). Motion instead
gates on the WebSocket command's freshness (ARM_CMD_TIMEOUT_SEC) -- this
process has its own watchdog, entirely independent of robot_agent.py's. A
stale/missing command means "stop sending new targets", not "actively
zero" -- the Damiao MIT/POS_VEL modes already hold their last commanded
position on their own, so there is nothing analogous to the base's
stop_robot() ramp-to-zero here.

Threading model: the 50Hz send_action()/get_observation() control loop runs
in a background thread (same split as robot/uvc_camera_server.py's
CameraCapture) so the ~1ms-scale CAN I/O it does never blocks the asyncio
WebSocket server (running on the main thread) from receiving the next
joint command. The Zenoh subscribers (stop/reset) run on Zenoh's own
internal callback thread, same as before. All three touch the shared
`state` object only under `state.lock`.
"""
import asyncio
import json
import os
import sys
import threading
import time
from pathlib import Path

import websockets
import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from zenoh_config import load_robot_config

from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerRobotConfig
from lerobot.utils.robot_utils import precise_sleep

# --- Safety parameters -------------------------------------------------------
ARM_CMD_TIMEOUT_SEC = 0.3  # stop sending new targets if no fresh WS command
CONTROL_PERIOD = 0.02      # 50 Hz -- matches the hz already proven on this arm
                            # (see ~/03_JelloSoft/rebot_lerobot/scripts/gello_follow.py)
HEARTBEAT_PERIOD = 0.2     # 5 Hz heartbeat / state, incl. a present-position read

ARM_WS_HOST = "0.0.0.0"
ARM_WS_PORT = 8767

running = True  # set False on shutdown to stop the control-loop thread

# --- Arm connection -----------------------------------------------------------
# NOT a bare /dev/ttyACM0: USB-serial enumeration order isn't stable across
# reboots on this machine, so a plain "ttyACM0" can silently point at a
# different physical device after a reboot/replug -- same class of bug as the
# camera's /dev/videoN probing (see camera_pub.py), fixed the same way Linux
# fixes it for USB-serial: address by the kernel's own stable by-id symlink
# (vendor+product+serial), immune to enumeration order.
#
# The board's USB descriptor reports vendor "HDSC" / model "CDC Device" --
# NOT a "DaMiao-Tech" string, despite the CAN bridge being a Damiao part
# internally. An earlier version of this file assumed the HDSC device was a
# different, unrelated peripheral and hardcoded a "DaMiao-Tech_DM-USB2FDCAN"
# by-id path instead -- that device was never actually observed on this
# machine (confirmed 2026-07-08 via lsusb/udevadm monitor across several
# physical replugs), so robot/cmd/arm silently went nowhere. Verified
# correct the same day with a read-only motorbridge probe straight against
# this exact by-id path -- Controller.from_dm_serial() + add_damiao_motor()
# + request_feedback() (no enable(), no motion possible) got a real state
# reply from all 7 joints.
ARM_PORT = os.environ.get(
    "ARM_PORT",
    "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00",
)
ARM_ID = "follower"  # must match the calibration file already generated on this
                       # machine (~/.cache/huggingface/lerobot/calibration/robots/
                       # rebot_b601_follower/follower.json)

# Extra software safety net for this arm's first time being driven through
# THIS code path (on top of the smoothing already applied leader-side in the
# browser's GELLO reader): caps how far a single send_action() call may move
# any joint from its last observed position. Catches a bad/glitched leader
# reading (e.g. a large jump right after connecting, before the leader's own
# filter has settled) rather than translating it into a fast physical move.
# Not meant to be the primary safety mechanism -- GELLO's own leader-side
# smoothing is -- just a ceiling.
ARM_MAX_RELATIVE_TARGET_DEG = 3.0

# --- Optional per-run joint_limits override -----------------------------------
# RebotB601FollowerRobotConfig.joint_limits (vendored, robot PC) was never
# actually measured -- calibrate() just copies the hardcoded config default
# straight into the calibration file, no range-of-motion sweep at all (see
# that file's configure()/calibrate() for the full story, confirmed
# 2026-07-08: at least shoulder_lift and gripper were clipped hard enough to
# be unusable in real teleop). operator/calibrate_arm_limits.py measures the
# real range by sweeping the GELLO leader by hand and saves it as JSON.
# Point ARM_JOINT_LIMITS_FILE at that file to use it for THIS run only,
# without ever touching the vendored file -- handy for trying out a fresh
# measurement before committing to it permanently.
_EXPECTED_ARM_JOINTS = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_yaw", "wrist_roll", "gripper",
}


def _load_joint_limits_override() -> dict[str, tuple[float, float]] | None:
    path = os.environ.get("ARM_JOINT_LIMITS_FILE")
    if not path:
        return None
    with open(path) as f:
        data = json.load(f)
    # Accept either calibrate_arm_limits.py's full record (which nests the
    # limits under "proposed_joint_limits") or a bare {name: [min, max]} file.
    limits = data.get("proposed_joint_limits", data)
    missing = _EXPECTED_ARM_JOINTS - set(limits)
    if missing:
        sys.exit(f"ARM_JOINT_LIMITS_FILE={path!r} is missing joints: {sorted(missing)}")
    result: dict[str, tuple[float, float]] = {}
    for name, bounds in limits.items():
        if name not in _EXPECTED_ARM_JOINTS:
            continue
        lo, hi = float(bounds[0]), float(bounds[1])
        if lo >= hi:
            sys.exit(f"ARM_JOINT_LIMITS_FILE={path!r}: {name} has min >= max ({lo}, {hi})")
        result[name] = (lo, hi)
    return result


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.arm_cmd: dict | None = None
        self.last_arm_ts = 0.0
        self.estop = False  # latching


state = State()


def _raw_action_from_cmd(cmd: dict) -> dict[str, float]:
    """The `raw_action` start_teleoperation.py would have gotten from
    `teleop.get_action()` -- same {name.pos: deg, ...} shape
    GelloAs5600RawLeader.get_action() returns (already leader-calibrated),
    just sourced from the WebSocket command instead of a local serial read.
    """
    action = {f"{name}.pos": float(deg) for name, deg in cmd.get("joints", {}).items()}
    gripper = cmd.get("gripper")
    if gripper is not None:
        action["gripper.pos"] = float(gripper)
    return action


# --- WebSocket handler (joint commands, direct from the browser) ------------

async def on_arm_ws(websocket) -> None:
    peer = websocket.remote_address
    print(f"[arm_agent] client connected from {peer}", flush=True)
    try:
        async for message in websocket:
            if not isinstance(message, str):
                continue
            try:
                cmd = json.loads(message)
            except ValueError:
                continue
            if cmd.get("mode") != "joint_position":
                continue
            with state.lock:
                state.arm_cmd = cmd
                state.last_arm_ts = time.time()
    except websockets.ConnectionClosed:
        print(f"[arm_agent] client {peer} disconnected", flush=True)


# --- Zenoh subscribers (E-stop / reset, unchanged) ---------------------------

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


# --- Control loop (background thread) ----------------------------------------

def control_loop(follower: RebotB601Follower, pub_state, teleop_action_processor, robot_action_processor) -> None:
    last_beat = 0.0
    while running:
        tick_start = time.time()
        now = tick_start
        with state.lock:
            cmd = state.arm_cmd
            estop = state.estop
            fresh_cmd = (now - state.last_arm_ts) < ARM_CMD_TIMEOUT_SEC

        moving = fresh_cmd and not estop and cmd is not None
        # Same per-tick shape as start_teleoperation.py's loop: obs ->
        # raw_action -> teleop_action_processor -> robot_action_processor ->
        # send_action(). teleop_action_processor/robot_action_processor come
        # from make_default_processors(), same call every lerobot script
        # uses -- currently both IdentityProcessorStep (no-ops) but going
        # through them (instead of building the .pos dict and calling
        # send_action() straight off it) means this keeps working unchanged
        # if lerobot's own default pipeline ever stops being a no-op.
        obs = None
        if moving:
            try:
                obs = follower.get_observation()
                raw_action = _raw_action_from_cmd(cmd)
                teleop_action = teleop_action_processor((raw_action, obs))
                robot_action = robot_action_processor((teleop_action, obs))
                follower.send_action(robot_action)
            except Exception as exc:
                print(f"[arm_agent] send_action failed: {exc}")

        if now - last_beat >= HEARTBEAT_PERIOD:
            last_beat = now
            try:
                if obs is None:  # not already fetched above this tick
                    obs = follower.get_observation()
                joints = {k.removesuffix(".pos"): v for k, v in obs.items()}
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

        # Elapsed-time-aware sleep, same as start_teleoperation.py's own
        # `precise_sleep(max(1 / fps - dt_s, 0.0))` -- a flat
        # time.sleep(CONTROL_PERIOD) would make the loop run SLOWER than
        # 50Hz by however long send_action()/get_observation() actually
        # took (silently -- nothing would ever indicate the loop had fallen
        # behind). Measured cost of send_action()'s CAN round-trip on this
        # hardware is ~1.2ms (negligible against the 20ms budget), so this
        # rarely matters in practice -- but the print below makes a future
        # regression (a flaky USB moment, a much heavier CAN load) visible
        # instead of just adding silent, unmeasured latency to every
        # GELLO->arm movement. precise_sleep() is a plain time.sleep() on
        # Linux (this robot) -- the spin-the-last-few-ms behavior it adds
        # only kicks in on macOS/Windows.
        elapsed = time.time() - tick_start
        if elapsed > CONTROL_PERIOD:
            print(f"[arm_agent] tick took {elapsed * 1000:.1f}ms "
                  f"(budget {CONTROL_PERIOD * 1000:.0f}ms) -- loop running behind", flush=True)
        precise_sleep(max(0.0, CONTROL_PERIOD - elapsed))


async def _serve_forever() -> None:
    async with websockets.serve(on_arm_ws, ARM_WS_HOST, ARM_WS_PORT, ping_interval=20):
        print(f"arm_agent: listening on ws://{ARM_WS_HOST}:{ARM_WS_PORT}", flush=True)
        await asyncio.Future()


def main() -> None:
    global running

    print(f"arm_agent: connecting to reBot B601 follower on {ARM_PORT} (id={ARM_ID})...")
    config_kwargs = dict(
        port=ARM_PORT,
        id=ARM_ID,
        max_relative_target=ARM_MAX_RELATIVE_TARGET_DEG,
    )
    joint_limits_override = _load_joint_limits_override()
    if joint_limits_override is not None:
        config_kwargs["joint_limits"] = joint_limits_override
        print("arm_agent: using CUSTOM joint_limits for this run only "
              f"(ARM_JOINT_LIMITS_FILE={os.environ['ARM_JOINT_LIMITS_FILE']!r}):")
        for name, (lo, hi) in joint_limits_override.items():
            print(f"    {name}: ({lo}, {hi})")
    config = RebotB601FollowerRobotConfig(**config_kwargs)
    # make_robot_from_config() just dispatches on config.type to
    # RebotB601Follower(config) -- functionally identical to constructing it
    # directly, done this way to match
    # ~/03_JelloSoft/rebot_lerobot/scripts/start_teleoperation.py's pattern.
    follower = make_robot_from_config(config)
    follower.connect(calibrate=False)  # never block on stdin in a headless service
    if not follower.is_calibrated:
        raise RuntimeError(
            f"reBot B601 follower has no calibration file matching id={ARM_ID!r} -- "
            "refusing to run uncalibrated. Run `lerobot-calibrate "
            f"--robot.type=rebot_b601_follower --robot.port={ARM_PORT} --robot.id={ARM_ID}` "
            "once first (see ~/03_JelloSoft/rebot_lerobot/scripts/README.md)."
        )
    print("arm_agent: follower connected and calibrated.")

    # Same make_default_processors() call start_teleoperation.py makes --
    # robot_observation_processor is unused there too (that script only
    # ever calls robot.get_observation() directly, same as here).
    teleop_action_processor, robot_action_processor, _robot_observation_processor = make_default_processors()

    thread = None
    try:
        with zenoh.open(load_robot_config("arm_agent")) as session:
            session.declare_subscriber("robot/cmd/stop", lambda s: on_stop(s, follower))
            session.declare_subscriber("robot/cmd/reset", lambda s: on_reset(s, follower))
            pub_state = session.declare_publisher("robot/arm/state")

            # Background thread (not asyncio): send_action()/get_observation()
            # do blocking CAN I/O -- running them as a coroutine on the same
            # loop as websockets.serve() would stall incoming-message
            # handling for however long each CAN round-trip takes.
            thread = threading.Thread(
                target=control_loop,
                args=(follower, pub_state, teleop_action_processor, robot_action_processor),
                daemon=True,
            )
            thread.start()

            print("arm_agent running. Waiting for arm WebSocket commands...")
            try:
                asyncio.run(_serve_forever())
            except KeyboardInterrupt:
                pass
    finally:
        running = False
        if thread is not None:
            # Let the control loop finish whatever tick it's mid-way through
            # (worst case ~CONTROL_PERIOD) before disconnect() runs on this
            # thread -- otherwise send_action() and disconnect() could race
            # on the follower object from two threads at once.
            thread.join(timeout=1.0)
        try:
            follower.disconnect()
            print("\narm_agent stopped, arm torque disabled.")
        except Exception as exc:
            print(f"[shutdown] disconnect failed: {exc}")


if __name__ == "__main__":
    main()
