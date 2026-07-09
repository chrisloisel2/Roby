#!/usr/bin/env python3
"""Arm-side agent: drives the reBot B601 follower arm.

Deliberately a SEPARATE process from robot_agent.py (mecanum base), never
imported into it. Pulls in the full lerobot package (torch, etc.) via
start_teleoperationV2.py below, which has no business anywhere near
robot_agent.py's lean `.venv --system-site-packages`, already-validated-
on-hardware 100Hz base control loop. This file MUST run with the `lerobot`
conda env's python (see scripts/start_arm.sh), not the project's own .venv.

2026-07-10: this file now DELEGATES ALL teleoperation logic to
robot/start_teleoperationV2.py's run_teleoperation() -- it does zero
reimplementation of its own. Why: two earlier versions of this file (their
own hand-rolled loops, one computing the calibrated GELLO action itself,
one manually poking a teleoperator object's internals to skip its own
connect()) both had real bugs, despite passing isolated/synthetic testing.
The user then independently confirmed a specific setup DOES work correctly
on the real hardware:

    socat TCP-LISTEN:9999,reuseaddr,fork OPEN:/dev/cu.usbserial-2130,raw,ispeed=115200,ospeed=115200,echo=0
    python start_teleoperation.py --teleop-port socket://<leader PC ip>:9999

i.e. the ORIGINAL, unmodified reference script
(~/03_JelloSoft/rebot_lerobot/scripts/start_teleoperation.py on this
machine), fed via a socat TCP-to-serial relay. start_teleoperationV2.py is
a minimal-diff variant of exactly that confirmed-working script -- kept
here in the Roby repo (git-tracked, single canonical copy) rather than
next to the original, so it's never a stale duplicate arm_agent.py
silently drifts out of sync with (see that file's own module docstring for
why that specific risk was worth calling out explicitly). The only
structural change from the confirmed-working script: teleop.connect()
dials into a tiny local TCP server WE feed from a WebSocket, instead of
dialing out to socat -- same `socket://` URL mechanism pyserial itself
provides, so connect() and its _reader_loop() run for real either way. See
start_teleoperationV2.py's own module docstring for the full story and for
why an earlier version's `leader_smooth` was silently wrong (0.15 instead
of the reference script's own default of 1).

This file's only remaining job is the safety wrapper around that proven
core: Zenoh E-stop/reset (shared with the base) and the robot/arm/state
heartbeat, via run_teleoperation()'s stop_event/on_tick/on_ready hooks --
none of which change its teleoperation loop itself.

Command contract
----------------
WebSocket ws://<robot-ip>:ARM_WS_PORT   JSON text messages: {"raw": "<line>"}
                  where <line> is a raw, UNPROCESSED line of GELLO firmware
                  output (`t<ms> J1:<deg> J2:<deg> ... J7:<deg>`, absolute
                  sensor degrees in [0, 360), relayed byte-for-byte from
                  the serial port -- see gello.js's readLoop() /
                  gello_reader.py). Sent DIRECTLY by the browser
                  (operator/web/static/js/armLink.js), bypassing Zenoh +
                  web_server.py's /ws/control relay -- same "direct
                  WebSocket, one fewer hop" pattern as camera_pub.py's
                  video, but its OWN connection/port, not shared with the
                  camera: this process needs the `lerobot` conda env,
                  whose own opencv-python has NO GStreamer support
                  (confirmed empirically 2026-07-09), so it can't share a
                  process with camera_pub.py (which needs system cv2 with
                  GStreamer) without breaking one of the two.
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
the same time isn't workable (see operator/input_agent.py). E-stop
(robot/cmd/stop) sets `stop_event`, checked once per tick inside
run_teleoperation(): while set, it stops calling send_action() and calls
robot.disable_torque() once (edge-triggered) -- verified directly (fake
follower, real run_teleoperation()) to actually halt sending and resume
correctly on reset.

NOTE: this version does NOT have a freshness watchdog on the GELLO data
itself (no ARM_CMD_TIMEOUT_SEC) -- that matches start_teleoperation.py's
own behavior exactly (it has none either), which is the confirmed-working
baseline this file defers to. It also does NOT set max_relative_target
(the earlier version's extra software safety cap) -- the reference script
doesn't either. Both are easy to reintroduce as additional stop_event/
on_tick logic or a RebotB601FollowerRobotConfig kwarg if wanted, but
weren't part of what was actually confirmed working, so they're left out
rather than silently reintroduced.
"""
import json
import os
import sys
import threading
import time
from pathlib import Path

import zenoh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from zenoh_config import load_robot_config
from start_teleoperationV2 import run_teleoperation  # noqa: E402

# Same by-id stable symlink as before (NOT a bare /dev/ttyACM0 -- USB-serial
# enumeration order isn't stable across reboots on this machine, see git
# history for the incident this fixed). Orthogonal to the GELLO/leader
# fix above -- this is about the FOLLOWER port, unrelated to what was
# actually broken, so it's kept.
ARM_PORT = os.environ.get(
    "ARM_PORT",
    "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00",
)
ARM_ID = "follower"
GELLO_TELEOP_ID = os.environ.get("GELLO_TELEOP_ID", "mon_gello")
ARM_WS_HOST = "0.0.0.0"
ARM_WS_PORT = 8767
ARM_LEADER_SMOOTH = float(os.environ.get("ARM_LEADER_SMOOTH", "1"))  # matches start_teleoperation.py's own default
ARM_FPS = int(os.environ.get("ARM_FPS", "60"))
HEARTBEAT_PERIOD = 0.2  # 5 Hz heartbeat / state

stop_event = threading.Event()
_robot_holder: dict = {"robot": None}
_last_beat = 0.0
_pub_state = None


def on_stop(_sample) -> None:
    stop_event.set()
    print("[STOP] Arm emergency stop latched (robot/cmd/reset to clear).", flush=True)


def on_reset(_sample) -> None:
    stop_event.clear()
    robot = _robot_holder["robot"]
    if robot is not None:
        try:
            robot.configure()  # re-enables torque and re-applies control modes
            print("[RESET] Arm motors re-enabled, estop cleared.", flush=True)
        except Exception as exc:
            print(f"[RESET] configure() failed: {exc}", flush=True)


def _on_ready(robot, _teleop) -> None:
    _robot_holder["robot"] = robot


def _on_tick(obs: dict, moving: bool) -> None:
    global _last_beat
    now = time.time()
    if now - _last_beat < HEARTBEAT_PERIOD:
        return
    _last_beat = now
    robot = _robot_holder["robot"]
    joints = {k.removesuffix(".pos"): v for k, v in obs.items()}
    _pub_state.put(json.dumps({
        "connected": robot.is_connected if robot is not None else False,
        "moving": moving,
        "fresh_cmd": moving,
        "estop": stop_event.is_set(),
        "joints": joints,
        "ts": now,
    }))


def main() -> None:
    global _pub_state
    with zenoh.open(load_robot_config("arm_agent")) as session:
        session.declare_subscriber("robot/cmd/stop", on_stop)
        session.declare_subscriber("robot/cmd/reset", on_reset)
        _pub_state = session.declare_publisher("robot/arm/state")

        print("arm_agent: delegating to start_teleoperationV2.run_teleoperation() "
              f"(GELLO_TELEOP_ID={GELLO_TELEOP_ID!r}, ARM_PORT={ARM_PORT!r}).", flush=True)
        run_teleoperation(
            robot_port=ARM_PORT,
            ws_host=ARM_WS_HOST,
            ws_port=ARM_WS_PORT,
            robot_id=ARM_ID,
            teleop_id=GELLO_TELEOP_ID,
            leader_smooth=ARM_LEADER_SMOOTH,
            fps=ARM_FPS,
            stop_event=stop_event,
            on_tick=_on_tick,
            on_ready=_on_ready,
        )


if __name__ == "__main__":
    main()
