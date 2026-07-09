#!/usr/bin/env python
"""
start_teleoperationV2.py -- Teleoperation GELLO -> reBot B601, leader fed
from a WebSocket instead of a local/socat-relayed serial port.

This is start_teleoperation.py (same file, unmodified core loop) confirmed
working 2026-07-10 via:

    socat TCP-LISTEN:9999,reuseaddr,fork OPEN:/dev/cu.usbserial-2130,raw,ispeed=115200,ospeed=115200,echo=0
    python start_teleoperation.py --teleop-port socket://<leader PC ip>:9999

The ONLY structural change from that confirmed-working setup: instead of
teleop.connect() dialing OUT to a real/socat-relayed serial port, it dials
IN to a tiny local TCP server (SerialBridge below) that WE feed from an
incoming WebSocket connection. pyserial's socket:// URL handler is the
EXACT SAME mechanism a socat TCP-LISTEN relay uses (verified directly:
SerialBridge + serial.serial_for_url("socket://...") delivers lines
byte-exact) -- so GelloAs5600RawLeader.connect() still runs for real: same
_reader_loop() thread, same is_connected semantics, same everything as the
confirmed-working baseline. run_teleoperation()'s actual teleoperation
loop below is functionally UNCHANGED from start_teleoperation.py (only
optional stop_event/on_tick/on_ready hooks added, all no-ops when unused --
see their docstrings).

Why this exists: an earlier attempt bypassed connect() entirely (manually
writing into teleop._raw_angles, never calling connect()) and did not work
correctly in practice, despite the calibration math itself checking out in
isolated testing. Going through the real connect()/_reader_loop() path
removes that whole class of doubt -- whatever socat + start_teleoperation.py
already do correctly, this does identically, just fed from a WebSocket
instead of a socat pipe. Also matches DEFAULT_LEADER_SMOOTH=1 (see below)
exactly as the reference script does -- an earlier version silently used
the config class's own default (0.15, much smoother/laggier) instead by
never passing leader_smooth at all.

This file lives in the Roby repo (robot/), NOT next to the original
start_teleoperation.py in ~/03_JelloSoft/rebot_lerobot/scripts/ --
deliberately a SINGLE canonical, git-tracked copy rather than a duplicate
that could quietly drift out of sync with what arm_agent.py actually runs
(exactly the class of bug a stale duplicate operator/gello_calibration.json
caused earlier in this project's history). Because of that, the sys.path
setup below points at the external lerobot checkout by an absolute path
instead of one derived from this file's own location (as the original
script does, since IT lives inside that checkout).

Usage:
    python start_teleoperationV2.py
    python start_teleoperationV2.py --ws-port 8767 --robot-port /dev/ttyACM0
"""
import argparse
import asyncio
import json
import logging
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Callable

# Absolute, not derived from this file's own location -- see module
# docstring. Harmless (just a no-op) if lerobot is already importable
# without it, e.g. pip-installed -e into the active conda env.
_EXTERNAL_LEROBOT_SRC = Path.home() / "03_JelloSoft" / "rebot_lerobot" / "lerobot" / "src"
if _EXTERNAL_LEROBOT_SRC.is_dir() and str(_EXTERNAL_LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(_EXTERNAL_LEROBOT_SRC))

import websockets

from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.robots.rebot_b601_follower import RebotB601FollowerRobotConfig
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.gello_as5600_raw_leader import GelloAs5600RawLeaderTeleopConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up

logger = logging.getLogger(__name__)

DEFAULT_ROBOT_PORT = "/dev/ttyACM0"
DEFAULT_WS_HOST = "0.0.0.0"
DEFAULT_WS_PORT = 8767
DEFAULT_FPS = 60
DEFAULT_LEADER_SMOOTH = 1  # same as start_teleoperation.py -- NOT the config class's own 0.15 default


class SerialBridge:
    """Local TCP server that pyserial's socket:// URL handler connects to
    -- the exact mechanism a socat TCP-LISTEN relay uses (confirmed working
    with real GELLO hardware 2026-07-10), just fed by feed_line() (called
    from the WebSocket handler below) instead of a real serial device on
    the other end. Lets GelloAs5600RawLeader.connect() run for real.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(1)
        self.host, self.port = self._sock.getsockname()
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self) -> None:
        while True:
            try:
                conn, addr = self._sock.accept()
            except OSError:
                return
            print(f"[SerialBridge] teleop connected from {addr}", flush=True)
            with self._lock:
                self._clients.append(conn)

    def feed_line(self, line: str) -> None:
        data = (line.rstrip("\r\n") + "\n").encode("ascii", errors="ignore")
        with self._lock:
            dead = []
            for c in self._clients:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for d in dead:
                self._clients.remove(d)

    def close(self) -> None:
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
        try:
            self._sock.close()
        except OSError:
            pass


async def _ws_server(bridge: SerialBridge, host: str, port: int) -> None:
    async def handler(websocket):
        peer = websocket.remote_address
        print(f"[start_teleoperationV2] WebSocket client connected from {peer}", flush=True)
        try:
            async for message in websocket:
                if not isinstance(message, str):
                    continue
                try:
                    msg = json.loads(message)
                except ValueError:
                    continue
                line = msg.get("raw")
                if isinstance(line, str):
                    bridge.feed_line(line)
        except websockets.ConnectionClosed:
            print(f"[start_teleoperationV2] WebSocket client {peer} disconnected", flush=True)

    async with websockets.serve(handler, host, port, ping_interval=20):
        print(f"[start_teleoperationV2] Listening for raw GELLO data on ws://{host}:{port}", flush=True)
        await asyncio.Future()


def run_teleoperation(
    robot_port: str = DEFAULT_ROBOT_PORT,
    ws_host: str = DEFAULT_WS_HOST,
    ws_port: int = DEFAULT_WS_PORT,
    robot_id: str = "follower",
    teleop_id: str = "mon_gello",
    leader_smooth: float = DEFAULT_LEADER_SMOOTH,
    fps: int = DEFAULT_FPS,
    duration_s: float | None = None,
    stop_event=None,
    on_tick: "Callable[[dict, bool], None] | None" = None,
    on_ready: "Callable[[object, object], None] | None" = None,
) -> None:
    """Same core loop as start_teleoperation.py's run_teleoperation(), fed
    by a WebSocket instead of a real/socat-relayed serial port. Three
    optional, purely additive hooks (all no-ops by default, so calling
    this with none of them behaves identically to the reference script):

      stop_event   threading.Event -- while set, skips send_action() and
                   calls robot.disable_torque() once (edge-triggered) --
                   an external E-stop gate. Never checked if None.
      on_tick      (obs, moving) -> None, called every tick after the
                   send/skip decision -- e.g. to publish a heartbeat.
      on_ready     (robot, teleop) -> None, called once right after
                   connect() succeeds -- e.g. to stash references for an
                   external reset handler to call robot.configure() on.
    """
    bridge = SerialBridge(port=0)
    print(f"[start_teleoperationV2] Internal serial bridge listening on 127.0.0.1:{bridge.port}", flush=True)

    threading.Thread(
        target=lambda: asyncio.run(_ws_server(bridge, ws_host, ws_port)), daemon=True
    ).start()

    robot_config = RebotB601FollowerRobotConfig(port=robot_port, id=robot_id)
    teleop_config = GelloAs5600RawLeaderTeleopConfig(
        port=f"socket://127.0.0.1:{bridge.port}", id=teleop_id, leader_smooth=leader_smooth
    )

    robot = make_robot_from_config(robot_config)
    teleop = make_teleoperator_from_config(teleop_config)
    teleop_action_processor, robot_action_processor, _robot_observation_processor = (
        make_default_processors()
    )

    print(f"Connexion au follower ({robot_config.type}) sur {robot_port}...")
    robot.connect()
    print(f"Connexion au leader ({teleop_config.type}) via le pont WebSocket (attend la 1ere ligne)...")
    try:
        teleop.connect()
    except Exception as exc:
        print(f"Echec de connexion au leader : {exc}")
        raise

    if on_ready is not None:
        on_ready(robot, teleop)

    print(
        f"\nTeleoperation en cours (fps={fps}, leader_smooth={leader_smooth}). "
        "Ctrl+C pour arreter.\n"
    )

    was_stopped = False
    try:
        start = time.perf_counter()
        while True:
            loop_start = time.perf_counter()

            obs = robot.get_observation()
            estopped = stop_event.is_set() if stop_event is not None else False
            if estopped:
                if not was_stopped:
                    try:
                        robot.disable_torque()
                    except Exception as exc:
                        print(f"[start_teleoperationV2] disable_torque failed: {exc}")
                    was_stopped = True
            else:
                was_stopped = False
                raw_action = teleop.get_action()
                teleop_action = teleop_action_processor((raw_action, obs))
                robot_action = robot_action_processor((teleop_action, obs))
                robot.send_action(robot_action)

            if on_tick is not None:
                try:
                    on_tick(obs, not estopped)
                except Exception as exc:
                    print(f"[start_teleoperationV2] on_tick failed: {exc}")

            dt_s = time.perf_counter() - loop_start
            precise_sleep(max(1 / fps - dt_s, 0.0))
            loop_s = time.perf_counter() - loop_start
            print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)", end="\r")

            if duration_s is not None and time.perf_counter() - start >= duration_s:
                break
    except KeyboardInterrupt:
        move_cursor_up(1)
        print("\nArret demande (Ctrl+C).")
    finally:
        teleop.disconnect()
        robot.disconnect()
        bridge.close()
        print("Leader et follower deconnectes.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-port", default=DEFAULT_ROBOT_PORT, help="port du follower (ex: /dev/ttyACM0)")
    parser.add_argument("--ws-host", default=DEFAULT_WS_HOST, help="interface d'ecoute du WebSocket (donnees GELLO brutes)")
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT, help="port du WebSocket (donnees GELLO brutes)")
    parser.add_argument("--robot-id", default="follower")
    parser.add_argument("--teleop-id", default="mon_gello")
    parser.add_argument("--leader-smooth", type=float, default=DEFAULT_LEADER_SMOOTH)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument(
        "--duration-s", type=float, default=None, help="duree max en secondes (defaut: illimite, Ctrl+C pour arreter)"
    )
    args = parser.parse_args()

    init_logging()
    run_teleoperation(
        robot_port=args.robot_port,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        robot_id=args.robot_id,
        teleop_id=args.teleop_id,
        leader_smooth=args.leader_smooth,
        fps=args.fps,
        duration_s=args.duration_s,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
