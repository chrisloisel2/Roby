#!/usr/bin/env python3
"""Operator-side web server.

Bridges Zenoh <-> browser for state/control. The browser never speaks Zenoh
directly: this server relays robot/arm state to the browser and forwards
operator commands (base, deadman, gripper, E-stop, reset) from the browser
to Zenoh.

    Browser  <--WebSocket-->  web_server.py  <--Zenoh-->  robot PC

Video is NOT part of this bridge: the browser connects straight to
robot/camera_pub.py's own WebSocket server (ws://<robot-ip>:8765, see
static/js/camera.js) for lower latency than an extra JPEG-over-Zenoh hop
would add. Losing this server does not lose the camera feed.

Arm/GELLO data is ALSO not part of this bridge anymore: the browser relays
raw GELLO serial lines straight to robot/arm_agent.py's own WebSocket
(ws://<robot-ip>:8767, see static/js/gello.js + armLink.js) -- same
reasoning as video. Calibration now happens server-side in arm_agent.py
(a real lerobot teleoperator class), so this server no longer even serves
gello_calibration.json -- the browser has nothing left to fetch. robot/cmd/
stop and robot/cmd/reset (E-stop / re-arm) still go through this server's
/ws/control, since they're shared with the base's own E-stop.

The mast (vertical carriage, robot/mast_serial_bridge.py <-> Arduino) IS
relayed through this bridge, unlike the arm/camera: it's low-bandwidth
(one JSON command per browser tick, ~60Hz telemetry) so the extra Zenoh
hop costs nothing here, and it keeps the same "browser never speaks Zenoh
directly" story as base/gripper/stop/reset. robot/mast/event (raw firmware
ACK/MSG/WARN/ERR lines) is intentionally NOT relayed to the browser -- it's
a CLI/debug stream (`z_sub -k robot/mast/event`), not part of the UI.

Endpoints
---------
GET  /                          the operator UI (operator/web/index.html)
GET  /static/*                  the UI's assets (operator/web/static: css + JS modules)
WS   /ws/status                server -> browser : robot heartbeat, reported state, arm state, mast state
WS   /ws/control               browser -> server : {type: base|deadman|stop|reset|gripper|mast}

Note: run ONE operator input source at a time (this web UI OR input_agent.py) —
both publish to the same command topics. The browser can read the joystick
(Gamepad API) and the GELLO (Web Serial API) directly, making it a complete
alternative to input_agent.py -- see the "Piloter depuis ce navigateur"
toggle in index.html.
"""
import asyncio
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
import zenoh
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

WEB_DIR = Path(__file__).resolve().parent / "web"
CONFIG = Path(__file__).resolve().parent.parent / "config" / "operator_zenoh.json5"

HEARTBEAT_KEY = "robot/heartbeat"
STATE_KEY = "robot/state"
ARM_STATE_KEY = "robot/arm/state"
MAST_STATE_KEY = "robot/mast/state"
MAST_LINK_KEY = "robot/mast/link"
STALE_AFTER = 1.0  # seconds without data => considered lost

# --- Shared state (written by Zenoh callbacks, read by WS coroutines) --------
_heartbeat_ts = 0.0
_robot_state = {}
_arm_state = {}
_mast_state = {}
_mast_linked = False  # robot/mast/link is RELIABLE + change-only (see mast_serial_bridge.py):
                       # kept separate from _mast_state so a lost link is visible even once
                       # the bridge/firmware stops sending state entirely (not just gone stale).

# --- Zenoh handles, populated on startup -------------------------------------
Z = {"session": None, "base": None, "stop": None, "reset": None, "deadman": None,
     "gripper": None, "mast": None}


def on_heartbeat(_sample):
    global _heartbeat_ts
    _heartbeat_ts = time.time()


def on_state(sample):
    global _robot_state
    try:
        _robot_state = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        pass


def on_arm_state(sample):
    global _arm_state
    try:
        _arm_state = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        pass


def on_mast_state(sample):
    global _mast_state
    try:
        _mast_state = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        pass


def on_mast_link(sample):
    global _mast_linked
    _mast_linked = sample.payload.to_bytes().decode("utf-8").strip() == "Connected"


@asynccontextmanager
async def lifespan(_app: FastAPI):
    session = zenoh.open(zenoh.Config.from_file(str(CONFIG)))
    session.declare_subscriber(HEARTBEAT_KEY, on_heartbeat)
    session.declare_subscriber(STATE_KEY, on_state)
    session.declare_subscriber(ARM_STATE_KEY, on_arm_state)
    session.declare_subscriber(MAST_STATE_KEY, on_mast_state)
    session.declare_subscriber(MAST_LINK_KEY, on_mast_link)
    Z["session"] = session
    Z["base"] = session.declare_publisher("robot/cmd/base")
    Z["stop"] = session.declare_publisher("robot/cmd/stop")
    Z["reset"] = session.declare_publisher("robot/cmd/reset")
    Z["deadman"] = session.declare_publisher("operator/deadman")
    Z["gripper"] = session.declare_publisher("robot/cmd/gripper")
    Z["mast"] = session.declare_publisher("robot/mast/cmd")
    print("web_server ready on http://0.0.0.0:8080")
    try:
        yield
    finally:
        # Safety: release deadman, zero the base, and stop the mast as the
        # server goes down.
        try:
            Z["deadman"].put("false")
            Z["base"].put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
            Z["mast"].put(json.dumps({"action": "stop"}))
        finally:
            session.close()


app = FastAPI(lifespan=lifespan)

# CSS + JS modules of the UI. StaticFiles handles ETag/Last-Modified itself,
# so a hard-refresh after editing a file picks up the change without any
# cache-busting machinery.
app.mount("/static", StaticFiles(directory=WEB_DIR / "static"), name="static")


@app.get("/")
async def index():
    return FileResponse(WEB_DIR / "index.html")


@app.websocket("/ws/status")
async def status_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            now = time.time()
            # _arm_state has no separate heartbeat topic: go stale/empty past
            # STALE_AFTER using its own "ts" field instead, so a dead
            # arm_agent.py doesn't leave the UI showing a frozen "connected"
            # forever.
            arm_fresh = bool(_arm_state) and (now - _arm_state.get("ts", 0)) < STALE_AFTER
            # Same staleness treatment for the mast's position/limit telemetry
            # ("t", set by mast_serial_bridge.py on receipt) -- but "linked" is
            # sent unconditionally: it's the one thing we still know for sure
            # even once the bridge has stopped publishing state entirely (the
            # bridge publishes robot/mast/link="Disconnected" itself in that
            # case, not just silence).
            mast_fresh = bool(_mast_state) and (now - _mast_state.get("t", 0)) < STALE_AFTER
            await ws.send_text(json.dumps({
                "robot": (now - _heartbeat_ts) < STALE_AFTER,
                "state": _robot_state,
                "arm": _arm_state if arm_fresh else {},
                "mast": {**(_mast_state if mast_fresh else {}), "linked": _mast_linked},
            }))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass


def _handle_control(msg: dict) -> None:
    kind = msg.get("type")
    if kind == "base":
        Z["base"].put(json.dumps({
            "vx": float(msg.get("vx", 0.0)),
            "vy": float(msg.get("vy", 0.0)),
            "wz": float(msg.get("wz", 0.0)),
            "ts": time.time(),
        }))
    elif kind == "deadman":
        Z["deadman"].put("true" if msg.get("value") else "false")
    elif kind == "stop":
        Z["stop"].put("1")
    elif kind == "reset":
        Z["reset"].put("1")
    elif kind == "gripper":
        Z["gripper"].put(json.dumps({"gripper": float(msg.get("value", 0.0)), "ts": time.time()}))
    elif kind == "mast":
        # Passed through mostly as-is onto robot/mast/cmd: control.js already
        # sends the exact {"action": ...} shape mast_serial_bridge.py expects
        # (see that module's docstring for the full action contract), just
        # stripped of our own "type" envelope key and timestamped.
        payload = {k: v for k, v in msg.items() if k != "type"}
        payload.setdefault("ts", time.time())
        Z["mast"].put(json.dumps(payload))
    # No "arm" case: joint-position commands go straight to arm_agent.py's
    # own WebSocket now (armLink.js), not through this relay -- see this
    # file's module docstring.


@app.websocket("/ws/control")
async def control_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            _handle_control(json.loads(await ws.receive_text()))
    except WebSocketDisconnect:
        pass
    finally:
        # Browser gone => stop the robot (section 11: loss of client = stop).
        Z["deadman"].put("false")
        Z["base"].put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
        Z["mast"].put(json.dumps({"action": "velocity", "mm_s": 0.0}))


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
