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

Arm joint-position commands are ALSO not part of this bridge anymore: the
browser sends those straight to robot/arm_agent.py's own WebSocket
(ws://<robot-ip>:8767, see static/js/armLink.js) -- same reasoning as
video. robot/cmd/stop and robot/cmd/reset (E-stop / re-arm) still go
through this server's /ws/control, since they're shared with the base's
own E-stop.

Endpoints
---------
GET  /                          the operator UI (operator/web/index.html)
GET  /static/*                  the UI's assets (operator/web/static: css + JS modules)
GET  /gello_calibration.json   GELLO calibration, fetched once by the browser's
                                own Web Serial reader (see static/js/gello.js) so
                                the measured values aren't duplicated in the page.
WS   /ws/status                server -> browser : robot heartbeat, reported state, arm state
WS   /ws/control               browser -> server : {type: base|deadman|stop|reset|gripper}

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
GELLO_CALIBRATION_PATH = Path(__file__).resolve().parent / "gello_calibration.json"

HEARTBEAT_KEY = "robot/heartbeat"
STATE_KEY = "robot/state"
ARM_STATE_KEY = "robot/arm/state"
STALE_AFTER = 1.0  # seconds without data => considered lost

# --- Shared state (written by Zenoh callbacks, read by WS coroutines) --------
_heartbeat_ts = 0.0
_robot_state = {}
_arm_state = {}

# --- Zenoh handles, populated on startup -------------------------------------
Z = {"session": None, "base": None, "stop": None, "reset": None, "deadman": None, "gripper": None}


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


@asynccontextmanager
async def lifespan(_app: FastAPI):
    session = zenoh.open(zenoh.Config.from_file(str(CONFIG)))
    session.declare_subscriber(HEARTBEAT_KEY, on_heartbeat)
    session.declare_subscriber(STATE_KEY, on_state)
    session.declare_subscriber(ARM_STATE_KEY, on_arm_state)
    Z["session"] = session
    Z["base"] = session.declare_publisher("robot/cmd/base")
    Z["stop"] = session.declare_publisher("robot/cmd/stop")
    Z["reset"] = session.declare_publisher("robot/cmd/reset")
    Z["deadman"] = session.declare_publisher("operator/deadman")
    Z["gripper"] = session.declare_publisher("robot/cmd/gripper")
    print("web_server ready on http://0.0.0.0:8080")
    try:
        yield
    finally:
        # Safety: release deadman and zero the base as the server goes down.
        try:
            Z["deadman"].put("false")
            Z["base"].put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
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


@app.get("/gello_calibration.json")
async def gello_calibration():
    return FileResponse(GELLO_CALIBRATION_PATH)


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
            await ws.send_text(json.dumps({
                "robot": (now - _heartbeat_ts) < STALE_AFTER,
                "state": _robot_state,
                "arm": _arm_state if arm_fresh else {},
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


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
