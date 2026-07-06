#!/usr/bin/env python3
"""Operator-side web server.

Bridges Zenoh <-> browser. The browser never speaks Zenoh directly: this
server relays the robot camera/state to the browser and forwards operator
commands (base, deadman, gripper, E-stop, reset) from the browser to Zenoh.

    Browser  <--WebSocket-->  web_server.py  <--Zenoh-->  robot PC

Endpoints
---------
GET  /             the operator UI (operator/web/index.html)
WS   /ws/camera    server -> browser : base64 JPEG frames
WS   /ws/status    server -> browser : robot heartbeat, reported state, fps
WS   /ws/control   browser -> server : {type: base|deadman|stop|reset|gripper}

Note: run ONE operator input source at a time (this web UI OR input_agent.py) —
both publish to the same command topics.
"""
import asyncio
import base64
import json
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
import zenoh
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

WEB_DIR = Path(__file__).resolve().parent / "web"
CONFIG = Path(__file__).resolve().parent.parent / "config" / "operator_zenoh.json5"

CAMERA_KEY = "robot/camera/front/jpeg"
HEARTBEAT_KEY = "robot/heartbeat"
STATE_KEY = "robot/state"
STALE_AFTER = 1.0  # seconds without data => considered lost

# --- Shared state (written by Zenoh callbacks, read by WS coroutines) --------
_frame = {"data": None, "ts": 0.0}
_frame_times = deque(maxlen=30)
_heartbeat_ts = 0.0
_robot_state = {}

# --- Zenoh handles, populated on startup -------------------------------------
Z = {"session": None, "base": None, "stop": None, "reset": None, "deadman": None, "gripper": None}


def on_camera(sample):
    now = time.time()
    _frame["data"] = sample.payload.to_bytes()
    _frame["ts"] = now
    _frame_times.append(now)


def on_heartbeat(_sample):
    global _heartbeat_ts
    _heartbeat_ts = time.time()


def on_state(sample):
    global _robot_state
    try:
        _robot_state = json.loads(sample.payload.to_bytes().decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        pass


def _fps() -> float:
    if len(_frame_times) < 2:
        return 0.0
    span = _frame_times[-1] - _frame_times[0]
    return round((len(_frame_times) - 1) / span, 1) if span > 0 else 0.0


@asynccontextmanager
async def lifespan(_app: FastAPI):
    session = zenoh.open(zenoh.Config.from_file(str(CONFIG)))
    session.declare_subscriber(CAMERA_KEY, on_camera)
    session.declare_subscriber(HEARTBEAT_KEY, on_heartbeat)
    session.declare_subscriber(STATE_KEY, on_state)
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


@app.get("/")
async def index():
    return FileResponse(WEB_DIR / "index.html")


@app.websocket("/ws/camera")
async def camera_ws(ws: WebSocket):
    await ws.accept()
    last_ts = 0.0
    try:
        while True:
            if _frame["data"] is not None and _frame["ts"] != last_ts:
                last_ts = _frame["ts"]
                await ws.send_text(base64.b64encode(_frame["data"]).decode("ascii"))
            await asyncio.sleep(1 / 30)
    except WebSocketDisconnect:
        pass


@app.websocket("/ws/status")
async def status_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            now = time.time()
            await ws.send_text(json.dumps({
                "robot": (now - _heartbeat_ts) < STALE_AFTER,
                "camera": (now - _frame["ts"]) < STALE_AFTER,
                "fps": _fps(),
                "state": _robot_state,
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
