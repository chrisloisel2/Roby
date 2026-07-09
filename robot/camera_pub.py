#!/usr/bin/env python3
"""Robot-side camera server.

Captures 1920x1200 and serves it straight to the browser over a raw
WebSocket (ws://<robot-ip>:8765), bypassing Zenoh entirely -- the browser
connects to this process directly (see operator/web/static/js/camera.js).
Zenoh still carries base/arm control and state; only the video path is
direct, because JPEG-over-Zenoh-over-another-WebSocket-hop added latency
for no benefit at this resolution.

Capture (camera_loop, background thread) and delivery (stream, per client)
are decoupled: the thread always holds only the newest encoded frame
(latest_jpeg/latest_id under a lock), and each client coroutine sends it the
instant it changes. A slow client or a stalled encode never queues up stale
frames -- both sides just skip straight to whatever is newest.

Known camera-specific pitfalls (HSTD USB3.0 UVC camera on this robot),
already worked around below -- do not "fix" these back:
  - Never call cap.set(cv2.CAP_PROP_FPS, ...): requesting an explicit FPS
    this camera doesn't natively expose at this resolution breaks the
    GStreamer pipeline negotiation outright (isOpened() goes False).
  - /dev/videoN is not stable across boots/USB re-enumeration, and USB
    webcams commonly expose a second metadata-only node that opens but
    never reads -- open_camera() probes indices and keeps the first one
    that both opens AND delivers a real frame.
"""
import asyncio
import os
import socket
import threading
import time

import cv2
import websockets

HOST = "0.0.0.0"
PORT = 8765

WIDTH = 1920
HEIGHT = 1200
JPEG_QUALITY = 60
MAX_PROBE_INDEX = 8  # highest /dev/videoN index to try when auto-detecting

latest_jpeg: bytes | None = None
latest_id = 0
lock = threading.Lock()
running = True


def open_camera(camera_id: int | None) -> tuple[cv2.VideoCapture, int]:
    """Open a working camera by index.

    If ``camera_id`` is given (CAMERA_ID env var), use it directly. Otherwise
    probe indices 0..MAX_PROBE_INDEX and return the first one that both opens
    AND delivers a real frame: USB webcams commonly expose a second
    metadata-only /dev/videoN node that opens fine but never reads, and the
    index a given camera lands on shifts whenever the USB topology
    re-enumerates (e.g. another device unplugged/replugged) — a hardcoded
    index silently starts pointing at the wrong (or a dead) node.
    """
    candidates = [camera_id] if camera_id is not None else range(MAX_PROBE_INDEX + 1)
    for idx in candidates:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                return cap, idx
        cap.release()
    tried = f"index {camera_id}" if camera_id is not None else f"indices 0..{MAX_PROBE_INDEX}"
    raise RuntimeError(f"No working camera found (tried {tried})")


def camera_loop(camera_id: int | None) -> None:
    global latest_jpeg, latest_id, running

    cap, idx = open_camera(camera_id)

    # MJPG (compressed in the camera's own hardware) before the resolution:
    # this camera's raw (YUYV) format can't sustain 1920x1200 over USB
    # bandwidth at a usable framerate, and the FOURCC must be set first for
    # the driver to renegotiate correctly.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    # Keep the driver's internal buffer at 1 frame so cap.read() always
    # returns the newest frame instead of draining a backlog queued while we
    # were busy encoding/sending the previous one.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"camera_pub: streaming camera index {idx} on ws://{HOST}:{PORT}")
    print("Camera configuration:")
    print("  FOURCC:", int(cap.get(cv2.CAP_PROP_FOURCC)))
    print("  Width :", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("  Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("  FPS   :", cap.get(cv2.CAP_PROP_FPS))

    try:
        while running:
            ok, frame = cap.read()

            if not ok:
                time.sleep(0.001)
                continue

            ok, jpeg = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    JPEG_QUALITY,
                    cv2.IMWRITE_JPEG_OPTIMIZE,
                    0,
                ],
            )

            if not ok:
                continue

            with lock:
                latest_jpeg = jpeg.tobytes()
                latest_id += 1
    finally:
        cap.release()


async def stream(websocket):
    print("Client connected")

    sock = websocket.transport.get_extra_info("socket")
    if sock is not None:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    last_sent_id = -1

    try:
        while True:
            with lock:
                jpeg = latest_jpeg
                frame_id = latest_id

            if jpeg is None or frame_id == last_sent_id:
                await asyncio.sleep(0.001)
                continue

            last_sent_id = frame_id
            await websocket.send(jpeg)
            await asyncio.sleep(0)  # give control back to asyncio
    except websockets.ConnectionClosed:
        print("Client disconnected")


async def main() -> None:
    env_id = os.environ.get("CAMERA_ID")
    thread = threading.Thread(
        target=camera_loop, args=(int(env_id) if env_id is not None else None,), daemon=True
    )
    thread.start()

    async with websockets.serve(
        stream,
        HOST,
        PORT,
        max_size=None,
        max_queue=1,
        compression=None,
        ping_interval=None,
        write_limit=(1024 * 1024, 0),  # (high, low) write-buffer watermarks; see stream()
    ):
        print(f"Listening on ws://{HOST}:{PORT}")
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("\ncamera_pub stopped.")
