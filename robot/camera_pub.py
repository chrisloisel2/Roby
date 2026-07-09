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

Logging (all flush=True, since this runs backgrounded with stdout
redirected to a file by scripts/start_robot.sh -- a print that isn't
flushed can simply never show up in the log if the process is later
killed): open_camera() logs every index it probes and why each one was
rejected; camera_loop() prints a FATAL line (not just a bare traceback) if
it can never open a camera, and otherwise a heartbeat every ~2s with
frames/fps/failure counts so "server is up but browser shows nothing" is
diagnosable from logs/camera_pub.log alone -- was it the camera (no
heartbeat, or heartbeat shows 0 fps) or the network (no "Client connected"
line at all)?
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
HEARTBEAT_SEC = 2.0  # how often camera_loop() logs a frames/fps summary

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
        if not cap.isOpened():
            print(f"camera_pub: probe /dev/video{idx}: isOpened() = False, skipping", flush=True)
            cap.release()
            continue
        ok, _ = cap.read()
        if ok:
            print(f"camera_pub: probe /dev/video{idx}: opened and read() ok -- using this one", flush=True)
            return cap, idx
        print(f"camera_pub: probe /dev/video{idx}: opened but read() failed (likely a metadata-only node), skipping", flush=True)
        cap.release()
    tried = f"index {camera_id}" if camera_id is not None else f"indices 0..{MAX_PROBE_INDEX}"
    raise RuntimeError(f"No working camera found (tried {tried})")


def _fourcc_str(fourcc_int: int) -> str:
    return "".join(chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4))


def camera_loop(camera_id: int | None) -> None:
    global latest_jpeg, latest_id, running

    try:
        cap, idx = open_camera(camera_id)
    except Exception as e:
        print(
            f"camera_pub: FATAL -- {e}. The WebSocket server will keep "
            f"listening (so start_robot.sh's liveness check still passes) "
            f"but will never have a frame to send -- this is the "
            f"'server up, 0 frames in the browser' symptom.",
            flush=True,
        )
        raise

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

    negotiated_fourcc = _fourcc_str(int(cap.get(cv2.CAP_PROP_FOURCC)))
    print(f"camera_pub: streaming camera index {idx} on ws://{HOST}:{PORT}", flush=True)
    print("Camera configuration:", flush=True)
    print(f"  FOURCC: {negotiated_fourcc!r}" + (" (WARNING: expected 'MJPG')" if negotiated_fourcc != "MJPG" else ""), flush=True)
    print("  Width :", cap.get(cv2.CAP_PROP_FRAME_WIDTH), flush=True)
    print("  Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT), flush=True)
    print("  FPS   :", cap.get(cv2.CAP_PROP_FPS), flush=True)

    frames_ok = frames_read_fail = frames_encode_fail = 0
    last_heartbeat = time.monotonic()

    try:
        while running:
            ok, frame = cap.read()

            if not ok:
                frames_read_fail += 1
                time.sleep(0.001)
            else:
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
                    frames_encode_fail += 1
                else:
                    with lock:
                        latest_jpeg = jpeg.tobytes()
                        latest_id += 1
                    frames_ok += 1

            now = time.monotonic()
            elapsed = now - last_heartbeat
            if elapsed >= HEARTBEAT_SEC:
                size_kb = len(latest_jpeg) / 1024 if latest_jpeg else 0
                print(
                    f"camera_pub: heartbeat -- {frames_ok} frames in {elapsed:.1f}s "
                    f"(~{frames_ok / elapsed:.1f} fps), read_fail={frames_read_fail} "
                    f"encode_fail={frames_encode_fail}, latest jpeg={size_kb:.0f}KB",
                    flush=True,
                )
                if frames_ok == 0:
                    print(
                        "camera_pub: WARNING -- 0 frames encoded this interval, "
                        "camera is open but not producing usable frames",
                        flush=True,
                    )
                frames_ok = frames_read_fail = frames_encode_fail = 0
                last_heartbeat = now
    finally:
        cap.release()


async def stream(websocket):
    peer = websocket.remote_address
    print(f"camera_pub: client connected from {peer}", flush=True)

    sock = websocket.transport.get_extra_info("socket")
    if sock is not None:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    last_sent_id = -1
    warned_no_frame = False

    try:
        while True:
            with lock:
                jpeg = latest_jpeg
                frame_id = latest_id

            if jpeg is None:
                if not warned_no_frame:
                    print(
                        f"camera_pub: client {peer} connected but no frame captured "
                        f"yet -- check the camera_loop startup/heartbeat lines above",
                        flush=True,
                    )
                    warned_no_frame = True
                await asyncio.sleep(0.05)
                continue

            if frame_id == last_sent_id:
                await asyncio.sleep(0.001)
                continue

            last_sent_id = frame_id
            await websocket.send(jpeg)
            await asyncio.sleep(0)  # give control back to asyncio
    except websockets.ConnectionClosed:
        print(f"camera_pub: client {peer} disconnected", flush=True)


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
        print(f"Listening on ws://{HOST}:{PORT}", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("\ncamera_pub stopped.", flush=True)
