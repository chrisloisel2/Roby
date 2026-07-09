#!/usr/bin/env python3
"""Generic UVC-webcam capture + multi-camera WebSocket server.

Used by robot/camera_pub.py to serve every UVC camera plugged into the
robot PC over a SINGLE WebSocket connection/port, so all feeds get
byte-for-byte identical network treatment -- same TCP connection, same
write-buffer/backpressure domain, same asyncio scheduling -- instead of each
camera racing independently over its own socket. Every binary message sent
to a client is [1 byte cam_id][JPEG bytes]; the browser demuxes by that
first byte (see operator/web/static/js/videoMux.js). Text messages carry a
JSON camera list (see CameraManager.snapshot()) instead of a JPEG -- the
browser tells binary from text frames apart the normal WebSocket way, no
extra framing needed.

Which camera plays which UI role (main tile vs. picture-in-picture) is a
browser-side, per-operator setting now (operator/web/static/js/config.js's
`cameras.primaryId`/`secondaryId`, picked from the discovered list in the
settings modal) -- this file's only job is finding every working camera and
streaming it, not deciding which one is "front".

Three things:
  CameraCapture   one camera's capture thread (open, negotiate, encode
                  loop). Holds only the newest encoded frame
                  (latest_jpeg/latest_id under a lock) -- a slow client or a
                  stalled encode never queues up stale frames, both sides
                  just skip straight to whatever is newest. Sets `alive =
                  False` (never raises out of the thread) if it can't open
                  its camera at all, or if reads stop succeeding for
                  `lost_after_sec` after having worked -- CameraManager
                  reaps it either way, freeing its /dev/videoN index for
                  rediscovery (e.g. after a replug).
  CameraManager   owns discovery: a background thread that periodically
                  probes every /dev/videoN index NOT already claimed by a
                  live CameraCapture, and promotes any that both opens and
                  delivers a real frame to a full CameraCapture + thread.
                  Single-threaded, sequential probing -- unlike the old
                  multi-CameraCapture-instances-each-auto-probing design,
                  there's no possibility of two probes racing for the same
                  index and swapping which physical camera ends up on which
                  cam_id, so no NAME_FILTER-style pinning is needed for
                  correctness anymore.
  MultiCameraServer  runs ONE websockets.serve() and, per connected client,
                  round-robins over every live CameraCapture's latest
                  frame, sending whichever changed, plus a JSON camera-list
                  text message whenever the active set changes.

Logging (all flush=True, since this runs backgrounded with stdout
redirected to a file -- a print that isn't flushed can simply never show up
in the log if the process is later killed): CameraManager logs every
camera it discovers and drops; CameraCapture's loop prints a FATAL line (not
just a bare traceback) if it can never open its camera, and otherwise a
heartbeat every ~2s with frames/fps/failure counts so "server is up but
browser shows nothing" is diagnosable from the log alone -- no heartbeat at
all -> camera never opened; heartbeat with 0 fps -> opened but not producing
frames; heartbeat healthy but no "client connected" line -> network/
reachability, not the camera.
"""
import asyncio
import json
import os
import socket
import threading
import time
from pathlib import Path

# Must be set before cv2 is imported (read once at native-library init).
# Discovery probes every unclaimed /dev/videoN index every few seconds
# (CameraManager), so indices that never have a camera on them (no second
# camera plugged in, a metadata-only node, ...) would otherwise print a
# libv4l "can't open camera by index" warning straight to stderr on every
# single pass forever -- confirmed directly (2026-07-10): our own
# `_probe_index()` already handles that failure cleanly and silently, this
# is purely OpenCV's own noise on top of it.
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2  # noqa: E402
import websockets  # noqa: E402


def _fourcc_str(fourcc_int: int) -> str:
    return "".join(chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4))


def _video_device_name(idx: int) -> str:
    """Best-effort /dev/videoN -> V4L2 device name, via the standard sysfs
    node. Returns "" if unreadable rather than raising, since this is
    advisory (shown to the operator so they can tell two cameras apart in
    the settings dropdown), not required. NOTE: this is NOT the same string
    `lsusb` reports (confirmed empirically 2026-07-09: lsusb showed "HSTD
    USB3.0 Camera" for a device whose V4L2 name is the more generic
    "USB3.0 Camera: USB3.0 Camera").
    """
    try:
        return Path(f"/sys/class/video4linux/video{idx}/name").read_text().strip()
    except OSError:
        return ""


def _probe_index(idx: int) -> bool:
    """True if /dev/video<idx> both opens AND delivers a real frame. USB
    webcams commonly expose a second metadata-only /dev/videoN node that
    opens fine but never reads -- this is what actually excludes those,
    not name filtering.
    """
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            return False
        ok, _ = cap.read()
        return ok
    finally:
        cap.release()


class CameraCapture:
    def __init__(
        self,
        *,
        cam_id: int,
        label: str,
        width: int,
        height: int,
        jpeg_quality: int,
        lost_after_sec: float = 5.0,
        heartbeat_sec: float = 2.0,
    ):
        """
        cam_id        1-byte tag prefixed on every WebSocket message sent
                      for this camera, and the id the browser shows/lets
                      the operator pick in settings -- always the camera's
                      own /dev/videoN index (see CameraManager), stable for
                      the lifetime of a boot.
        label         short tag prefixed on every log line (e.g. the V4L2
                      device name, or "cam3" if that was empty).
        lost_after_sec  no successful read for this long after previously
                      streaming fine -> treat as unplugged, set alive =
                      False so CameraManager reaps it (freeing this index
                      for rediscovery if replugged).
        """
        self.cam_id = cam_id
        self.label = label
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.lost_after_sec = lost_after_sec
        self.heartbeat_sec = heartbeat_sec

        self.latest_jpeg: bytes | None = None
        self.latest_id = 0
        self.lock = threading.Lock()
        self.running = True
        self.alive = True  # set False by _loop() on open failure or a lost camera; CameraManager reaps on this

    def _log(self, msg: str) -> None:
        print(f"[{self.label}] {msg}", flush=True)

    def _loop(self) -> None:
        cap = cv2.VideoCapture(self.cam_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            self._log("FATAL -- camera vanished between discovery and capture start, dropping.")
            cap.release()
            self.alive = False
            return

        # MJPG (compressed in the camera's own hardware) before the
        # resolution: raw (YUYV) can't sustain this resolution over USB
        # bandwidth at a usable framerate, and the FOURCC must be set first
        # for the driver to renegotiate correctly. Deliberately NOT setting
        # CAP_PROP_FPS: on the front HSTD camera, requesting an explicit FPS
        # it doesn't natively expose at this resolution breaks GStreamer
        # pipeline negotiation outright (isOpened() goes False) -- since
        # that failure mode costs nothing to avoid and we have no evidence
        # any camera here needs an explicit FPS request, every
        # CameraCapture instance skips it.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # Keep the driver's internal buffer at 1 frame so cap.read() always
        # returns the newest frame instead of draining a backlog queued
        # while we were busy encoding/sending the previous one.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        negotiated_fourcc = _fourcc_str(int(cap.get(cv2.CAP_PROP_FOURCC)))
        self._log(f"streaming /dev/video{self.cam_id} (cam_id={self.cam_id})")
        self._log("Camera configuration:")
        self._log(f"  FOURCC: {negotiated_fourcc!r}" + (" (WARNING: expected 'MJPG')" if negotiated_fourcc != "MJPG" else ""))
        self._log(f"  Width : {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
        self._log(f"  Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        self._log(f"  FPS   : {cap.get(cv2.CAP_PROP_FPS)}")
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.width
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.height

        frames_ok = frames_read_fail = frames_encode_fail = 0
        last_heartbeat = last_frame_ts = time.monotonic()

        try:
            while self.running:
                ok, frame = cap.read()
                now = time.monotonic()

                if not ok:
                    frames_read_fail += 1
                    time.sleep(0.001)
                else:
                    ok, jpeg = cv2.imencode(
                        ".jpg",
                        frame,
                        [
                            cv2.IMWRITE_JPEG_QUALITY,
                            self.jpeg_quality,
                            cv2.IMWRITE_JPEG_OPTIMIZE,
                            0,
                        ],
                    )
                    if not ok:
                        frames_encode_fail += 1
                    else:
                        with self.lock:
                            self.latest_jpeg = jpeg.tobytes()
                            self.latest_id += 1
                        frames_ok += 1
                        last_frame_ts = now

                if now - last_frame_ts >= self.lost_after_sec:
                    self._log(
                        f"no frame in {self.lost_after_sec:.0f}s -- treating as unplugged, "
                        f"dropping (will rediscover if replugged)."
                    )
                    self.alive = False
                    break

                elapsed = now - last_heartbeat
                if elapsed >= self.heartbeat_sec:
                    size_kb = len(self.latest_jpeg) / 1024 if self.latest_jpeg else 0
                    self._log(
                        f"heartbeat -- {frames_ok} frames in {elapsed:.1f}s "
                        f"(~{frames_ok / elapsed:.1f} fps), read_fail={frames_read_fail} "
                        f"encode_fail={frames_encode_fail}, latest jpeg={size_kb:.0f}KB"
                    )
                    if frames_ok == 0:
                        self._log("WARNING -- 0 frames encoded this interval, camera is open but not producing usable frames")
                    frames_ok = frames_read_fail = frames_encode_fail = 0
                    last_heartbeat = now
        finally:
            cap.release()

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        return thread


class CameraManager:
    """Discovers every working UVC camera on /dev/video* automatically and
    keeps a CameraCapture running for each one -- no env vars needed for
    the common case (see module docstring).
    """

    def __init__(
        self,
        *,
        width: int = 1920,
        height: int = 1080,
        jpeg_quality: int = 60,
        max_probe_index: int = 8,
        discover_every_sec: float = 3.0,
        exclude_indices: "set[int] | None" = None,
    ):
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.max_probe_index = max_probe_index
        self.discover_every_sec = discover_every_sec
        self.exclude_indices = exclude_indices or set()

        self.cameras: dict[int, CameraCapture] = {}
        self._lock = threading.Lock()
        self._running = True

    def _discover_once(self) -> None:
        # Reap anything that stopped delivering frames first, so a replug
        # can immediately reclaim its old index in the same pass.
        with self._lock:
            dead = [idx for idx, cap in self.cameras.items() if not cap.alive]
            for idx in dead:
                del self.cameras[idx]
            claimed = set(self.cameras.keys())
        for idx in dead:
            print(f"[multicam] camera /dev/video{idx} dropped.", flush=True)

        for idx in range(self.max_probe_index + 1):
            if idx in claimed or idx in self.exclude_indices:
                continue
            if not _probe_index(idx):
                continue
            name = _video_device_name(idx)
            print(f"[multicam] discovered camera at /dev/video{idx} (name={name!r})", flush=True)
            capture = CameraCapture(
                cam_id=idx,
                label=name or f"cam{idx}",
                width=self.width,
                height=self.height,
                jpeg_quality=self.jpeg_quality,
            )
            capture.name = name or f"Caméra {idx}"
            capture.start()
            with self._lock:
                self.cameras[idx] = capture

    def _discovery_loop(self) -> None:
        while self._running:
            try:
                self._discover_once()
            except Exception as exc:  # never let a probe glitch kill discovery for good
                print(f"[multicam] discovery pass failed: {exc}", flush=True)
            time.sleep(self.discover_every_sec)

    def active_captures(self) -> list[CameraCapture]:
        with self._lock:
            return [cap for cap in self.cameras.values() if cap.alive]

    def snapshot(self) -> list[dict]:
        """JSON-serializable description of every currently-live camera,
        sent to the browser so its settings page can list them by name and
        let the operator pick which plays which UI role.
        """
        return [
            {"id": cap.cam_id, "name": getattr(cap, "name", cap.label), "width": cap.width, "height": cap.height}
            for cap in sorted(self.active_captures(), key=lambda c: c.cam_id)
        ]

    def start(self) -> None:
        threading.Thread(target=self._discovery_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            for cap in self.cameras.values():
                cap.running = False


class MultiCameraServer:
    """Serves every camera CameraManager discovers over ONE WebSocket
    connection per client -- each binary message is [1 byte cam_id][JPEG
    bytes], so every feed shares identical connection-level behavior
    (TCP_NODELAY, write-buffer watermarks, asyncio scheduling) instead of
    independent sockets that could each stall/jitter differently. A JSON
    text message (`{"type": "camera_list", "cameras": [...]}`) is sent on
    connect and again whenever the active set changes, so the browser's
    camera-role settings stay in sync with what's actually plugged in
    without needing to reconnect.
    """

    def __init__(self, *, host: str = "0.0.0.0", port: int, manager: CameraManager):
        self.host = host
        self.port = port
        self.manager = manager

    async def _stream(self, websocket):
        peer = websocket.remote_address
        print(f"[multicam] client connected from {peer}", flush=True)

        sock = websocket.transport.get_extra_info("socket")
        if sock is not None:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        last_sent: dict[int, int] = {}
        warned_no_frame: set[int] = set()
        last_sent_list: list[dict] | None = None

        try:
            while True:
                cams = self.manager.active_captures()

                current_list = self.manager.snapshot()
                if current_list != last_sent_list:
                    await websocket.send(json.dumps({"type": "camera_list", "cameras": current_list}))
                    last_sent_list = current_list

                sent_any = False
                live_ids = set()
                for cam in cams:
                    live_ids.add(cam.cam_id)
                    with cam.lock:
                        jpeg = cam.latest_jpeg
                        frame_id = cam.latest_id

                    if jpeg is None:
                        if cam.cam_id not in warned_no_frame:
                            print(
                                f"[multicam] client {peer}: cam_id={cam.cam_id} ({cam.label}) "
                                f"has no frame yet -- check that camera's startup/heartbeat lines above",
                                flush=True,
                            )
                            warned_no_frame.add(cam.cam_id)
                        continue

                    if frame_id == last_sent.get(cam.cam_id):
                        continue

                    last_sent[cam.cam_id] = frame_id
                    await websocket.send(bytes([cam.cam_id]) + jpeg)
                    sent_any = True

                # Forget bookkeeping for cameras that disappeared, so a
                # replug (new CameraCapture, frame ids starting back at 0)
                # doesn't get mistaken for "already sent".
                for idx in list(last_sent):
                    if idx not in live_ids:
                        del last_sent[idx]
                        warned_no_frame.discard(idx)

                await asyncio.sleep(0 if sent_any else 0.001)
        except websockets.ConnectionClosed:
            print(f"[multicam] client {peer} disconnected", flush=True)

    async def _main(self) -> None:
        self.manager.start()

        async with websockets.serve(
            self._stream,
            self.host,
            self.port,
            max_size=None,
            max_queue=1,
            compression=None,
            ping_interval=None,
            write_limit=(1024 * 1024, 0),  # (high, low) write-buffer watermarks
        ):
            print(f"[multicam] Listening on ws://{self.host}:{self.port}, discovering cameras...", flush=True)
            await asyncio.Future()

    def run(self) -> None:
        """Blocking: starts camera discovery and serves forever. Call this
        directly from `if __name__ == "__main__":`.
        """
        try:
            asyncio.run(self._main())
        except KeyboardInterrupt:
            pass
        finally:
            self.manager.stop()
            print("[multicam] stopped.", flush=True)
