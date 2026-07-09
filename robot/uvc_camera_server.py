#!/usr/bin/env python3
"""Generic UVC-webcam capture + multi-camera WebSocket server.

Used by robot/camera_pub.py to serve N cameras (currently: front + a second
generic UVC camera) over a SINGLE WebSocket connection/port, so both feeds
get byte-for-byte identical network treatment -- same TCP connection, same
write-buffer/backpressure domain, same asyncio scheduling -- instead of each
camera racing independently over its own socket. Every binary message sent
to a client is [1 byte camera_id][JPEG bytes]; the browser demuxes by that
first byte (see operator/web/static/js/videoMux.js).

Two classes:
  CameraCapture      one camera's capture thread (open, negotiate, encode
                     loop). Holds only the newest encoded frame
                     (latest_jpeg/latest_id under a lock) -- a slow client
                     or a stalled encode never queues up stale frames, both
                     sides just skip straight to whatever is newest.
  MultiCameraServer  runs ONE websockets.serve() and, per connected client,
                     round-robins over every CameraCapture's latest frame,
                     sending whichever changed.

Logging (all flush=True, since this runs backgrounded with stdout
redirected to a file -- a print that isn't flushed can simply never show up
in the log if the process is later killed): CameraCapture.open_camera()
logs every index it probes and why each was rejected; its capture loop
prints a FATAL line (not just a bare traceback) if it can never open a
camera, and otherwise a heartbeat every ~2s with frames/fps/failure counts
so "server is up but browser shows nothing" is diagnosable from the log
alone -- no heartbeat at all -> camera never opened; heartbeat with 0 fps ->
opened but not producing frames; heartbeat healthy but no "client
connected" line -> network/reachability, not the camera.
"""
import asyncio
import socket
import threading
import time
from pathlib import Path

import cv2
import websockets


def _fourcc_str(fourcc_int: int) -> str:
    return "".join(chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4))


def _video_device_name(idx: int) -> str:
    """Best-effort /dev/videoN -> V4L2 device name, via the standard sysfs
    node. Used to tell two different UVC cameras apart when auto-probing
    (see CameraCapture.name_filter) -- returns "" if unreadable rather than
    raising, since this is advisory, not required. NOTE: this is NOT the
    same string `lsusb` reports (confirmed empirically 2026-07-09: lsusb
    showed "HSTD USB3.0 Camera" for a device whose V4L2 name is the more
    generic "USB3.0 Camera: USB3.0 Camera") -- always match against what
    this function actually returns (visible in the probe log lines below),
    not against lsusb output.
    """
    try:
        return Path(f"/sys/class/video4linux/video{idx}/name").read_text().strip()
    except OSError:
        return ""


class CameraCapture:
    def __init__(
        self,
        *,
        cam_id: int,
        label: str,
        width: int,
        height: int,
        jpeg_quality: int,
        camera_id: int | None = None,
        max_probe_index: int = 8,
        name_filter: str | None = None,
        heartbeat_sec: float = 2.0,
    ):
        """
        cam_id        1-byte tag prefixed on every WebSocket message sent
                      for this camera (see MultiCameraServer) -- how the
                      browser tells the two streams apart on one socket.
        label         short tag prefixed on every log line (e.g. "front").
        camera_id     explicit /dev/videoN index (e.g. from a CAMERA_ID env
                      var). None = auto-probe.
        name_filter   case-insensitive substring matched against each
                      probed index's V4L2 device name (see
                      _video_device_name). Only used when auto-probing
                      (camera_id is None) -- with two+ cameras on the same
                      box, unfiltered probing risks two capture threads
                      racing for the same /dev/videoN and each other's
                      camera. None = old single-camera behavior (first
                      index that opens AND reads wins).
        """
        self.cam_id = cam_id
        self.label = label
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.camera_id = camera_id
        self.max_probe_index = max_probe_index
        self.name_filter = name_filter
        self.heartbeat_sec = heartbeat_sec

        self.latest_jpeg: bytes | None = None
        self.latest_id = 0
        self.lock = threading.Lock()
        self.running = True

    def _log(self, msg: str) -> None:
        print(f"[{self.label}] {msg}", flush=True)

    def open_camera(self) -> tuple[cv2.VideoCapture, int]:
        """Open a working camera by index.

        If self.camera_id is set, use it directly (bypasses name_filter --
        an explicit index always wins). Otherwise probe indices
        0..max_probe_index, skip any whose V4L2 name doesn't match
        name_filter (when set), and return the first one that both opens
        AND delivers a real frame: USB webcams commonly expose a second
        metadata-only /dev/videoN node that opens fine but never reads, and
        the index a given camera lands on shifts whenever the USB topology
        re-enumerates (e.g. another device unplugged/replugged) -- a
        hardcoded index silently starts pointing at the wrong (or a dead)
        node.
        """
        candidates = [self.camera_id] if self.camera_id is not None else range(self.max_probe_index + 1)
        for idx in candidates:
            name = _video_device_name(idx)
            if self.camera_id is None and self.name_filter and self.name_filter.lower() not in name.lower():
                self._log(f"probe /dev/video{idx}: name={name!r} doesn't match filter {self.name_filter!r}, skipping")
                continue
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                self._log(f"probe /dev/video{idx} (name={name!r}): isOpened() = False, skipping")
                cap.release()
                continue
            ok, _ = cap.read()
            if ok:
                self._log(f"probe /dev/video{idx} (name={name!r}): opened and read() ok -- using this one")
                return cap, idx
            self._log(f"probe /dev/video{idx} (name={name!r}): opened but read() failed (likely a metadata-only node), skipping")
            cap.release()
        tried = f"index {self.camera_id}" if self.camera_id is not None else f"indices 0..{self.max_probe_index}"
        filt = f", name filter {self.name_filter!r}" if self.camera_id is None and self.name_filter else ""
        raise RuntimeError(f"No working camera found (tried {tried}{filt})")

    def _loop(self) -> None:
        try:
            cap, idx = self.open_camera()
        except Exception as e:
            self._log(
                f"FATAL -- {e}. The WebSocket server will keep listening "
                f"(so start_robot.sh's liveness check still passes) but "
                f"this camera will never have a frame to send -- this is "
                f"the 'server up, 0 frames in the browser' symptom."
            )
            raise

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
        self._log(f"streaming camera index {idx} (cam_id={self.cam_id})")
        self._log("Camera configuration:")
        self._log(f"  FOURCC: {negotiated_fourcc!r}" + (" (WARNING: expected 'MJPG')" if negotiated_fourcc != "MJPG" else ""))
        self._log(f"  Width : {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
        self._log(f"  Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        self._log(f"  FPS   : {cap.get(cv2.CAP_PROP_FPS)}")

        frames_ok = frames_read_fail = frames_encode_fail = 0
        last_heartbeat = time.monotonic()

        try:
            while self.running:
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

                now = time.monotonic()
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


class MultiCameraServer:
    """Serves N CameraCapture instances over ONE WebSocket connection per
    client -- each message is [1 byte cam_id][JPEG bytes], so both cameras
    share identical connection-level behavior (TCP_NODELAY, write-buffer
    watermarks, asyncio scheduling) instead of two independent sockets that
    could each stall/jitter differently.
    """

    def __init__(self, *, host: str = "0.0.0.0", port: int, cameras: list[CameraCapture]):
        self.host = host
        self.port = port
        self.cameras = cameras

    async def _stream(self, websocket):
        peer = websocket.remote_address
        print(f"[multicam] client connected from {peer}", flush=True)

        sock = websocket.transport.get_extra_info("socket")
        if sock is not None:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        last_sent = {cam.cam_id: -1 for cam in self.cameras}
        warned_no_frame = {cam.cam_id: False for cam in self.cameras}

        try:
            while True:
                sent_any = False
                for cam in self.cameras:
                    with cam.lock:
                        jpeg = cam.latest_jpeg
                        frame_id = cam.latest_id

                    if jpeg is None:
                        if not warned_no_frame[cam.cam_id]:
                            print(
                                f"[multicam] client {peer}: cam_id={cam.cam_id} ({cam.label}) "
                                f"has no frame yet -- check that camera's startup/heartbeat lines above",
                                flush=True,
                            )
                            warned_no_frame[cam.cam_id] = True
                        continue

                    if frame_id == last_sent[cam.cam_id]:
                        continue

                    last_sent[cam.cam_id] = frame_id
                    await websocket.send(bytes([cam.cam_id]) + jpeg)
                    sent_any = True

                await asyncio.sleep(0 if sent_any else 0.001)
        except websockets.ConnectionClosed:
            print(f"[multicam] client {peer} disconnected", flush=True)

    async def _main(self) -> None:
        for cam in self.cameras:
            cam.start()

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
            print(f"[multicam] Listening on ws://{self.host}:{self.port} ({len(self.cameras)} camera(s))", flush=True)
            await asyncio.Future()

    def run(self) -> None:
        """Blocking: starts every camera's capture thread and serves them
        forever. Call this directly from `if __name__ == "__main__":`.
        """
        try:
            asyncio.run(self._main())
        except KeyboardInterrupt:
            pass
        finally:
            for cam in self.cameras:
                cam.running = False
            print("[multicam] stopped.", flush=True)
