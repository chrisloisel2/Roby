#!/usr/bin/env python3
"""Generic UVC-webcam-to-WebSocket server.

Shared by robot/camera_pub.py (front HSTD camera) and
robot/insta360_pub.py (Insta360, in USB webcam mode) -- both PCs' cameras
are plugged into the same robot PC and each gets its own CameraServer
instance (own port, own process, launched separately by
scripts/start_robot.sh) so one camera stalling or dying never affects the
other.

Architecture (see either caller's module docstring for the "why direct
WebSocket, not Zenoh" rationale): capture (camera_loop, background thread)
and delivery (stream, per client) are decoupled -- the thread always holds
only the newest encoded frame (latest_jpeg/latest_id under a lock), and each
client coroutine sends it the instant it changes. A slow client or a
stalled encode never queues up stale frames -- both sides just skip
straight to whatever is newest.

Logging (all flush=True, since this runs backgrounded with stdout
redirected to a file -- a print that isn't flushed can simply never show up
in the log if the process is later killed): open_camera() logs every index
it probes and why each was rejected; camera_loop() prints a FATAL line (not
just a bare traceback) if it can never open a camera, and otherwise a
heartbeat every ~2s with frames/fps/failure counts so "server is up but
browser shows nothing" is diagnosable from the log alone -- no heartbeat at
all -> camera never opened; heartbeat with 0 fps -> opened but not
producing frames; heartbeat healthy but no "client connected" line ->
network/reachability, not the camera.
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
    """Best-effort /dev/videoN -> USB product string, via the standard V4L2
    sysfs node. Used to tell two different UVC cameras apart when
    auto-probing (see CameraServer.name_filter) -- returns "" if unreadable
    rather than raising, since this is advisory, not required.
    """
    try:
        return Path(f"/sys/class/video4linux/video{idx}/name").read_text().strip()
    except OSError:
        return ""


class CameraServer:
    def __init__(
        self,
        *,
        label: str,
        host: str = "0.0.0.0",
        port: int,
        width: int,
        height: int,
        jpeg_quality: int,
        max_probe_index: int = 8,
        name_filter: str | None = None,
        heartbeat_sec: float = 2.0,
    ):
        """
        label         short tag prefixed on every log line (e.g. "camera_pub").
        name_filter   case-insensitive substring matched against each probed
                      index's /dev/videoN USB product name (via sysfs). Only
                      used when auto-probing (i.e. no explicit index is
                      passed to open_camera) -- with two+ cameras on the same
                      box, unfiltered index probing risks two server
                      processes racing for the same /dev/videoN and each
                      other's camera. None = old single-camera behavior
                      (first index that opens AND reads wins).
        """
        self.label = label
        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.max_probe_index = max_probe_index
        self.name_filter = name_filter
        self.heartbeat_sec = heartbeat_sec

        self.latest_jpeg: bytes | None = None
        self.latest_id = 0
        self.lock = threading.Lock()
        self.running = True

    def _log(self, msg: str) -> None:
        print(f"[{self.label}] {msg}", flush=True)

    def open_camera(self, camera_id: int | None) -> tuple[cv2.VideoCapture, int]:
        """Open a working camera by index.

        If ``camera_id`` is given, use it directly (bypasses name_filter --
        an explicit index always wins). Otherwise probe indices
        0..max_probe_index, skip any whose sysfs name doesn't match
        name_filter (when set), and return the first one that both opens
        AND delivers a real frame: USB webcams commonly expose a second
        metadata-only /dev/videoN node that opens fine but never reads, and
        the index a given camera lands on shifts whenever the USB topology
        re-enumerates (e.g. another device unplugged/replugged) -- a
        hardcoded index silently starts pointing at the wrong (or a dead)
        node.
        """
        candidates = [camera_id] if camera_id is not None else range(self.max_probe_index + 1)
        for idx in candidates:
            name = _video_device_name(idx)
            if camera_id is None and self.name_filter and self.name_filter.lower() not in name.lower():
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
        tried = f"index {camera_id}" if camera_id is not None else f"indices 0..{self.max_probe_index}"
        filt = f", name filter {self.name_filter!r}" if camera_id is None and self.name_filter else ""
        raise RuntimeError(f"No working camera found (tried {tried}{filt})")

    def camera_loop(self, camera_id: int | None) -> None:
        try:
            cap, idx = self.open_camera(camera_id)
        except Exception as e:
            self._log(
                f"FATAL -- {e}. The WebSocket server will keep listening "
                f"(so start_robot.sh's liveness check still passes) but "
                f"will never have a frame to send -- this is the 'server "
                f"up, 0 frames in the browser' symptom."
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
        # either camera needs an explicit FPS request, every CameraServer
        # instance skips it.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # Keep the driver's internal buffer at 1 frame so cap.read() always
        # returns the newest frame instead of draining a backlog queued
        # while we were busy encoding/sending the previous one.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        negotiated_fourcc = _fourcc_str(int(cap.get(cv2.CAP_PROP_FOURCC)))
        self._log(f"streaming camera index {idx} on ws://{self.host}:{self.port}")
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

    async def stream(self, websocket):
        peer = websocket.remote_address
        self._log(f"client connected from {peer}")

        sock = websocket.transport.get_extra_info("socket")
        if sock is not None:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        last_sent_id = -1
        warned_no_frame = False

        try:
            while True:
                with self.lock:
                    jpeg = self.latest_jpeg
                    frame_id = self.latest_id

                if jpeg is None:
                    if not warned_no_frame:
                        self._log(f"client {peer} connected but no frame captured yet -- check the camera_loop startup/heartbeat lines above")
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
            self._log(f"client {peer} disconnected")

    async def _main(self, camera_id: int | None) -> None:
        thread = threading.Thread(target=self.camera_loop, args=(camera_id,), daemon=True)
        thread.start()

        async with websockets.serve(
            self.stream,
            self.host,
            self.port,
            max_size=None,
            max_queue=1,
            compression=None,
            ping_interval=None,
            write_limit=(1024 * 1024, 0),  # (high, low) write-buffer watermarks; see stream()
        ):
            self._log(f"Listening on ws://{self.host}:{self.port}")
            await asyncio.Future()

    def run(self, camera_id: int | None) -> None:
        """Blocking: opens the camera and serves it forever. Call this
        directly from `if __name__ == "__main__":` in each caller script.
        """
        try:
            asyncio.run(self._main(camera_id))
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self._log("stopped.")
