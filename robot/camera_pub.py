#!/usr/bin/env python3
"""Robot-side camera server -- front HSTD USB3.0 camera.

Captures 1920x1200 and serves it straight to the browser over a raw
WebSocket (ws://<robot-ip>:8765), bypassing Zenoh entirely -- the browser
connects to this process directly (see operator/web/static/js/camera.js).
Zenoh still carries base/arm control and state; only the video path is
direct, because JPEG-over-Zenoh-over-another-WebSocket-hop added latency
for no benefit at this resolution.

The actual server (capture thread, WebSocket delivery, probing, logging) is
shared with robot/insta360_pub.py -- see robot/uvc_camera_server.py's
module docstring for the architecture. This file is just this camera's
config.

Known camera-specific pitfalls (HSTD USB3.0 UVC camera on this robot),
already worked around in uvc_camera_server.py -- do not "fix" these back:
  - Never call cap.set(cv2.CAP_PROP_FPS, ...): requesting an explicit FPS
    this camera doesn't natively expose at this resolution breaks the
    GStreamer pipeline negotiation outright (isOpened() goes False).
  - /dev/videoN is not stable across boots/USB re-enumeration, and USB
    webcams commonly expose a second metadata-only node that opens but
    never reads -- open_camera() probes indices and keeps the first one
    that both opens AND delivers a real frame.
  - Now that a second camera (Insta360) is also on this robot,
    NAME_FILTER pins probing to this camera's V4L2 device name (NOT the
    same string as `lsusb` shows -- `lsusb` reports "HSTD USB3.0 Camera"
    but V4L2's own /sys/class/video4linux/videoN/name is the more generic
    "USB3.0 Camera: USB3.0 Camera", confirmed from logs/camera_pub.log's
    probe lines on 2026-07-09) so this process can never accidentally grab
    the Insta360's /dev/videoN instead (and vice versa) -- without it,
    whichever process's open_camera() reaches a shared index first would
    silently win it, possibly swapping which stream shows which camera.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from uvc_camera_server import CameraServer  # noqa: E402

PORT = 8765
WIDTH = 1920
HEIGHT = 1200
JPEG_QUALITY = 60
NAME_FILTER = "USB3.0 Camera"  # confirmed via logs/camera_pub.log's probe lines (2026-07-09) -- lsusb's "HSTD USB3.0 Camera" string is NOT what V4L2 reports

if __name__ == "__main__":
    env_id = os.environ.get("CAMERA_ID")
    CameraServer(
        label="camera_pub",
        port=PORT,
        width=WIDTH,
        height=HEIGHT,
        jpeg_quality=JPEG_QUALITY,
        name_filter=NAME_FILTER,
    ).run(int(env_id) if env_id is not None else None)
