#!/usr/bin/env python3
"""Robot-side camera server -- Insta360, plugged into the robot PC in USB
webcam mode.

Same architecture as robot/camera_pub.py (see that file and
robot/uvc_camera_server.py's module docstring for the "why"): captures from
the camera and serves it straight to the browser over its own WebSocket
(ws://<robot-ip>:8766 -- a different port from camera_pub.py's 8765, since
both run as separate processes at the same time), bypassing Zenoh entirely.

NAME_FILTER pins auto-probing to this camera's USB product name so this
process can't accidentally grab the front HSTD camera's /dev/videoN (or
vice versa) -- see camera_pub.py's docstring for why that matters now that
two cameras share the same probing scheme. "insta360" is a best guess for
the substring V4L2 reports (via /sys/class/video4linux/videoN/name); if
this camera doesn't actually show up, the startup log will print every
probed index's real name (see uvc_camera_server.CameraServer.open_camera) --
fix NAME_FILTER below to match, or pin INSTA360_CAMERA_ID directly to skip
probing.

Resolution/quality here are a starting guess (Insta360 webcam-mode UVC
output varies by model); the startup log always prints what actually got
negotiated (Camera configuration: Width/Height/FOURCC/FPS) -- adjust WIDTH/
HEIGHT/JPEG_QUALITY below to match once you've seen that.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from uvc_camera_server import CameraServer  # noqa: E402

PORT = 8766
WIDTH = 1920
HEIGHT = 1080
JPEG_QUALITY = 60
NAME_FILTER = "insta360"

if __name__ == "__main__":
    env_id = os.environ.get("INSTA360_CAMERA_ID")
    CameraServer(
        label="insta360_pub",
        port=PORT,
        width=WIDTH,
        height=HEIGHT,
        jpeg_quality=JPEG_QUALITY,
        name_filter=NAME_FILTER,
    ).run(int(env_id) if env_id is not None else None)
