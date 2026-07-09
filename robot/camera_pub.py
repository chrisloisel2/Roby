#!/usr/bin/env python3
"""Robot-side camera server -- every UVC camera discovered automatically,
one WebSocket connection.

Serves every working /dev/videoN straight to the browser over a single raw
WebSocket (ws://<robot-ip>:8765), bypassing Zenoh entirely -- the browser
connects to this process directly (see
operator/web/static/js/videoMux.js). Zenoh still carries base/arm control
and state; only video is direct, because JPEG-over-Zenoh-over-another-
WebSocket-hop added latency for no benefit at this resolution.

2026-07-10: camera discovery is now fully automatic (robot/
uvc_camera_server.py's CameraManager) -- no more NAME_FILTER/CAMERA_ID/
SECOND_CAMERA_ID/SECOND_NAME_FILTER env vars, and no more "second camera
stays off until you SSH in and set one". Plug in any number of UVC cameras
and they show up on their own within a few seconds (see
CameraManager.discover_every_sec); which one is shown in the main tile vs.
the picture-in-picture thumbnail is now a browser-side setting (operator/
web/static/js/config.js's `cameras.primaryId`/`secondaryId`, picked by name
in the operator's Réglages > Caméras tab) instead of something pinned here
by V4L2 name/index. Rationale for dropping NAME_FILTER specifically: it
only ever existed to stop two separate auto-probing CameraCapture threads
from racing for the same /dev/videoN index -- CameraManager's discovery
loop probes indices one at a time from a single thread, so that race can't
happen anymore regardless of how many cameras are plugged in.

Known camera-specific pitfall (front: HSTD USB3.0 UVC camera on this
robot), already worked around in uvc_camera_server.py -- do not "fix" this
back: never call cap.set(cv2.CAP_PROP_FPS, ...): requesting an explicit FPS
this camera doesn't natively expose at this resolution breaks the
GStreamer pipeline negotiation outright (isOpened() goes False).

Escape hatches (rarely needed -- the defaults below are a fine starting
point for any UVC camera; the startup log's "Camera configuration" block
always shows what actually got negotiated per camera):
  CAMERA_WIDTH / CAMERA_HEIGHT / CAMERA_JPEG_QUALITY   requested capture
        size/quality, applied to every discovered camera alike.
  CAMERA_MAX_INDEX     highest /dev/videoN index to probe (default 8).
  CAMERA_EXCLUDE       comma-separated /dev/videoN indices to never probe
        (e.g. a device on this box that opens+reads but is known-bad).
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from uvc_camera_server import CameraManager, MultiCameraServer  # noqa: E402

PORT = 8765

# 1280x720, not 1920x1080: confirmed directly on this robot's cameras
# (2026-07-11) that dropping to 720p does NOT unlock a higher framerate --
# both resolutions deliver the exact same ~15fps ceiling in MJPG mode (a
# fixed characteristic of this camera model, not a resolution/bandwidth
# effect -- cap.get(CAP_PROP_FPS) always reports a nominal 30 regardless).
# What 720p DOES do is cut each JPEG from ~77-120KB down to ~39-53KB, i.e.
# combined bandwidth for two simultaneous streams sharing one WebSocket
# drops from ~24 Mbps to ~11 Mbps -- real headroom against network
# capacity, which is what was actually causing visible latency to build up
# over a session (any dip below the combined requirement fills the send
# buffer; see write_limit's comment in uvc_camera_server.py). Raise these
# back via CAMERA_WIDTH/CAMERA_HEIGHT if your network can sustain more and
# image detail matters more than smoothness for your use case.
DEFAULT_WIDTH, DEFAULT_HEIGHT = 1280, 720
DEFAULT_JPEG_QUALITY = 60

if __name__ == "__main__":
    exclude = os.environ.get("CAMERA_EXCLUDE", "")
    manager = CameraManager(
        width=int(os.environ.get("CAMERA_WIDTH", DEFAULT_WIDTH)),
        height=int(os.environ.get("CAMERA_HEIGHT", DEFAULT_HEIGHT)),
        jpeg_quality=int(os.environ.get("CAMERA_JPEG_QUALITY", DEFAULT_JPEG_QUALITY)),
        max_probe_index=int(os.environ.get("CAMERA_MAX_INDEX", 8)),
        exclude_indices={int(x) for x in exclude.split(",") if x.strip()},
    )
    MultiCameraServer(port=PORT, manager=manager).run()
