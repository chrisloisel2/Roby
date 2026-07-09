#!/usr/bin/env python3
"""Robot-side camera server -- both cameras, one WebSocket connection.

Serves the front camera and a second UVC camera straight to the browser
over a single raw WebSocket (ws://<robot-ip>:8765), bypassing Zenoh
entirely -- the browser connects to this process directly (see
operator/web/static/js/videoMux.js + camera.js/camera2.js). Zenoh still
carries base/arm control and state; only video is direct, because
JPEG-over-Zenoh-over-another-WebSocket-hop added latency for no benefit at
this resolution.

Both cameras share the SAME WebSocket connection/port -- deliberately, not
two separate servers -- so they get byte-for-byte identical connection-level
treatment (same TCP_NODELAY socket, same write-buffer watermarks, same
asyncio scheduling) instead of two independent sockets that could each
stall/jitter differently. Each message is [1 byte cam_id][JPEG bytes]; the
browser demuxes by that prefix. See robot/uvc_camera_server.py for the
actual capture-thread + multiplexed-server implementation; this file is
just both cameras' config.

Known camera-specific pitfalls (front: HSTD USB3.0 UVC camera on this
robot), already worked around in uvc_camera_server.py -- do not "fix" these
back:
  - Never call cap.set(cv2.CAP_PROP_FPS, ...): requesting an explicit FPS
    this camera doesn't natively expose at this resolution breaks the
    GStreamer pipeline negotiation outright (isOpened() goes False).
  - /dev/videoN is not stable across boots/USB re-enumeration, and USB
    webcams commonly expose a second metadata-only node that opens but
    never reads -- CameraCapture.open_camera() probes indices and keeps
    the first one that both opens AND delivers a real frame.
  - With two cameras auto-probing the same /dev/videoN range, NAME_FILTER
    pins each to its own V4L2 device name so neither can accidentally grab
    the other's index -- without it, whichever capture thread reaches a
    shared index first would silently win it, possibly swapping which
    stream shows which camera. IMPORTANT: the V4L2 name is NOT the same
    string `lsusb` reports (confirmed 2026-07-09: lsusb showed "HSTD
    USB3.0 Camera", V4L2 reported "USB3.0 Camera: USB3.0 Camera") -- always
    check logs/camera_pub.log's probe lines for the real name, never guess
    from lsusb.
  - The front camera's two /dev/videoN nodes (capture + metadata-only) both
    report the SAME generic V4L2 name here ("USB3.0 Camera: USB3.0
    Camera") -- if a second camera happens to report that same generic
    uvcvideo-driver name too, NAME_FILTER can't tell them apart by name
    alone. Pin SECOND_CAMERA_ID=<index> (env var) in that case instead of
    relying on SECOND_NAME_FILTER -- find the right index from
    logs/camera_pub.log's probe lines (whichever /dev/videoN isn't the
    front camera's).
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from uvc_camera_server import CameraCapture, MultiCameraServer  # noqa: E402

PORT = 8765

CAM_ID_FRONT = 0
CAM_ID_SECOND = 1

FRONT_WIDTH, FRONT_HEIGHT = 1920, 1200
FRONT_JPEG_QUALITY = 60
FRONT_NAME_FILTER = "USB3.0 Camera"  # confirmed via logs/camera_pub.log (2026-07-09)

# Second camera: a plain second UVC webcam (no longer an Insta360). Its
# V4L2 name is unknown until it's actually plugged in and probed -- check
# logs/camera_pub.log's probe lines the first time this runs, then set
# SECOND_NAME_FILTER (or pin SECOND_CAMERA_ID) below to match. Resolution/
# quality here are a starting guess; the startup log always prints what
# actually got negotiated (Camera configuration: Width/Height/FOURCC/FPS).
SECOND_WIDTH, SECOND_HEIGHT = 1920, 1080
SECOND_JPEG_QUALITY = 60
SECOND_NAME_FILTER = None

if __name__ == "__main__":
    front_id = os.environ.get("CAMERA_ID")
    second_id = os.environ.get("SECOND_CAMERA_ID")
    second_name_filter = os.environ.get("SECOND_NAME_FILTER", SECOND_NAME_FILTER)

    cameras = [
        CameraCapture(
            cam_id=CAM_ID_FRONT,
            label="front",
            width=FRONT_WIDTH,
            height=FRONT_HEIGHT,
            jpeg_quality=FRONT_JPEG_QUALITY,
            camera_id=int(front_id) if front_id is not None else None,
            name_filter=os.environ.get("NAME_FILTER", FRONT_NAME_FILTER),
        ),
    ]

    if second_id is not None or second_name_filter:
        cameras.append(
            CameraCapture(
                cam_id=CAM_ID_SECOND,
                label="second",
                width=SECOND_WIDTH,
                height=SECOND_HEIGHT,
                jpeg_quality=SECOND_JPEG_QUALITY,
                camera_id=int(second_id) if second_id is not None else None,
                name_filter=second_name_filter,
            )
        )
    else:
        # Refuse to auto-probe the second camera unfiltered: with the front
        # camera's own capture thread probing the same /dev/videoN range at
        # the same time, an unfiltered second probe could win the race for
        # the front camera's index on some runs and lose it on others --
        # nondeterministic, silent camera-swapping. Safer to just run
        # front-only until the second camera's identity is known. The front
        # camera's own probe already logs every index's real V4L2 name (see
        # logs/camera_pub.log) even for indices it skips -- use that output
        # to pick SECOND_CAMERA_ID or SECOND_NAME_FILTER once the second
        # camera is plugged in.
        print(
            "[camera_pub] SECOND_CAMERA_ID / SECOND_NAME_FILTER not set -- "
            "running with the front camera only. The [front] probe lines "
            "just below list every /dev/videoN's real V4L2 name -- use "
            "that to set SECOND_CAMERA_ID=<index> or "
            "SECOND_NAME_FILTER='<name>' and add the second camera.",
            flush=True,
        )

    MultiCameraServer(port=PORT, cameras=cameras).run()
