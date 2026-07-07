#!/usr/bin/env python3
"""Robot-side camera publisher.

Reads a camera, encodes each frame as JPEG, and publishes it on Zenoh.

MVP: JPEG over Zenoh at ~30 FPS in 1920x1080. For true low latency at high
resolution, move the video to H.264/WebRTC and keep Zenoh for commands,
state, heartbeat, and supervision.
"""
import json
import os
import time
from pathlib import Path

import cv2
import zenoh

KEY = "robot/camera/front/jpeg"
WIDTH, HEIGHT, FPS = 1920, 1080, 30
JPEG_QUALITY = 70
MAX_PROBE_INDEX = 8  # highest /dev/videoN index to try when auto-detecting


def load_config() -> zenoh.Config:
    path = Path(__file__).resolve().parent.parent / "config" / "robot_zenoh.json5"
    config = zenoh.Config.from_file(str(path))
    operator_ip = os.environ.get("OPERATOR_IP")
    if operator_ip:
        config.insert_json5("connect/endpoints", json.dumps([f"tcp/{operator_ip}:7447"]))
    return config


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
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                return cap, idx
        cap.release()
    tried = f"index {camera_id}" if camera_id is not None else f"indices 0..{MAX_PROBE_INDEX}"
    raise RuntimeError(f"No working camera found (tried {tried})")


def main() -> None:
    with zenoh.open(load_config()) as session:
        # DROP + express: under network congestion, prefer dropping a stale
        # frame over queueing it — queueing is exactly what turns a slow link
        # into ever-growing latency instead of just a lower delivered fps.
        pub = session.declare_publisher(
            KEY,
            congestion_control=zenoh.CongestionControl.DROP(),
            express=True,
        )
        env_id = os.environ.get("CAMERA_ID")
        cap, camera_id = open_camera(int(env_id) if env_id is not None else None)
        # Most UVC webcams only expose their raw (YUYV) format at high
        # resolutions like 1080p at a few fps — USB bandwidth for uncompressed
        # video that large is too high. Requesting MJPG (compressed in the
        # camera's own hardware) is what actually unlocks 30fps at 1080p; this
        # must be set before the resolution for the driver to renegotiate.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        # Keep the driver's internal buffer at 1 frame so cap.read() always
        # returns the newest frame instead of draining a backlog that was
        # queued while we were busy encoding/publishing the previous one.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Deliberately NOT setting CAP_PROP_FPS: on some UVC cameras (GStreamer
        # backend) requesting an explicit fps the camera doesn't natively
        # expose at this resolution breaks pipeline renegotiation entirely
        # (isOpened() becomes False). We instead read at the camera's native
        # rate and throttle publishing below via frame_period.

        frame_period = 1.0 / FPS
        print(f"camera_pub streaming camera {camera_id} on '{KEY}'")

        try:
            next_tick = time.monotonic()
            while True:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.1)
                    next_tick = time.monotonic()
                    continue

                ok, jpg = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                )
                if ok:
                    pub.put(jpg.tobytes())

                # Pace by a fixed deadline instead of a flat sleep: if this
                # iteration ran long (slow encode/publish), don't add a full
                # frame_period on top of that overrun, and don't try to burst
                # extra frames to catch up either — just resync to "now" and
                # keep going. A flat sleep-after-work compounds any overrun
                # frame after frame, which is how latency creeps upward over
                # time instead of staying flat.
                next_tick += frame_period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            print("\ncamera_pub stopped.")


if __name__ == "__main__":
    main()
