#!/usr/bin/env python3
"""Robot-side camera publisher.

Reads a camera, encodes each frame as JPEG, and publishes it on Zenoh.

MVP: JPEG over Zenoh at ~15 FPS in 640x480. For true low latency at high
resolution, move the video to H.264/WebRTC and keep Zenoh for commands,
state, heartbeat, and supervision.
"""
import json
import os
import time
from pathlib import Path

import cv2
import zenoh

CAMERA_ID = int(os.environ.get("CAMERA_ID", "0"))
KEY = "robot/camera/front/jpeg"
WIDTH, HEIGHT, FPS = 640, 480, 15
JPEG_QUALITY = 70


def load_config() -> zenoh.Config:
    path = Path(__file__).resolve().parent.parent / "config" / "robot_zenoh.json5"
    config = zenoh.Config.from_file(str(path))
    operator_ip = os.environ.get("OPERATOR_IP")
    if operator_ip:
        config.insert_json5("connect/endpoints", json.dumps([f"tcp/{operator_ip}:7447"]))
    return config


def main() -> None:
    with zenoh.open(load_config()) as session:
        pub = session.declare_publisher(KEY)
        cap = cv2.VideoCapture(CAMERA_ID)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        # Deliberately NOT setting CAP_PROP_FPS: on some UVC cameras (GStreamer
        # backend) requesting an explicit fps the camera doesn't natively
        # expose at this resolution breaks pipeline renegotiation entirely
        # (isOpened() becomes False). We instead read at the camera's native
        # rate and throttle publishing below via frame_period.

        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {CAMERA_ID}")

        frame_period = 1.0 / FPS
        print(f"camera_pub streaming camera {CAMERA_ID} on '{KEY}'")

        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.1)
                    continue

                ok, jpg = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                )
                if ok:
                    pub.put(jpg.tobytes())

                time.sleep(frame_period)
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            print("\ncamera_pub stopped.")


if __name__ == "__main__":
    main()
