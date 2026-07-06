#!/usr/bin/env python3
"""Operator-side input agent.

Reads the joystick (and GELLO, if present) and publishes robot commands on
Zenoh. The deadman button on the joystick gates all motion: when it is
released, we publish a zero command and deadman="false" so the robot stops.

Base command contract: vx / vy / wz are normalized to [-1, 1]; the robot
agent scales them to physical velocity limits.
"""
import json
import os
import time
from pathlib import Path

# Must be set before `import pygame`: SDL2's HIDAPI joystick backend segfaults
# on macOS for some devices (Thrustmaster T.Flight Stick X confirmed) as soon
# as a HID input report comes in (crash in libSDL2's hid_report_callback,
# reached from pygame.event.pump() — see macOS crash report, faulting thread
# through IOHIDDeviceInputReportApplier). Forcing the legacy IOKit joystick
# backend avoids that codepath entirely.
os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "0")

import pygame
import zenoh

PUBLISH_PERIOD = 0.02  # 50 Hz
DEADMAN_BUTTON = 0     # joystick button index used as deadman

# GELLO integration point. Provide a reader that returns a dict like:
#   {"joints": [...], "gripper": float, "mode": "joint_position"}
# or None when no GELLO is connected. Left as a stub for now.
try:
    from gello_reader import read_gello  # type: ignore
except ImportError:
    def read_gello():
        return None


def load_config() -> zenoh.Config:
    path = Path(__file__).resolve().parent.parent / "config" / "operator_zenoh.json5"
    return zenoh.Config.from_file(str(path))


def main() -> None:
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected.")
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()}")

    with zenoh.open(load_config()) as session:
        pub_base = session.declare_publisher("robot/cmd/base")
        pub_arm = session.declare_publisher("robot/cmd/arm")
        pub_deadman = session.declare_publisher("operator/deadman")
        pub_joystick = session.declare_publisher("operator/input/joystick")

        print("input_agent running. Hold the deadman button to enable motion.")
        try:
            while True:
                pygame.event.pump()
                deadman = bool(joy.get_button(DEADMAN_BUTTON))

                if not deadman:
                    pub_base.put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
                    pub_deadman.put("false")
                    time.sleep(PUBLISH_PERIOD)
                    continue

                pub_deadman.put("true")

                # Axes are already in [-1, 1]. Adjust mapping/signs to your pad.
                vx = -joy.get_axis(1)  # forward = push stick up
                vy = joy.get_axis(0)
                wz = joy.get_axis(2)
                base_cmd = {"vx": float(vx), "vy": float(vy), "wz": float(wz),
                            "ts": time.time()}
                pub_base.put(json.dumps(base_cmd))
                pub_joystick.put(json.dumps(base_cmd))

                gello = read_gello()
                if gello is not None:
                    gello["ts"] = time.time()
                    pub_arm.put(json.dumps(gello))

                time.sleep(PUBLISH_PERIOD)
        except KeyboardInterrupt:
            pass
        finally:
            # Best effort: tell the robot to stop as we leave.
            pub_base.put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
            pub_deadman.put("false")
            print("\ninput_agent stopped.")


if __name__ == "__main__":
    main()
