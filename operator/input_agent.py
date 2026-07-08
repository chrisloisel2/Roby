#!/usr/bin/env python3
"""Operator-side input agent.

Reads the joystick (and GELLO, if present) and publishes robot commands on
Zenoh. The deadman button on the joystick gates all motion: when it is
released, we publish a zero command and deadman="false" so the robot stops.

Direction is SNAPPED to 8 zones of 45° (mecanum: forward/back, strafe
left/right, 4 diagonals) rather than read as a raw continuous stick value --
same pattern already proven on this robot in
~/catkin_ws/u2canfd/mecanum_control.py. Two deliberate reasons, not just
style:

  1. A raw stick reading with no deadzone lets any analog noise near center
     bleed into vx/vy/wz continuously, which the robot then tries to track
     at 100 Hz -- this is what caused the wheels to shake/buzz on deadman
     press with the previous continuous-axis version of this file.
  2. Cheap sticks rarely return to *exact* (0, 0); a snap-to-nearest-sector
     with a deadzone radius makes "centered" reliably mean "stopped".

Base command contract: vx / vy / wz are normalized to [-1, 1]; vx=forward,
vy=lateral (right +), wz=rotation. The robot agent mixes these into mecanum
wheel targets and scales by its own MAX_VEL/ROT_VEL.
"""
import json
import math
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

# Axes (Thrustmaster T.Flight Stick X: 0=X, 1=Y, 2=twist/rudder, 3=throttle).
AXIS_STICK_X = 0
AXIS_STICK_Y = 1
AXIS_ROTATION = 2
STICK_X_INVERT = False
DEADZONE = 0.25           # stick: below this magnitude -> STOP, not drift
ROTATION_DEADZONE = 0.15  # twist axis: below this -> no rotation, not drift

# GELLO integration point. Provide a reader that returns a dict like:
#   {"joints": [...], "gripper": float, "mode": "joint_position"}
# or None when no GELLO is connected. Left as a stub for now.
try:
    from gello_reader import read_gello  # type: ignore
except ImportError:
    def read_gello():
        return None


# 8 snapped directions, 45° apart: (forward, lateral, label). Matches
# mecanum_control.py's SNAP_TABLE exactly (there named vy=forward,
# vx=lateral; renamed here to this project's vx=forward/vy=lateral
# convention -- same physical mapping, no sign changes).
_D = 1 / math.sqrt(2)
SNAP_TABLE = [
    (  0.0, +1.0, "strafe droite"),    # secteur 0 -> E    0°
    ( +_D,  +_D,  "diag av-droite"),   # secteur 1 -> NE  45°
    ( +1.0,  0.0, "avant"),            # secteur 2 -> N   90°
    ( +_D,  -_D,  "diag av-gauche"),   # secteur 3 -> NW 135°
    (  0.0, -1.0, "strafe gauche"),    # secteur 4 -> W  180°
    ( -_D,  -_D,  "diag ar-gauche"),   # secteur 5 -> SW 225°
    ( -1.0,  0.0, "recule"),           # secteur 6 -> S  270°
    ( -_D,  +_D,  "diag ar-droite"),   # secteur 7 -> SE 315°
]


def snap_direction(joy: pygame.joystick.JoystickType) -> tuple[float, float, str]:
    """Quantize the stick angle into one of the 8 sectors above.

    Returns (0.0, 0.0, "stop") inside the deadzone.
    """
    sx = -joy.get_axis(AXIS_STICK_X) if STICK_X_INVERT else joy.get_axis(AXIS_STICK_X)
    sy = -joy.get_axis(AXIS_STICK_Y)

    if math.hypot(sx, sy) < DEADZONE:
        return 0.0, 0.0, "stop"

    angle = math.degrees(math.atan2(sy, sx)) % 360
    sector = round(angle / 45) % 8
    return SNAP_TABLE[sector]


def read_rotation(joy: pygame.joystick.JoystickType) -> float:
    raw = joy.get_axis(AXIS_ROTATION)
    return 0.0 if abs(raw) < ROTATION_DEADZONE else raw


def load_config() -> zenoh.Config:
    path = Path(__file__).resolve().parent.parent / "config" / "operator_zenoh.json5"
    return zenoh.Config.from_file(str(path))


def main() -> None:
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        # Not fatal: the browser UI (Gamepad API + Web Serial, see
        # operator/web/index.html) can drive both the base and the GELLO on
        # its own now. start_operator.sh backgrounds this process precisely
        # so that having nothing to do here doesn't take zenohd/web_server
        # down with it -- exit clean (not an exception) so that shows up as
        # a plain informational line, not a traceback.
        print("input_agent: no joystick detected -- nothing to do (drive from the browser "
              "instead). Exiting.")
        return
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

                # GELLO arm teleop is intentionally independent of the base's
                # joystick deadman -- operating a 7-DOF leader arm needs both
                # hands, so requiring the joystick button held at the same
                # time isn't workable. Safety for the arm instead comes from
                # the robot-side watchdog on robot/cmd/arm freshness.
                gello = read_gello()
                if gello is not None:
                    gello["ts"] = time.time()
                    pub_arm.put(json.dumps(gello))

                if not deadman:
                    pub_base.put(json.dumps({"vx": 0.0, "vy": 0.0, "wz": 0.0}))
                    pub_deadman.put("false")
                    time.sleep(PUBLISH_PERIOD)
                    continue

                pub_deadman.put("true")

                vx, vy, label = snap_direction(joy)
                wz = read_rotation(joy)

                base_cmd = {"vx": float(vx), "vy": float(vy), "wz": float(wz),
                            "ts": time.time()}
                pub_base.put(json.dumps(base_cmd))
                pub_joystick.put(json.dumps(base_cmd))
                print(f"\r{label:16s} vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}   ", end="")

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
