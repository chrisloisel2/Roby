#!/usr/bin/env python3
"""Bench test: spin the new DAMIAO DM2325 tool motor (screwdriver) on its own,
independent of the arm / robot_agent.py. Just confirms it's wired up, has the
right CAN/master ID, and turns -- nothing else touches it.

Motor is assumed already configured on the motor itself (via the Damiao
assistant tool) as DM2325, mode 3 = VEL_MODE (speed mode), CAN ID 0x09,
master ID 0x19 -- matching the 9th can_id/mst_id slot already reserved (but
unused) in damiao.py's own __main__ block. Override with --can-id/--master-id
if that guess is wrong.

Usage
-----
    python3 robot/test_screwdriver_motor.py                  # spin 3s at 3 rad/s
    python3 robot/test_screwdriver_motor.py --vel 6 --duration 5
    python3 robot/test_screwdriver_motor.py --can-id 0x09 --master-id 0x19

Ctrl+C stops it safely at any time (ramps won't happen -- VEL_MODE has no
ramp -- it just cuts to disable_all() immediately).
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dmcan import dmcan_device_type
from damiao import DM_Motor_Type, Control_Mode, DmActData, Motor_Control

CONTROL_PERIOD = 0.01  # 100 Hz, same as robot_agent.py -- motor needs a steady stream of frames or it times out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--can-id", type=lambda s: int(s, 0), default=0x09, help="motor CAN id (default 0x09)")
    p.add_argument("--master-id", type=lambda s: int(s, 0), default=0x19, help="motor master/feedback id (default 0x19)")
    p.add_argument("--vel", type=float, default=3.0, help="target speed in rad/s (default 3.0, modest for a first spin)")
    p.add_argument("--duration", type=float, default=3.0, help="seconds to run before auto-stop (default 3.0)")
    p.add_argument("--device-type", default="USB2CANFD", choices=["USB2CANFD", "USB2CANFD_DUAL"],
                    help="dmcan adapter type (default USB2CANFD, matches robot_agent.py)")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    init_data = [DmActData(
        motorType=DM_Motor_Type.DMS2325,
        mode=Control_Mode.VEL_MODE,
        can_id=args.can_id,
        mst_id=args.master_id,
    )]

    print(f"test_screwdriver_motor: opening CAN adapter ({args.device_type})...")
    ctrl = Motor_Control(
        1_000_000, 1_000_000, "", init_data,
        device_type=getattr(dmcan_device_type, args.device_type),
        canfd=False, brs=False,
    )
    motor = ctrl.getMotor(args.can_id)
    if motor is None:
        print("test_screwdriver_motor: motor not found after init -- check --can-id/--master-id.", file=sys.stderr)
        os._exit(1)

    try:
        print(f"test_screwdriver_motor: spinning can_id=0x{args.can_id:02X} at {args.vel} rad/s for {args.duration}s "
              "(Ctrl+C to stop early)...")
        start = time.perf_counter()
        last_print = 0.0
        while True:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= args.duration:
                break
            ctrl.control_vel(motor, args.vel)
            if now - last_print >= 0.2:
                last_print = now
                print(f"  t={elapsed:4.1f}s  pos={motor.Get_Position():+7.3f}  "
                      f"vel={motor.Get_Velocity():+7.3f}  tau={motor.Get_tau():+6.3f}  err={motor.Get_err()}")
            time.sleep(CONTROL_PERIOD)
    except KeyboardInterrupt:
        print("\ntest_screwdriver_motor: interrupted, stopping.")
    finally:
        # Cut to zero velocity briefly, then hard-disable. NOT a `with
        # Motor_Control(...)` block on purpose -- Motor_Control.close() /
        # DmCanContext.__del__ crashes the process (libusb assertion, native
        # lib bug), same issue documented in robot_agent.py. disable_all()
        # always completes first, so the motor is safely stopped regardless.
        for _ in range(5):
            ctrl.control_vel(motor, 0.0)
            time.sleep(CONTROL_PERIOD)
        ctrl.disable_all()
        print("test_screwdriver_motor: motor disabled, exiting.")
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
