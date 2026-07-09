#!/usr/bin/env python3
"""Measure real follower joint_limits by sweeping the GELLO leader by hand.

Why this exists
----------------
robot/arm_agent.py's follower (RebotB601Follower, vendored in
~/03_JelloSoft/rebot_lerobot/... on the robot PC) clips every commanded
position against RebotB601FollowerRobotConfig.joint_limits before sending it
to a motor. Confirmed 2026-07-08: those limits were never actually measured
-- RebotB601Follower.calibrate() just copies the hardcoded config default
straight into the calibration file, with no range-of-motion sweep at all.
At least shoulder_lift (max=1.0deg) and gripper (min=-270deg) are too
narrow for real GELLO output, clipping those joints (and elbow_flex, right
at the edge) to a near-frozen position during normal teleop -- see the
"BLOQUE" / "BLOCKED" flag in this script's report for exactly which ones.

This script re-measures the real usable range straight from the actual
control input device (GELLO), instead of guessing new numbers. It reuses
gello_reader.GelloReader as-is -- the exact same serial-read + calibration
transform arm_agent.py's real commands are built from -- so what gets
measured here is exactly what would be SENT to the follower in normal
operation, not some independently-derived number.

Safety
------
Reads the GELLO leader ONLY. Never opens a connection to the follower arm,
never touches a robot motor, never enables torque anywhere. The leader is a
passive, hand-manipulated sensor rig (no motors) -- there is no possible
physical risk from running this, at any point, for as long as you want.

Usage
-----
    GELLO_PORT=/dev/tty.usbserial-XXXX python3 operator/calibrate_arm_limits.py [--margin DEG]

Press Enter to start recording, then move the GELLO by hand through the
FULL safe range of motion you want the follower to be able to reach, for
EVERY joint (a few slow end-to-end sweeps per joint is enough -- recording
only needs to see each extreme once, so take your time and don't force
anything). Press Ctrl+C when done to get the report.

Output
------
Prints a comparison table (measured vs currently-configured joint_limits,
flagging which joints the current config actually clips), a ready-to-paste
Python snippet for config_rebot_b601_follower.py's joint_limits field, and
saves the full measurement as JSON next to this script (arm_limits_measured
.json) so it stays available as a reference after the terminal closes.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from gello_reader import GelloReader, JOINT_IDS, PORT_ENV

# Mirror of RebotB601FollowerRobotConfig.joint_limits (config_rebot_b601_
# follower.py on the robot PC, as of 2026-07-08) -- kept here purely so the
# report below can flag which joints the measured range actually exceeds.
# Not read from disk: that file lives on the robot PC, not here. Update this
# constant by hand if the follower's config changes.
CURRENT_FOLLOWER_JOINT_LIMITS = {
    "shoulder_pan": (-150.0, 150.0),
    "shoulder_lift": (-200.0, 1.0),
    "elbow_flex": (-200.0, 1.0),
    "wrist_flex": (-80.0, 90.0),
    "wrist_yaw": (-90.0, 90.0),
    "wrist_roll": (-90.0, 90.0),
    "gripper": (-270.0, 0.0),
}

OUTPUT_PATH = Path(__file__).resolve().parent / "arm_limits_measured.json"
POLL_PERIOD = 0.02  # 50Hz -- plenty to catch a hand-speed sweep's extremes


def record_sweep(reader: GelloReader) -> tuple[dict[str, float], dict[str, float], int, float]:
    observed_min: dict[str, float] = {}
    observed_max: dict[str, float] = {}
    samples = 0
    t_start = time.time()
    try:
        while True:
            action = reader.get_action()
            for name in JOINT_IDS:
                val = action.get(name)
                if val is None:
                    continue
                observed_min[name] = val if name not in observed_min else min(observed_min[name], val)
                observed_max[name] = val if name not in observed_max else max(observed_max[name], val)
            samples += 1

            elapsed = time.time() - t_start
            line = f"\r[{elapsed:6.1f}s, {samples:5d} reads]  "
            line += "  ".join(
                f"{name}:{observed_min.get(name, float('nan')):+7.1f}.."
                f"{observed_max.get(name, float('nan')):+7.1f}"
                for name in JOINT_IDS
            )
            print(line, end="", flush=True)
            time.sleep(POLL_PERIOD)
    except KeyboardInterrupt:
        print("\n\nRecording stopped.\n")
    return observed_min, observed_max, samples, time.time() - t_start


def print_report(
    observed_min: dict[str, float],
    observed_max: dict[str, float],
    margin: float,
) -> dict[str, tuple[float, float]]:
    print(f"{'joint':<15} {'meas min':>10} {'meas max':>10}  "
          f"{'current limit':>17}  {'proposed (+/-margin)':>21}  status")
    print("-" * 100)
    proposed: dict[str, tuple[float, float]] = {}
    any_blocked = False
    for name in JOINT_IDS:
        if name not in observed_min:
            print(f"{name:<15}  no reading -- this joint was never moved during the recording")
            continue
        mn, mx = observed_min[name], observed_max[name]
        cur_min, cur_max = CURRENT_FOLLOWER_JOINT_LIMITS.get(name, (float("-inf"), float("inf")))
        prop_min, prop_max = round(mn - margin, 1), round(mx + margin, 1)
        proposed[name] = (prop_min, prop_max)
        blocked = mn < cur_min or mx > cur_max
        any_blocked = any_blocked or blocked
        status = "BLOCKED by current limit" if blocked else "ok"
        print(f"{name:<15} {mn:>10.1f} {mx:>10.1f}  "
              f"({cur_min:>6.1f}, {cur_max:>6.1f})  ({prop_min:>7.1f}, {prop_max:>7.1f})  {status}")

    if any_blocked:
        print("\n>>> At least one joint reached, during this recording, a value that the CURRENT")
        print(">>> follower joint_limits would have clipped. Those are almost certainly the")
        print(">>> joints that feel stuck/dead during real teleoperation.")
    else:
        print("\nNo joint exceeded its current configured limit during this recording -- if a "
              "joint still\nfeels stuck in real use, sweep it again covering more of its range.")

    print("\nProposed joint_limits, ready to paste into config_rebot_b601_follower.py "
          "(RebotB601FollowerConfig.joint_limits):\n")
    print("    joint_limits: dict[str, tuple[float, float]] = field(")
    print("        default_factory=lambda: {")
    for name in JOINT_IDS:
        if name in proposed:
            mn, mx = proposed[name]
            print(f'            "{name}": ({mn}, {mx}),')
        else:
            cur = CURRENT_FOLLOWER_JOINT_LIMITS.get(name)
            print(f'            "{name}": {cur},  # not measured -- kept as-is')
    print("        }")
    print("    )")
    return proposed


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--margin", type=float, default=3.0,
        help="Degrees added on each side of the measured range for the proposed limits (default: 3.0)",
    )
    args = parser.parse_args()

    port = os.environ.get(PORT_ENV)
    if not port:
        sys.exit(f"{PORT_ENV} is not set. Example:\n"
                  f"  {PORT_ENV}=/dev/tty.usbserial-XXXX python3 {Path(__file__).name}")

    print(f"Connecting to GELLO on {port}...")
    reader = GelloReader(port)
    print("Connected.\n")
    print("This script only ever reads the GELLO leader -- the follower arm's motors are")
    print("never contacted, so there is zero physical risk no matter how you move it.")
    print("Move the GELLO by hand through the FULL safe range of motion you want the")
    print("follower to reach, for EVERY joint (a few slow end-to-end sweeps each is enough).")
    input("Press Enter to start recording...")
    print("Recording -- Ctrl+C when done.\n")

    try:
        observed_min, observed_max, samples, duration = record_sweep(reader)
    finally:
        reader.close()

    if not observed_min:
        sys.exit("No valid reading was received -- check the GELLO connection and try again.")

    proposed = print_report(observed_min, observed_max, args.margin)

    record = {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "gello_port": port,
        "margin_deg": args.margin,
        "samples": samples,
        "duration_sec": round(duration, 1),
        "measured_deg": {name: {"min": observed_min[name], "max": observed_max[name]} for name in observed_min},
        "current_follower_joint_limits": CURRENT_FOLLOWER_JOINT_LIMITS,
        "proposed_joint_limits": proposed,
    }
    OUTPUT_PATH.write_text(json.dumps(record, indent=2))
    print(f"\nSaved to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
