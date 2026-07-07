#!/usr/bin/env bash
# Robot PC: start the robot agent and camera publisher.
# Set OPERATOR_IP to the operator PC address, e.g.:
#     OPERATOR_IP=192.168.15.106 scripts/start_robot.sh
#
# Idempotent: kills any already-running instance of each component first, so
# re-running this script never leaves duplicates racing each other over CAN
# commands or the camera device.
set -euo pipefail
cd "$(dirname "$0")/.."

# Always use the dedicated .venv (created with --system-site-packages, see
# README) if present, NEVER the bare `python` from whatever shell/conda env
# happens to be active: conda's own opencv-python wheel has no GStreamer
# support and fails outright on this robot's camera ("can't open camera by
# index"), while .venv inherits the working system cv2.
if [ -x .venv/bin/python3 ]; then
    PY=.venv/bin/python3
else
    echo "AVERTISSEMENT: .venv introuvable — utilisation de python3 du PATH." >&2
    echo "  Voir README § Installation pour créer .venv (--system-site-packages)." >&2
    PY=python3
fi

# Best-effort graceful stop first: robot_agent.py's SIGINT/KeyboardInterrupt
# handler disables the motors (disable_all()) before exiting, and
# camera_pub.py's releases the camera handle -- both matter more than just
# tidiness. Escalates to SIGKILL if a match is still alive after ~1.5s so
# this is idempotent either way (the new robot_agent.py's own control loop
# starts every wheel at commanded-zero regardless, per its stop_robot()
# fail-safe, so a forced kill here is not itself unsafe -- just less clean).
stop_running() {
    local pattern="$1" pids
    pids=$(pgrep -f "$pattern" || true)
    [ -z "$pids" ] && return 0
    echo "start_robot.sh: stopping existing '$pattern' ($pids)" >&2
    kill -INT $pids 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        pgrep -f "$pattern" >/dev/null || return 0
        sleep 0.3
    done
    kill -9 $pids 2>/dev/null || true
    sleep 0.3
}

stop_running "robot/robot_agent.py"
stop_running "robot/camera_pub.py"

"$PY" robot/robot_agent.py &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null || true' EXIT

"$PY" robot/camera_pub.py
