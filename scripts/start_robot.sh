#!/usr/bin/env bash
# Robot PC: start the robot agent and camera publisher.
# Set OPERATOR_IP to the operator PC address, e.g.:
#     OPERATOR_IP=192.168.15.106 scripts/start_robot.sh
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

"$PY" robot/robot_agent.py &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null || true' EXIT

"$PY" robot/camera_pub.py
