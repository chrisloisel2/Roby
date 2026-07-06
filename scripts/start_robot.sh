#!/usr/bin/env bash
# Robot PC: start the robot agent and camera publisher.
# Set OPERATOR_IP to the operator PC address, e.g.:
#     OPERATOR_IP=192.168.1.50 scripts/start_robot.sh
set -euo pipefail
cd "$(dirname "$0")/.."

python robot/robot_agent.py &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null || true' EXIT

python robot/camera_pub.py
