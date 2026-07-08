#!/usr/bin/env bash
# Robot PC: start the arm agent (reBot B601 follower).
#
# Deliberately NOT folded into start_robot.sh: the base (mecanum) control
# path is already validated on real hardware; the arm path (this script) is
# brand new. Keeping them separate means restarting/testing the base never
# implicitly re-arms the arm, and vice versa. Fold this in once the arm path
# has had a real bring-up.
#
# Runs robot/arm_agent.py with the `lerobot` conda env's python (NOT the
# project's own .venv): RebotB601Follower is reused as-is from
# ~/03_JelloSoft/rebot_lerobot/lerobot, which only imports cleanly inside
# that env. Override with ARM_PYTHON=/path/to/python if the conda env lives
# somewhere else on this machine.
#
# Idempotent, same pattern as start_robot.sh: kills any already-running
# instance first so a re-run never leaves two processes racing over the same
# CAN bus / arm.
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${ARM_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python3}"
if [ ! -x "$PY" ]; then
    echo "start_arm.sh: python introuvable at '$PY'." >&2
    echo "  Vérifie l'env conda 'lerobot' (voir ~/03_JelloSoft/rebot_lerobot/scripts/README.md)" >&2
    echo "  ou passe ARM_PYTHON=/chemin/vers/python explicitement." >&2
    exit 1
fi

stop_running() {
    local pattern="$1" pids
    pids=$(pgrep -f "$pattern" || true)
    [ -z "$pids" ] && return 0
    echo "start_arm.sh: stopping existing '$pattern' ($pids)" >&2
    kill -INT $pids 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        pgrep -f "$pattern" >/dev/null || return 0
        sleep 0.3
    done
    kill -9 $pids 2>/dev/null || true
    sleep 0.3
}

stop_running "robot/arm_agent.py"

echo "start_arm.sh: bras dégagé/soutenu ? connect() active le couple moteur immédiatement." >&2

"$PY" robot/arm_agent.py
