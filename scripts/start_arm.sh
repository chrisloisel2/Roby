#!/usr/bin/env bash
# Robot PC: start ONLY the arm agent (reBot B601 follower), standalone.
#
# Normal operation now goes through scripts/start_robot.sh, which starts the
# base, camera, AND arm together (folded in 2026-07-08 after the arm path's
# real bring-up). Keep using THIS script for isolated arm-only work --
# testing/restarting the arm alone without touching the already-validated
# base, e.g. while iterating on calibration or ARM_PORT.
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

# OPERATOR_IP is required (robot/zenoh_config.py enforces it too, but
# failing here is friendlier -- before connect() below energizes the arm
# motors). See that module's docstring: a stale hardcoded default IP used
# to make the base's robot_agent.py silently connect nowhere; requiring the
# IP explicitly everywhere on this PC closes that whole bug class for good.
if [ -z "${OPERATOR_IP:-}" ]; then
    echo "start_arm.sh: OPERATOR_IP est requis, ex:" >&2
    echo "    OPERATOR_IP=192.168.15.111 scripts/start_arm.sh" >&2
    exit 1
fi

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
