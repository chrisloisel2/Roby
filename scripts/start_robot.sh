#!/usr/bin/env bash
# Robot PC: start the robot agent, camera publisher, and arm agent -- the
# full robot-side stack in one command.
# Set OPERATOR_IP to the operator PC address, e.g.:
#     OPERATOR_IP=192.168.15.111 scripts/start_robot.sh
#
# Idempotent: kills any already-running instance of each component first, so
# re-running this script never leaves duplicates racing each other over CAN
# commands or the camera device.
#
# OPERATOR_IP is REQUIRED (robot/zenoh_config.py enforces it too, but
# failing here is friendlier -- before touching any hardware). See that
# module's docstring for the incident this whole script hardening replaced
# (2026-07-08): a stale hardcoded default IP made robot_agent.py silently
# connect nowhere, while robot_agent.py's own crash was invisible (no log,
# backgrounded with no liveness check) and camera_pub.py kept working fine
# in the same script run -- from the operator UI it looked like "only the
# motors are broken" for a problem that was actually "robot_agent.py never
# came up at all".
#
# The arm agent (robot/arm_agent.py) used to be its own separate script
# (start_arm.sh) precisely because the arm path was brand new and unproven
# -- see that script's history. It has since had a real bring-up (2026-07-08:
# a read-only motorbridge probe got a live state reply from all 7 joints
# over its corrected ARM_PORT), so it's folded in here as a third required
# component, same fail-fast treatment as the other two. start_arm.sh still
# exists for isolated arm-only testing.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -z "${OPERATOR_IP:-}" ]; then
    echo "start_robot.sh: OPERATOR_IP est requis, ex:" >&2
    echo "    OPERATOR_IP=192.168.15.111 scripts/start_robot.sh" >&2
    echo "  IP actuelle du PC opérateur : \`ipconfig getifaddr en0\` sur le Mac." >&2
    exit 1
fi

mkdir -p logs
AGENT_LOG="logs/robot_agent.log"
CAMERA_LOG="logs/camera_pub.log"
ARM_LOG="logs/arm_agent.log"

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

# The arm needs the `lerobot` conda env's python instead (NOT .venv):
# RebotB601Follower is reused as-is from ~/03_JelloSoft/rebot_lerobot/lerobot,
# which only imports cleanly inside that env. Override with
# ARM_PYTHON=/path/to/python if the conda env lives somewhere else.
ARM_PY="${ARM_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python3}"
if [ ! -x "$ARM_PY" ]; then
    echo "start_robot.sh: python du bras introuvable at '$ARM_PY'." >&2
    echo "  Vérifie l'env conda 'lerobot' (voir ~/03_JelloSoft/rebot_lerobot/scripts/README.md)" >&2
    echo "  ou passe ARM_PYTHON=/chemin/vers/python explicitement." >&2
    exit 1
fi

# Best-effort graceful stop first: robot_agent.py's/arm_agent.py's
# SIGINT/KeyboardInterrupt handlers disable their motors before exiting, and
# camera_pub.py's releases the camera handle -- both matter more than just
# tidiness. Escalates to SIGKILL if a match is still alive after ~1.5s so
# this is idempotent either way (robot_agent.py's own control loop starts
# every wheel at commanded-zero regardless, per its stop_robot() fail-safe,
# and arm_agent.py sends no new targets without a fresh command either way,
# so a forced kill here is not itself unsafe -- just less clean).
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
stop_running "robot/arm_agent.py"

echo "start_robot.sh: OPERATOR_IP=$OPERATOR_IP -- logs dans logs/*.log" >&2

# -u (unbuffered): without it, stdout redirected to a file is block-buffered,
# so a crash right after startup can leave the log looking empty even though
# the process printed something first -- confirmed directly while debugging
# the incident above (had to kill and rerun in the foreground just to see
# what robot_agent.py had actually printed).
OPERATOR_IP="$OPERATOR_IP" "$PY" -u robot/robot_agent.py > "$AGENT_LOG" 2>&1 &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null || true' EXIT

# Give robot_agent.py a moment to either finish initializing or die (motor
# init + Zenoh open take well under this on a working setup). If it's gone,
# STOP HERE instead of falling through to the next component: that
# fallthrough is exactly what turned a robot_agent.py crash into an
# invisible failure last time -- camera_pub.py running fine made the
# operator UI look healthy ("Caméra: OK") while the base silently never had
# an agent behind it at all.
sleep 2
if ! kill -0 "$AGENT_PID" 2>/dev/null; then
    echo "start_robot.sh: robot_agent.py s'est arrêté immédiatement -- ABANDON." >&2
    echo "  --- $AGENT_LOG ---" >&2
    cat "$AGENT_LOG" >&2
    exit 1
fi
echo "start_robot.sh: robot_agent.py démarré (pid $AGENT_PID)." >&2

# camera_pub.py doesn't use Zenoh/OPERATOR_IP: it serves video straight to
# the browser over its own WebSocket server (ws://<robot-ip>:8765).
"$PY" -u robot/camera_pub.py > "$CAMERA_LOG" 2>&1 &
CAMERA_PID=$!
trap 'kill $AGENT_PID $CAMERA_PID 2>/dev/null || true' EXIT

sleep 1
if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    echo "start_robot.sh: camera_pub.py s'est arrêté immédiatement -- ABANDON." >&2
    echo "  --- $CAMERA_LOG ---" >&2
    cat "$CAMERA_LOG" >&2
    exit 1
fi
echo "start_robot.sh: camera_pub.py démarré (pid $CAMERA_PID)." >&2

echo "start_robot.sh: bras dégagé/soutenu ? connect() active le couple moteur immédiatement." >&2
OPERATOR_IP="$OPERATOR_IP" "$ARM_PY" -u robot/arm_agent.py > "$ARM_LOG" 2>&1 &
ARM_PID=$!
trap 'kill $AGENT_PID $CAMERA_PID $ARM_PID 2>/dev/null || true' EXIT

# Longer window than the other two: connect() does a real CAN handshake per
# joint (mode-ensure retries) plus a calibration-file check before it's done.
sleep 4
if ! kill -0 "$ARM_PID" 2>/dev/null; then
    echo "start_robot.sh: arm_agent.py s'est arrêté immédiatement -- ABANDON." >&2
    echo "  --- $ARM_LOG ---" >&2
    cat "$ARM_LOG" >&2
    exit 1
fi
echo "start_robot.sh: arm_agent.py démarré (pid $ARM_PID)." >&2

# Block here so Ctrl-C (SIGINT) reaches this script and runs the EXIT trap
# above, stopping all three children; also surfaces any one of them dying
# later (`wait -n` returns as soon as the first exits).
wait -n "$AGENT_PID" "$CAMERA_PID" "$ARM_PID"
echo "start_robot.sh: un des process s'est arrêté -- voir logs/*.log" >&2
exit 1
