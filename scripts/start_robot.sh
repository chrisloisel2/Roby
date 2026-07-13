#!/usr/bin/env bash
# Robot PC: start the robot agent, camera publisher, and arm agent -- the
# full robot-side stack in one command.
# Set OPERATOR_IP to the operator PC address, e.g.:
#     OPERATOR_IP=192.168.15.111 scripts/start_robot.sh
#
# Opt-out flags, for when a CAN adapter is unplugged/misbehaving and you
# still want the rest of the stack up rather than being blocked entirely by
# the fail-fast checks below:
#     NO_ARM=1      OPERATOR_IP=192.168.15.111 scripts/start_robot.sh   # base + caméras + mât, pas de bras
#     NO_BASE=1     OPERATOR_IP=192.168.15.111 scripts/start_robot.sh   # bras + caméras + mât, pas de base (ex: base CAN en panne, tu veux quand même tester le bras)
#     NO_MAST=1     OPERATOR_IP=192.168.15.111 scripts/start_robot.sh   # base + bras + caméras, pas de mât (ex: bridge Arduino du mât débranché)
#     CAMERA_ONLY=1 scripts/start_robot.sh                              # caméras seules (pas d'OPERATOR_IP requis : camera_pub.py ne parle pas Zenoh)
#
# camera_pub.py always runs regardless of these flags, same as before --
# it now serves BOTH the front camera and an optional second UVC camera
# over the SAME WebSocket connection/port (8765), not two separate
# processes/ports -- see robot/camera_pub.py and robot/uvc_camera_server.py
# for why (both cameras get identical connection-level treatment this way).
#
# Idempotent: kills any already-running instance of each component first, so
# re-running this script never leaves duplicates racing each other over CAN
# commands or the camera device.
#
# OPERATOR_IP is REQUIRED whenever base or arm run (robot/zenoh_config.py
# enforces it too, but failing here is friendlier -- before touching any
# hardware). See that module's docstring for the incident this whole script
# hardening replaced (2026-07-08): a stale hardcoded default IP made
# robot_agent.py silently connect nowhere, while robot_agent.py's own crash
# was invisible (no log, backgrounded with no liveness check) and
# camera_pub.py kept working fine in the same script run -- from the
# operator UI it looked like "only the motors are broken" for a problem that
# was actually "robot_agent.py never came up at all".
#
# The arm agent (robot/arm_agent.py) used to be its own separate script
# (start_arm.sh) precisely because the arm path was brand new and unproven
# -- see that script's history. It has since had a real bring-up (2026-07-08:
# a read-only motorbridge probe got a live state reply from all 7 joints
# over its corrected ARM_PORT), so it's folded in here as a third required
# component (unless NO_ARM/CAMERA_ONLY opt out of it), same fail-fast
# treatment as the other two. start_arm.sh still exists for isolated
# arm-only testing.
set -euo pipefail
cd "$(dirname "$0")/.."

CAMERA_ONLY="${CAMERA_ONLY:-0}"
NO_ARM="${NO_ARM:-0}"
NO_BASE="${NO_BASE:-0}"
NO_MAST="${NO_MAST:-0}"
RUN_BASE=1
RUN_ARM=1
RUN_MAST=1
if [ "$CAMERA_ONLY" = "1" ]; then
    RUN_BASE=0
    RUN_ARM=0
    RUN_MAST=0
fi
if [ "$NO_ARM" = "1" ]; then
    RUN_ARM=0
fi
if [ "$NO_BASE" = "1" ]; then
    RUN_BASE=0
fi
if [ "$NO_MAST" = "1" ]; then
    RUN_MAST=0
fi

if { [ "$RUN_BASE" = "1" ] || [ "$RUN_ARM" = "1" ] || [ "$RUN_MAST" = "1" ]; } && [ -z "${OPERATOR_IP:-}" ]; then
    echo "start_robot.sh: OPERATOR_IP est requis (base et/ou bras activés), ex:" >&2
    echo "    OPERATOR_IP=192.168.15.111 scripts/start_robot.sh" >&2
    echo "  IP actuelle du PC opérateur : \`ipconfig getifaddr en0\` sur le Mac." >&2
    echo "  Ou CAMERA_ONLY=1 scripts/start_robot.sh pour la caméra seule (pas d'OPERATOR_IP requis)." >&2
    exit 1
fi

mkdir -p logs
AGENT_LOG="logs/robot_agent.log"
CAMERA_LOG="logs/camera_pub.log"
ARM_LOG="logs/arm_agent.log"
MAST_LOG="logs/mast_serial_bridge.log"

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
if [ "$RUN_ARM" = "1" ]; then
    ARM_PY="${ARM_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python3}"
    if [ ! -x "$ARM_PY" ]; then
        echo "start_robot.sh: python du bras introuvable at '$ARM_PY'." >&2
        echo "  Vérifie l'env conda 'lerobot' (voir ~/03_JelloSoft/rebot_lerobot/scripts/README.md)" >&2
        echo "  ou passe ARM_PYTHON=/chemin/vers/python explicitement." >&2
        echo "  ou passe NO_ARM=1 / CAMERA_ONLY=1 pour sauter le bras." >&2
        exit 1
    fi
fi

# Best-effort graceful stop first: robot_agent.py's/arm_agent.py's
# SIGINT/KeyboardInterrupt handlers disable their motors before exiting, and
# camera_pub.py's releases the camera handle -- both matter more than just
# tidiness. Escalates to SIGKILL if a match is still alive after ~1.5s
# (robot_agent.py's own control loop starts every wheel at commanded-zero
# regardless, per its stop_robot() fail-safe, and arm_agent.py sends no new
# targets without a fresh command either way, so a forced kill here is not
# itself unsafe -- just less clean). Always tried for all four regardless of
# which flags this run uses, so a leftover process from a previous full run
# doesn't keep racing this one.
#
# Critically: SIGKILL is NOT verified to actually work by itself -- a
# process blocked in a kernel-side blocking syscall (a CAN read, the mast's
# serial read, a wedged V4L2/USB camera read) stays alive in uninterruptible
# sleep (state D) until that syscall returns, no matter the signal. Blindly
# continuing past that (the old behavior here: sleep a fixed 0.3s and move
# on regardless) let a genuinely-stuck old process keep holding the exact
# CAN bus / serial port / camera device the new instance is about to open,
# so the new one either fails its fail-fast startup check for a confusing
# reason ("port busy", not "didn't start") or -- worse -- both instances end
# up alive and racing each other over the same hardware. So: actually
# reconfirm the pattern is gone after SIGKILL, and if it still isn't, abort
# loudly instead of starting a second instance on top of a stuck one.
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
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        pgrep -f "$pattern" >/dev/null || return 0
        sleep 0.3
    done
    echo "start_robot.sh: '$pattern' ($pids) toujours vivant ~4.5s après kill -9 -- ABANDON." >&2
    echo "  probablement bloqué en E/S noyau (CAN / série mât / device caméra) plutôt que" >&2
    echo "  simplement lent à mourir -- démarrer une nouvelle instance par-dessus se" >&2
    echo "  terminerait en conflit sur le même device, pas en propreté. État du process :" >&2
    local p
    for p in $pids; do ps -o pid,stat,cmd -p "$p" 2>/dev/null >&2 || true; done
    echo "  Débloque/débranche-rebranche le matériel concerné, ou 'kill -9 $pids' à la main" >&2
    echo "  puis relance." >&2
    exit 1
}

stop_running "robot/robot_agent.py"
stop_running "robot/camera_pub.py"
stop_running "robot/arm_agent.py"
stop_running "robot/mast_serial_bridge.py"

echo "start_robot.sh: RUN_BASE=$RUN_BASE RUN_ARM=$RUN_ARM RUN_MAST=$RUN_MAST -- logs dans logs/*.log" >&2

# PIDs of whatever this run actually starts, so the EXIT trap and the final
# wait only ever reference processes that exist -- no matter which subset of
# base/camera/arm this run launched.
PIDS=()
cleanup() {
    local pid
    # -INT, not a plain kill (SIGTERM): robot_agent.py/arm_agent.py only
    # disable torque and disconnect cleanly from their KeyboardInterrupt
    # handler, which Python raises for SIGINT, never for SIGTERM. A plain
    # `kill` here was silently skipping that entirely -- the arm stayed
    # torqued/rigid after Ctrl-C because the finally: block in
    # start_teleoperationV2.py's run_teleoperation() (robot.disconnect(),
    # which disables torque per RebotB601Follower's
    # disable_torque_on_disconnect=True) never got to run.
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

if [ "$RUN_BASE" = "1" ]; then
    # -u (unbuffered): without it, stdout redirected to a file is
    # block-buffered, so a crash right after startup can leave the log
    # looking empty even though the process printed something first --
    # confirmed directly while debugging the incident above (had to kill
    # and rerun in the foreground just to see what robot_agent.py had
    # actually printed).
    OPERATOR_IP="$OPERATOR_IP" "$PY" -u robot/robot_agent.py > "$AGENT_LOG" 2>&1 &
    AGENT_PID=$!
    PIDS+=("$AGENT_PID")

    # Give robot_agent.py a moment to either finish initializing or die
    # (motor init + Zenoh open take well under this on a working setup). If
    # it's gone, STOP HERE instead of falling through to the next
    # component: that fallthrough is exactly what turned a robot_agent.py
    # crash into an invisible failure last time -- camera_pub.py running
    # fine made the operator UI look healthy ("Caméra: OK") while the base
    # silently never had an agent behind it at all.
    sleep 2
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        echo "start_robot.sh: robot_agent.py s'est arrêté immédiatement -- ABANDON." >&2
        echo "  --- $AGENT_LOG ---" >&2
        cat "$AGENT_LOG" >&2
        echo "  (NO_BASE=1 pour continuer sans la base -- ex: tester le bras seul pendant que son CAN est en panne)" >&2
        exit 1
    fi
    echo "start_robot.sh: robot_agent.py démarré (pid $AGENT_PID)." >&2
else
    echo "start_robot.sh: base sautée (RUN_BASE=0)." >&2
fi

# camera_pub.py doesn't use Zenoh/OPERATOR_IP: it serves video straight to
# the browser over its own WebSocket server (ws://<robot-ip>:8765). Always
# started -- it's the one component every mode (including CAMERA_ONLY) runs.
"$PY" -u robot/camera_pub.py > "$CAMERA_LOG" 2>&1 &
CAMERA_PID=$!
PIDS+=("$CAMERA_PID")

sleep 1
if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    echo "start_robot.sh: camera_pub.py s'est arrêté immédiatement -- ABANDON." >&2
    echo "  --- $CAMERA_LOG ---" >&2
    cat "$CAMERA_LOG" >&2
    exit 1
fi
echo "start_robot.sh: camera_pub.py démarré (pid $CAMERA_PID)." >&2

if [ "$RUN_ARM" = "1" ]; then
    echo "start_robot.sh: bras dégagé/soutenu ? connect() active le couple moteur immédiatement." >&2
    OPERATOR_IP="$OPERATOR_IP" "$ARM_PY" -u robot/arm_agent.py > "$ARM_LOG" 2>&1 &
    ARM_PID=$!
    PIDS+=("$ARM_PID")

    # Longer window than the other two: connect() does a real CAN handshake
    # per joint (mode-ensure retries) plus a calibration-file check before
    # it's done.
    sleep 4
    if ! kill -0 "$ARM_PID" 2>/dev/null; then
        echo "start_robot.sh: arm_agent.py s'est arrêté immédiatement -- ABANDON." >&2
        echo "  --- $ARM_LOG ---" >&2
        cat "$ARM_LOG" >&2
        echo "  (NO_ARM=1 ou CAMERA_ONLY=1 pour continuer sans le bras)" >&2
        exit 1
    fi
    echo "start_robot.sh: arm_agent.py démarré (pid $ARM_PID)." >&2
else
    echo "start_robot.sh: bras sauté (RUN_ARM=0)." >&2
fi

if [ "$RUN_MAST" = "1" ]; then
    # mast_serial_bridge.py only needs pyserial + eclipse-zenoh (already in
    # the shared .venv, see requirements.txt) -- runs under $PY, no separate
    # conda env like the arm. It retries the serial port internally (no
    # crash if the Arduino isn't plugged in yet), so this fail-fast check
    # only really catches Zenoh/config failures, not missing hardware.
    OPERATOR_IP="$OPERATOR_IP" "$PY" -u robot/mast_serial_bridge.py > "$MAST_LOG" 2>&1 &
    MAST_PID=$!
    PIDS+=("$MAST_PID")

    sleep 2
    if ! kill -0 "$MAST_PID" 2>/dev/null; then
        echo "start_robot.sh: mast_serial_bridge.py s'est arrêté immédiatement -- ABANDON." >&2
        echo "  --- $MAST_LOG ---" >&2
        cat "$MAST_LOG" >&2
        echo "  (NO_MAST=1 pour continuer sans le mât)" >&2
        exit 1
    fi
    echo "start_robot.sh: mast_serial_bridge.py démarré (pid $MAST_PID)." >&2
else
    echo "start_robot.sh: mât sauté (RUN_MAST=0)." >&2
fi

# Block here so Ctrl-C (SIGINT) reaches this script and runs the EXIT trap
# above, stopping every process this run started; also surfaces any one of
# them dying later (`wait -n` returns as soon as the first exits).
wait -n "${PIDS[@]}"
echo "start_robot.sh: un des process s'est arrêté -- voir logs/*.log" >&2
exit 1
