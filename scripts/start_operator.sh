#!/usr/bin/env bash
# Operator PC: start the Zenoh router, web server, and input agent.
# Run from the repo root. Ctrl-C stops everything.
#
# Set ROBOT_IP (the robot PC's address, e.g. 192.168.15.107) if GELLO_PORT
# is also set: input_agent.py needs it to reach robot/arm_agent.py's own
# WebSocket directly (not Zenoh-routed, see input_agent.py's ArmLink). Not
# required for joystick-only base control with no GELLO plugged in.
#
# Idempotent: kills any already-running instance of each component before
# starting fresh. This matters beyond tidiness -- a leftover zenohd fails
# the new one with "Address already in use", and a leftover input_agent.py
# (or a browser tab with "Piloter depuis ce navigateur" left on) publishes
# to the exact same robot/cmd/base + operator/deadman topics as the new one:
# two sources racing means deadman flips true/false on every message, so the
# robot reads deadman_ok=false most of the time and silently never moves,
# even with someone correctly holding the physical deadman button.
set -euo pipefail
cd "$(dirname "$0")/.."

# Best-effort graceful stop (lets input_agent.py publish a final zero/deadman
# -off via its SIGINT/KeyboardInterrupt handler), escalating to SIGKILL if a
# match is still alive after ~1.5s -- guarantees idempotency either way.
stop_running() {
    local sig="$1" pattern="$2" pids
    pids=$(pgrep -f "$pattern" || true)
    [ -z "$pids" ] && return 0
    echo "start_operator.sh: stopping existing '$pattern' ($pids)" >&2
    kill "$sig" $pids 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        pgrep -f "$pattern" >/dev/null || return 0
        sleep 0.3
    done
    kill -9 $pids 2>/dev/null || true
    sleep 0.3
}

stop_running -TERM "zenohd -c config/router.json5"
stop_running -TERM "operator/web_server.py"
stop_running -INT  "operator/input_agent.py"

zenohd -c config/router.json5 &
ROUTER_PID=$!
trap 'kill $ROUTER_PID 2>/dev/null || true' EXIT

sleep 1
python operator/web_server.py &
WEB_PID=$!
trap 'kill $ROUTER_PID $WEB_PID 2>/dev/null || true' EXIT

# input_agent.py is optional, not core: the browser UI (Gamepad API + Web
# Serial) can drive both the base and the GELLO on its own now, and
# input_agent.py itself exits cleanly (not an error) when no joystick is
# plugged in -- see its main(). Backgrounded and left OUT of the `wait`
# below on purpose, so zenohd/web_server keep serving even if this exits
# early; still added to the trap so Ctrl-C (or this script exiting for any
# other reason) takes it down along with everything else.
python operator/input_agent.py &
INPUT_PID=$!
trap 'kill $ROUTER_PID $WEB_PID $INPUT_PID 2>/dev/null || true' EXIT

# Block on the two core services only -- Ctrl-C (SIGINT) interrupts this
# `wait` immediately and runs the EXIT trap above.
wait "$ROUTER_PID" "$WEB_PID"
