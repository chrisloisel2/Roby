#!/usr/bin/env bash
# Operator PC: start the Zenoh router, web server, and input agent.
# Run from the repo root. Ctrl-C stops everything.
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

python operator/input_agent.py
