#!/usr/bin/env bash
# Operator PC: start the Zenoh router, web server, and input agent.
# Run from the repo root. Ctrl-C stops everything.
set -euo pipefail
cd "$(dirname "$0")/.."

zenohd -c config/router.json5 &
ROUTER_PID=$!
trap 'kill $ROUTER_PID 2>/dev/null || true' EXIT

sleep 1
python operator/web_server.py &
WEB_PID=$!
trap 'kill $ROUTER_PID $WEB_PID 2>/dev/null || true' EXIT

python operator/input_agent.py
