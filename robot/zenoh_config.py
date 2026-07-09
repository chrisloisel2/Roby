"""Shared Zenoh client config loader for the robot-PC processes
(robot_agent.py, arm_agent.py). camera_pub.py does NOT use Zenoh -- it
serves video straight to the browser over its own WebSocket server instead
(ws://<robot-ip>:8765, see camera_pub.py's module docstring).

Single source of truth for "which operator IP do we connect to". This used
to be reimplemented three times, out of sync: camera_pub.py had its own
hardcoded 169.254.140.115 fallback (unrelated to anything else in the repo),
robot_agent.py/arm_agent.py silently fell back to config/robot_zenoh.json5's
own default (192.168.15.106) whenever OPERATOR_IP wasn't set -- and that
default went stale the moment the operator Mac's DHCP lease changed to
192.168.15.111. Nothing printed which endpoint was actually in use.

Result (2026-07-08): robot_agent.py silently connected nowhere while
camera_pub.py, started from a shell that happened to export the correct
OPERATOR_IP directly, kept streaming video fine -- from the operator UI it
looked like "the robot is fine, only the motors are broken", when actually
robot_agent.py's Zenoh session never routed anywhere.

OPERATOR_IP is now REQUIRED, not an optional override with a silent
fallback -- a wrong-but-present default is worse than refusing to start,
because a wrong default still "works" (opens a client, publishes into the
void) instead of failing loudly where the operator will actually see it.
"""
import json
import os
import sys
from pathlib import Path

import zenoh

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "robot_zenoh.json5"


def load_robot_config(component: str) -> zenoh.Config:
    """Build the Zenoh client config for a robot-PC process.

    `component` is just the caller's name (e.g. "robot_agent"), used to make
    the required-env-var error and the connection confirmation identifiable
    when several of these processes share one terminal/log directory.
    """
    operator_ip = os.environ.get("OPERATOR_IP")
    if not operator_ip:
        sys.exit(
            f"{component}: la variable d'environnement OPERATOR_IP est requise "
            f"(ex: OPERATOR_IP=192.168.15.111 python3 robot/{component}.py) -- "
            f"pas de valeur par défaut, pour ne plus jamais se connecter en "
            f"silence à une IP obsolète. IP opérateur actuelle : "
            f"`ipconfig getifaddr en0` sur le Mac (LAN) ou `tailscale ip -4` "
            f"(Tailscale)."
        )
    config = zenoh.Config.from_file(str(CONFIG_PATH))
    config.insert_json5("connect/endpoints", json.dumps([f"tcp/{operator_ip}:7447"]))
    # Unbuffered-safe (flush=True): this must reach the log even if stdout is
    # redirected to a file and the process dies moments later, before Python's
    # block-buffering would otherwise have flushed it on its own.
    print(f"{component}: connexion Zenoh -> tcp/{operator_ip}:7447", flush=True)
    return config
