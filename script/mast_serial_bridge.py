#!/usr/bin/env python3
"""
mast_serial_bridge.py — Pont série Arduino (chariot du mât) → Zenoh.

Lit la télémétrie série de l'Arduino (firmware Moteur_pas_a_pas_v2, trames
"POS:xx.x,CNT:...,MIN:...,MAX:...,BRAKE:...,HOMED:...") et publie sur Zenoh,
selon le plan de clés de ZENOH.md :

  - robot/mast/state  (JSON {"position_mm": <float>, "t": <unix_ts>})
      QoS "télémétrie" : DROP / BEST_EFFORT / DATA (cf. ZENOH.md §5)
  - robot/mast/link   (string "Connected" | "Disconnected")
      QoS "important, peu fréquent" : RELIABLE, publié uniquement au
      changement d'état (connexion série gagnée/perdue).

Ce script tourne "onboard" (sur le PC actuellement branché en série à
l'Arduino) et se connecte au routeur zenohd local (tcp/localhost:7447),
cf. ZENOH.md §2 et §8 (robot_onboard.json5).

Usage :
    python3 mast_serial_bridge.py --port /dev/serial/by-id/usb-xxxx

Dépendances : pip install pyserial eclipse-zenoh
"""

import argparse
import json
import re
import sys
import time

import serial
import zenoh

POS_RE = re.compile(r"POS:(-?\d+(?:\.\d+)?)")

DEFAULT_KEY_STATE = "robot/mast/state"
DEFAULT_KEY_LINK = "robot/mast/link"


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--port",
        required=True,
        help="Port série de l'Arduino (idéalement /dev/serial/by-id/... ou un symlink udev fixe)",
    )
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument(
        "--key-state",
        default=DEFAULT_KEY_STATE,
        help=f"Clé Zenoh pour la position (défaut: {DEFAULT_KEY_STATE})",
    )
    p.add_argument(
        "--key-link",
        default=DEFAULT_KEY_LINK,
        help=f"Clé Zenoh pour l'état de connexion série (défaut: {DEFAULT_KEY_LINK})",
    )
    p.add_argument(
        "--zenoh-config",
        default=None,
        help=(
            "Fichier JSON5 de config Zenoh (ex: robot_onboard.json5). "
            "Sinon, config par défaut : mode client, tcp/localhost:7447, multicast off."
        ),
    )
    p.add_argument(
        "--reconnect-delay",
        type=float,
        default=2.0,
        help="Délai entre tentatives de reconnexion série (s)",
    )
    p.add_argument(
        "--read-timeout", type=float, default=1.0, help="Timeout de lecture série (s)"
    )
    p.add_argument(
        "--silence-timeout",
        type=float,
        default=3.0,
        help="Durée sans trame reçue avant de déclarer Disconnected (s)",
    )
    return p.parse_args()


def build_zenoh_config(args):
    """Construit la config Zenoh. Suit robot_onboard.json5 (ZENOH.md §8) par défaut :
    mode client, connexion au routeur local, multicast désactivé."""
    if args.zenoh_config:
        return zenoh.Config.from_file(args.zenoh_config)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    conf.insert_json5("scouting/multicast/enabled", "false")
    return conf


class MastBridge:
    def __init__(self, args, session):
        self.args = args
        self.session = session
        self.pub_state = session.declare_publisher(
            args.key_state,
            congestion_control=zenoh.CongestionControl.DROP,
            reliability=zenoh.Reliability.BEST_EFFORT,
            priority=zenoh.Priority.DATA,
        )
        self.pub_link = session.declare_publisher(
            args.key_link,
            reliability=zenoh.Reliability.RELIABLE,
        )
        self.connected = None  # None = pas encore connu -> force la 1re publication
        self.ser = None

    def set_connected(self, state: bool):
        if state == self.connected:
            return
        self.connected = state
        msg = "Connected" if state else "Disconnected"
        self.pub_link.put(msg)
        print(f"[link] {msg}", file=sys.stderr)

    def publish_position(self, line: str):
        m = POS_RE.search(line)
        if not m:
            return
        payload = json.dumps({"position_mm": float(m.group(1)), "t": time.time()})
        self.pub_state.put(payload)

    def open_serial(self):
        self.ser = serial.Serial(self.args.port, self.args.baud, timeout=self.args.read_timeout)
        self.set_connected(True)

    def close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.set_connected(False)

    def run(self):
        self.set_connected(False)  # état initial explicite tant que le port n'est pas ouvert
        try:
            while True:
                if self.ser is None:
                    try:
                        self.open_serial()
                    except (serial.SerialException, FileNotFoundError, OSError) as e:
                        print(
                            f"[link] ouverture {self.args.port} impossible ({e}), retry...",
                            file=sys.stderr,
                        )
                        time.sleep(self.args.reconnect_delay)
                        continue

                last_rx = time.monotonic()
                try:
                    while True:
                        raw = self.ser.readline()
                        now = time.monotonic()
                        if raw:
                            last_rx = now
                            line = raw.decode("ascii", errors="replace").strip()
                            if line:
                                self.publish_position(line)
                        elif now - last_rx > self.args.silence_timeout:
                            raise serial.SerialException("aucune trame reçue (timeout silence)")
                except (serial.SerialException, OSError) as e:
                    print(f"[link] port {self.args.port} perdu ({e})", file=sys.stderr)
                    self.close_serial()
                    time.sleep(self.args.reconnect_delay)
        except KeyboardInterrupt:
            pass
        finally:
            self.close_serial()


def main():
    args = parse_args()
    conf = build_zenoh_config(args)
    with zenoh.open(conf) as session:
        MastBridge(args, session).run()


if __name__ == "__main__":
    main()