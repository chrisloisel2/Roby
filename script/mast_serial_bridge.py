#!/usr/bin/env python3
"""
mast_serial_bridge.py — Pont bidirectionnel Arduino (chariot du mât) ↔ Zenoh.

Deux sens de communication :

  SÉRIE → ZENOH (télémétrie remontée par l'Arduino)
    - robot/mast/state  (JSON {"position_mm": <float>, "fdc_min": <bool>,
                               "fdc_max": <bool>, "t": <unix_ts>})
        fdc_min / fdc_max = fins de course bas / haut (true = déclenché).
        QoS "télémétrie" : DROP / BEST_EFFORT / DATA (cf. ZENOH.md §5)
    - robot/mast/link   (string "Connected" | "Disconnected")
        QoS "important, peu fréquent" : RELIABLE, publié uniquement au
        changement d'état (connexion série gagnée/perdue).
    - robot/mast/event  (string : lignes ACK/MSG/WARN/ERR du firmware)
        QoS RELIABLE : permet d'observer depuis le PC l'acquittement de
        chaque commande envoyée.

  ZENOH → SÉRIE (commandes envoyées par le PC opérateur)
    - robot/mast/cmd    (abonnement) : traduit chaque requête en une ligne
        du protocole série du firmware Moteur_pas_a_pas_v2 :
          H · POS:<mm> · STOP · UP_START/UP_STOP · DOWN_START/DOWN_STOP ·
          BRAKE:0/1 · FDC
        Le payload accepte DEUX formes (cf. §"PROTOCOLE COMMANDE" plus bas) :
          1. JSON structuré : {"action":"position","mm":342.5}
          2. Ligne série brute : "POS:342.5", "H", "STOP" ... (passthrough)

Ce script tourne sur la machine branchée en série à l'Arduino et se
connecte en mode client à un routeur zenohd, local ou distant, via
--connect (cf. ZENOH.md §2).

Usage :
    # routeur local (dev sur la même machine que l'Arduino)
    python3 mast_serial_bridge.py --port /dev/serial/by-id/usb-xxxx

    # routeur distant (ex: routeur sur le robot, script branché à l'Arduino)
    python3 mast_serial_bridge.py --port /dev/serial/by-id/usb-xxxx \
        --connect tcp/192.168.15.109:7447

Dépendances : pip install pyserial eclipse-zenoh

--------------------------------------------------------------------------
PROTOCOLE COMMANDE — clé robot/mast/cmd
--------------------------------------------------------------------------
Forme JSON (recommandée pour les applis) — champ "action" obligatoire :

  {"action": "position", "mm": 342.5}   → POS:342.5   (position absolue, mm)
  {"action": "velocity", "mm_s": 40}    → VEL:40.0    (mode vitesse, mm/s signé)
  {"action": "velocity", "mm_s": -25}   → VEL:-25.0   (>0 monte, <0 descend, 0 stop)
  {"action": "home"}                    → H           (homing complet)
  {"action": "stop"}                    → STOP        (ARRÊT D'URGENCE)
  {"action": "jog", "dir": "up",   "state": "start"}  → UP_START
  {"action": "jog", "dir": "up",   "state": "stop"}   → UP_STOP
  {"action": "jog", "dir": "down", "state": "start"}  → DOWN_START
  {"action": "jog", "dir": "down", "state": "stop"}   → DOWN_STOP
  {"action": "brake", "engaged": true}  → BRAKE:0     (frein SERRÉ)
  {"action": "brake", "engaged": false} → BRAKE:1     (frein LIBÉRÉ)
  {"action": "fdc"}                     → FDC         (test fins de course)
  {"action": "raw", "cmd": "POS:100"}   → POS:100     (passthrough explicite)

Alias tolérés : action "pos"/"move" = "position" ; "estop"/"emergency_stop"
= "stop" ; "homing" = "home" ; "test_fdc" = "fdc" ; champ "target_z" accepté
en plus de "mm".

Forme brute (pratique en CLI z_pub) : tout payload ne commençant pas par
'{' est envoyé tel quel comme une ligne série (ex: -p 'H', -p 'STOP',
-p 'POS:342.5').
"""

import argparse
import json
import re
import sys
import threading
import time

import serial
import zenoh

POS_RE = re.compile(r"POS:(-?\d+(?:\.\d+)?)")
MIN_RE = re.compile(r"MIN:([01])")
MAX_RE = re.compile(r"MAX:([01])")
EVENT_PREFIXES = ("ACK", "MSG:", "WARN:", "ERR:")

DEFAULT_KEY_STATE = "robot/mast/state"
DEFAULT_KEY_LINK = "robot/mast/link"
DEFAULT_KEY_EVENT = "robot/mast/event"
DEFAULT_KEY_CMD = "robot/mast/cmd"
DEFAULT_CONNECT = "tcp/localhost:7447"


# --------------------------------------------------------------------------
# Traduction requête Zenoh -> ligne série firmware. Fonction pure et testable.
# --------------------------------------------------------------------------
def translate_command(text: str) -> str:
    """Traduit un payload de commande en une ligne du protocole série.

    Lève ValueError si la requête est mal formée ou inconnue.
    Retourne la ligne série (sans le '\\n' final, ajouté à l'écriture).
    """
    text = text.strip()
    if not text:
        raise ValueError("payload vide")

    # Forme brute : passthrough d'une ligne série native.
    if not text.startswith("{"):
        return text.splitlines()[0].strip()

    try:
        obj = json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f"JSON invalide: {e}") from e
    if not isinstance(obj, dict):
        raise ValueError("le JSON doit être un objet")

    action = str(obj.get("action", "")).strip().lower()

    if action in ("position", "pos", "move"):
        if "mm" in obj:
            mm = obj["mm"]
        elif "target_z" in obj:
            mm = obj["target_z"]
        else:
            raise ValueError("position: champ 'mm' (ou 'target_z') requis")
        try:
            mm = float(mm)
        except (TypeError, ValueError) as e:
            raise ValueError(f"position: 'mm' non numérique ({mm!r})") from e
        return f"POS:{mm:.2f}"

    if action in ("velocity", "vel", "vitesse"):
        if "mm_s" in obj:
            v = obj["mm_s"]
        elif "v_z" in obj:
            v = obj["v_z"]
        elif "v" in obj:
            v = obj["v"]
        elif "speed" in obj:
            v = obj["speed"]
        else:
            raise ValueError("velocity: champ 'mm_s' (ou 'v_z'/'speed') requis")
        try:
            v = float(v)
        except (TypeError, ValueError) as e:
            raise ValueError(f"velocity: valeur non numérique ({v!r})") from e
        return f"VEL:{v:.1f}"

    if action in ("home", "homing"):
        return "H"

    if action in ("stop", "estop", "emergency_stop"):
        return "STOP"

    if action == "jog":
        d = str(obj.get("dir", "")).strip().lower()
        s = str(obj.get("state", "start")).strip().lower()
        if d not in ("up", "down"):
            raise ValueError("jog: 'dir' doit valoir 'up' ou 'down'")
        if s not in ("start", "stop"):
            raise ValueError("jog: 'state' doit valoir 'start' ou 'stop'")
        return f"{d.upper()}_{s.upper()}"

    if action == "brake":
        engaged = obj.get("engaged")
        if engaged is None:
            raise ValueError("brake: champ booléen 'engaged' requis")
        # engaged=True  -> frein serré  -> BRAKE:0
        # engaged=False -> frein libéré -> BRAKE:1
        return "BRAKE:0" if engaged else "BRAKE:1"

    if action in ("fdc", "test_fdc"):
        return "FDC"

    if action == "raw":
        cmd = str(obj.get("cmd", "")).strip()
        if not cmd:
            raise ValueError("raw: champ 'cmd' requis")
        return cmd.splitlines()[0].strip()

    raise ValueError(f"action inconnue: {action!r}")


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
        "--key-event",
        default=DEFAULT_KEY_EVENT,
        help=f"Clé Zenoh pour les acquittements ACK/MSG/WARN/ERR (défaut: {DEFAULT_KEY_EVENT})",
    )
    p.add_argument(
        "--key-cmd",
        default=DEFAULT_KEY_CMD,
        help=f"Clé Zenoh écoutée pour les commandes entrantes (défaut: {DEFAULT_KEY_CMD})",
    )
    p.add_argument(
        "--connect",
        default=DEFAULT_CONNECT,
        help=(
            "Endpoint du routeur zenohd à contacter, ex: tcp/192.168.15.109:7447 "
            f"(défaut: {DEFAULT_CONNECT}). Ignoré si --zenoh-config est fourni."
        ),
    )
    p.add_argument(
        "--zenoh-config",
        default=None,
        help=(
            "Fichier JSON5 de config Zenoh complet (ex: robot_onboard.json5), "
            "pour un contrôle plus fin qu'un simple --connect."
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
    """Construit la config Zenoh : mode client, connexion à l'endpoint --connect
    (routeur local ou distant, cf. ZENOH.md §2), multicast désactivé."""
    if args.zenoh_config:
        return zenoh.Config.from_file(args.zenoh_config)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", json.dumps([args.connect]))
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
        self.pub_event = session.declare_publisher(
            args.key_event,
            reliability=zenoh.Reliability.RELIABLE,
        )
        # Abonnement aux commandes entrantes. Le callback tourne sur un thread
        # interne de Zenoh : il doit rester court et non bloquant.
        self.sub_cmd = session.declare_subscriber(args.key_cmd, self.on_command)

        self.connected = None  # None = pas encore connu -> force la 1re publication
        self.ser = None
        # Protège l'accès concurrent au handle série : le thread principal lit,
        # le thread du callback Zenoh écrit ; open/close ré-assigne self.ser.
        self.ser_lock = threading.Lock()

    # ---- Sens Zenoh -> série (commandes) --------------------------------
    def on_command(self, sample):
        """Callback subscriber : traduit la requête et l'écrit sur le port série."""
        try:
            text = sample.payload.to_bytes().decode("ascii", errors="replace")
        except Exception as e:  # payload illisible
            print(f"[cmd] payload illisible ({e})", file=sys.stderr)
            return
        try:
            line = translate_command(text)
        except ValueError as e:
            print(f"[cmd] requête refusée: {e} | reçu={text.strip()!r}", file=sys.stderr)
            self.pub_event.put(f"MSG:REFUS_BRIDGE,{e}")
            return
        if self.write_command(line):
            print(f"[cmd] {text.strip()!r} -> série {line!r}", file=sys.stderr)

    def write_command(self, line: str) -> bool:
        """Écrit une ligne série de façon thread-safe. Retourne True si envoyée."""
        data = (line + "\n").encode("ascii", errors="ignore")
        with self.ser_lock:
            ser = self.ser
            if ser is None:
                print(f"[cmd] {line!r} ignorée : série non connectée", file=sys.stderr)
                self.pub_event.put("MSG:REFUS_BRIDGE,SERIE_DECONNECTEE")
                return False
            try:
                ser.write(data)
                ser.flush()
            except (serial.SerialException, OSError) as e:
                print(f"[cmd] échec écriture {line!r} ({e})", file=sys.stderr)
                return False
        return True

    # ---- Sens série -> Zenoh (télémétrie + acquittements) ---------------
    def set_connected(self, state: bool):
        if state == self.connected:
            return
        self.connected = state
        msg = "Connected" if state else "Disconnected"
        self.pub_link.put(msg)
        print(f"[link] {msg}", file=sys.stderr)

    def route_line(self, line: str):
        """Aiguille une ligne reçue de l'Arduino selon son préfixe."""
        if line.startswith("POS:"):
            m = POS_RE.search(line)
            if m:
                mn = MIN_RE.search(line)
                mx = MAX_RE.search(line)
                payload = json.dumps({
                    "position_mm": float(m.group(1)),
                    "fdc_min": bool(int(mn.group(1))) if mn else False,
                    "fdc_max": bool(int(mx.group(1))) if mx else False,
                    "t": time.time(),
                })
                self.pub_state.put(payload)
        elif line.startswith(EVENT_PREFIXES):
            self.pub_event.put(line)

    def open_serial(self):
        ser = serial.Serial(self.args.port, self.args.baud, timeout=self.args.read_timeout)
        with self.ser_lock:
            self.ser = ser
        self.set_connected(True)

    def close_serial(self):
        with self.ser_lock:
            ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
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
                                self.route_line(line)
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
        print(
            f"[bridge] écoute commandes sur {args.key_cmd!r} | "
            f"télémétrie sur {args.key_state!r} / {args.key_event!r}",
            file=sys.stderr,
        )
        MastBridge(args, session).run()


if __name__ == "__main__":
    main()