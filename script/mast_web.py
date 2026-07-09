#!/usr/bin/env python3
"""
mast_web.py — Serveur de test web pour le chariot du mât.

Sert une petite interface HTML de test (barre de contrôle 0→max, bouton de
homing, bouton d'arrêt d'urgence) et fait le pont HTTP/SSE ↔ Zenoh, pour que
le navigateur n'ait PAS besoin de parler Zenoh. Complémentaire (et autonome
vis-à-vis) de mast_serial_bridge.py :

    Navigateur ──HTTP/SSE──▶ mast_web.py ──Zenoh──▶ mast_serial_bridge.py ──série──▶ Arduino

Sens navigateur → mât :
    POST /cmd   (corps = payload de commande)  →  publié TEL QUEL sur robot/mast/cmd
        Le payload accepte les deux mêmes formes que le bridge : JSON
        {"action":"position","mm":342.5} ou ligne série brute "POS:342.5".
        La traduction en protocole série reste faite par le bridge : ce
        serveur ne duplique aucune logique métier.

Sens mât → navigateur (SSE, GET /events) :
    - robot/mast/state  →  event SSE "state"  (JSON position_mm/fdc_min/fdc_max)
    - robot/mast/event  →  event SSE "event"  (ACK/MSG/WARN/ERR — dont COURSE)
    - robot/mast/link   →  event SSE "link"   ("Connected"/"Disconnected")

Le serveur mémorise le dernier COURSE (max) annoncé par le homing et le
rejoue à chaque nouveau client : l'interface se recalibre seule au
rechargement, tant que le mât reste homé et que ce serveur tourne.

Dépendances : pip install eclipse-zenoh   (le reste est dans la stdlib)

Usage :
    # routeur local (même machine que le bridge)
    python3 mast_web.py

    # routeur distant
    python3 mast_web.py --connect tcp/192.168.15.109:7447

    # puis ouvrir http://localhost:8080
    # (--host 0.0.0.0 pour y accéder depuis une tablette/un autre PC)
"""

import argparse
import json
import os
import queue
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import zenoh

DEFAULT_KEY_STATE = "robot/mast/state"
DEFAULT_KEY_LINK = "robot/mast/link"
DEFAULT_KEY_EVENT = "robot/mast/event"
DEFAULT_KEY_CMD = "robot/mast/cmd"
DEFAULT_CONNECT = "tcp/localhost:7447"
DEFAULT_UI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mast_ui.html")

# Préfixes d'événements firmware qui invalident la calibration (le mât n'est
# plus/pas homé → on cesse de rejouer un COURSE périmé aux nouveaux clients).
DECALIBRATE_PREFIXES = ("MSG:BOOT", "ERR:HOMING", "ERR:ENC_OR_STALL")


def _decode(sample) -> str:
    return sample.payload.to_bytes().decode("utf-8", errors="replace").strip()


# --------------------------------------------------------------------------
# Diffusion SSE : un Queue par client connecté, alimenté par les callbacks
# Zenoh (qui tournent sur un thread interne de Zenoh, donc rester non bloquant).
# --------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self._clients = set()
        self._lock = threading.Lock()

    def register(self) -> "queue.Queue":
        q = queue.Queue(maxsize=200)
        with self._lock:
            self._clients.add(q)
        return q

    def unregister(self, q):
        with self._lock:
            self._clients.discard(q)

    def broadcast(self, kind: str, data: str):
        item = (kind, data)
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(item)
            except queue.Full:
                # Client trop lent : on sacrifie le message le plus ancien.
                try:
                    q.get_nowait()
                    q.put_nowait(item)
                except queue.Empty:
                    pass


# --------------------------------------------------------------------------
# Dernier état connu, rejoué à chaque nouveau client pour hydrater l'UI
# immédiatement (position, lien série, et COURSE si toujours calibré).
# --------------------------------------------------------------------------
class Cache:
    def __init__(self):
        self._lock = threading.Lock()
        self.last_state = None
        self.last_link = None
        self.course_line = None
        self.calibrated = False

    def note_state(self, data: str):
        with self._lock:
            self.last_state = data

    def note_link(self, data: str):
        with self._lock:
            self.last_link = data

    def note_event(self, line: str):
        with self._lock:
            if line.startswith("MSG:HOMING_OK"):
                self.course_line = line
                self.calibrated = True
            elif line.startswith(DECALIBRATE_PREFIXES):
                self.course_line = None
                self.calibrated = False

    def snapshot(self):
        with self._lock:
            out = []
            if self.last_link is not None:
                out.append(("link", self.last_link))
            if self.calibrated and self.course_line:
                out.append(("event", self.course_line))
            if self.last_state is not None:
                out.append(("state", self.last_state))
            return out


class WebServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, *, ui_path, pub_cmd, hub, cache):
        super().__init__(addr, handler)
        self.ui_path = ui_path
        self.pub_cmd = pub_cmd
        self.hub = hub
        self.cache = cache


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # silence des logs d'accès (SSE = bruyant)
        pass

    # ---- GET : UI + flux SSE --------------------------------------------
    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._serve_ui()
        elif path == "/events":
            self._serve_events()
        else:
            self._send_text(404, "not found")

    def _serve_ui(self):
        try:
            with open(self.server.ui_path, "rb") as f:
                body = f.read()
        except OSError as e:
            self._send_text(500, f"UI introuvable ({self.server.ui_path}): {e}")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")  # flux non borné : délimité par la fermeture
        self.end_headers()
        self.close_connection = True

        q = self.server.hub.register()
        try:
            # Hydratation immédiate du nouveau client depuis le cache.
            for kind, data in self.server.cache.snapshot():
                self._sse(kind, data)
            while True:
                try:
                    kind, data = q.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")  # heartbeat anti-timeout
                    self.wfile.flush()
                    continue
                self._sse(kind, data)
        except (BrokenPipeError, ConnectionResetError, OSError, ValueError):
            pass  # client parti
        finally:
            self.server.hub.unregister(q)

    def _sse(self, event: str, data: str):
        out = [f"event: {event}\n"]
        for line in (data.split("\n") or [""]):
            out.append(f"data: {line}\n")
        out.append("\n")
        self.wfile.write("".join(out).encode("utf-8"))
        self.wfile.flush()

    # ---- POST : commandes vers le mât -----------------------------------
    def do_POST(self):
        path = urlparse(self.path).path
        if path != "/cmd":
            self._send_text(404, "not found")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        body = self.rfile.read(length) if length else b""
        line = body.decode("utf-8", errors="replace").strip()
        if not line:
            self._send_text(400, "payload vide")
            return
        # On publie la ligne telle quelle : le bridge traduit (JSON ou brut).
        try:
            self.server.pub_cmd.put(line)
        except Exception as e:  # session Zenoh en vrac
            self._send_text(502, f"publication Zenoh impossible: {e}")
            return
        print(f"[cmd] -> {line!r}", file=sys.stderr)
        self._send_text(200, "ok")

    def _send_text(self, code: int, msg: str):
        body = msg.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--host", default="127.0.0.1",
                   help="Interface d'écoute HTTP (défaut: 127.0.0.1 ; 0.0.0.0 pour exposer)")
    p.add_argument("--http-port", type=int, default=8080, help="Port HTTP (défaut: 8080)")
    p.add_argument("--ui", default=DEFAULT_UI, help=f"Chemin du fichier HTML (défaut: {DEFAULT_UI})")
    p.add_argument("--key-state", default=DEFAULT_KEY_STATE)
    p.add_argument("--key-link", default=DEFAULT_KEY_LINK)
    p.add_argument("--key-event", default=DEFAULT_KEY_EVENT)
    p.add_argument("--key-cmd", default=DEFAULT_KEY_CMD)
    p.add_argument("--connect", default=DEFAULT_CONNECT,
                   help=f"Endpoint du routeur zenohd (défaut: {DEFAULT_CONNECT}). "
                        "Ignoré si --zenoh-config est fourni.")
    p.add_argument("--zenoh-config", default=None,
                   help="Fichier JSON5 de config Zenoh complet (contrôle plus fin que --connect).")
    return p.parse_args()


def build_zenoh_config(args):
    if args.zenoh_config:
        return zenoh.Config.from_file(args.zenoh_config)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", json.dumps([args.connect]))
    conf.insert_json5("scouting/multicast/enabled", "false")
    return conf


def main():
    args = parse_args()
    hub = Hub()
    cache = Cache()

    def on_state(sample):
        d = _decode(sample)
        cache.note_state(d)
        hub.broadcast("state", d)

    def on_event(sample):
        d = _decode(sample)
        cache.note_event(d)
        hub.broadcast("event", d)

    def on_link(sample):
        d = _decode(sample)
        cache.note_link(d)
        hub.broadcast("link", d)

    conf = build_zenoh_config(args)
    session = zenoh.open(conf)
    pub_cmd = session.declare_publisher(args.key_cmd)
    # Références gardées vivantes pour toute la durée de vie du process.
    subs = [
        session.declare_subscriber(args.key_state, on_state),
        session.declare_subscriber(args.key_event, on_event),
        session.declare_subscriber(args.key_link, on_link),
    ]

    server = WebServer((args.host, args.http_port), Handler,
                       ui_path=args.ui, pub_cmd=pub_cmd, hub=hub, cache=cache)
    shown = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print(f"[web] interface de test : http://{shown}:{args.http_port}", file=sys.stderr)
    print(f"[web] commandes -> {args.key_cmd!r} | télémétrie <- "
          f"{args.key_state!r}/{args.key_event!r}/{args.key_link!r}", file=sys.stderr)
    print(f"[web] routeur zenoh : {args.connect}", file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        for s in subs:
            try:
                s.undeclare()
            except Exception:
                pass
        session.close()


if __name__ == "__main__":
    main()
