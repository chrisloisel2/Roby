# Téléopération du mât (chariot pas-à-pas) — Bridge Arduino ↔ Zenoh

Guide complet pour **piloter le chariot du mât à distance**, via des commandes
publiées sur des topics Zenoh. Décrit la chaîne complète (front opérateur →
Zenoh → pont série → Arduino), le protocole de commande, la télémétrie
remontée, et fournit un exemple de client Python autonome.

> Le mât est un chariot sur vis sans fin entraîné par un moteur pas-à-pas
> NEMA23 boucle fermée (driver CL57T, STEP/DIR), frein électromagnétique 24 V,
> fins de course haut/bas. Le firmware Arduino (`src/main.cpp`, ce dossier)
> gère le mouvement temps réel ; tout le reste passe par Zenoh.

Pour piloter le mât **depuis l'interface web principale** (boutons
Monter/Descendre, touches `PgUp`/`PgDn`), voir le `README.md` à la racine du
dépôt, section **Mât (chariot pas-à-pas)** — ce fichier-ci ne couvre que le
protocole bas niveau (firmware + pont série), utile pour du debug CLI ou un
client custom.

---

## 1. Architecture

```
  ┌──────────────────────┐        ┌──────────────────────┐        ┌───────────────────┐
  │  PC OPÉRATEUR         │        │   PC ROBOT            │        │   ARDUINO (mât)   │
  │  navigateur / script  │        │                      │        │                   │
  │  session Zenoh CLIENT │  WiFi  │  zenohd (routeur)     │  USB   │  firmware v2      │
  │   (localhost)         ├───TCP──►  :7447                │  série │  (ATmega328)      │
  │   pub robot/mast/cmd  │        │  ▲                    │ 115200 │                   │
  │   sub robot/mast/state│◄───────┤  │                    │        │  STEP/DIR → CL57T │
  │   sub robot/mast/event│        │  │ robot/              ├────────►                   │
  │   sub robot/mast/link │        │  │ mast_serial_bridge.py│        │  télémétrie 60 Hz │
  └──────────────────────┘        │  └───────────────────    │        └───────────────────┘
                                   └──────────────────────┘
```

- **`robot/mast_serial_bridge.py`** (racine du dépôt) — le pont. Tourne sur le
  PC robot, branché en USB à l'Arduino. Traduit les commandes Zenoh en lignes
  du protocole série, et republie la télémétrie série sur des topics Zenoh.
  **C'est le seul composant obligatoire** pour téléopérer, au même titre que
  `robot_agent.py`/`arm_agent.py` — lancé par `scripts/start_robot.sh` (flag
  `NO_MAST=1` pour l'exclure).
- **`src/main.cpp`** (ce dossier, PlatformIO) — le firmware Arduino.
- **Interface** — les boutons Monter/Descendre de la page opérateur
  (`operator/web/static/js/control.js`), relayés par `web_server.py` sur
  `robot/mast/cmd` comme le reste des commandes base/pince. Pas de dashboard
  dédié ici (contrairement à une version antérieure de ce bridge) : tout passe
  par la même page que le reste du robot.

### Topologie Zenoh

**Important — différent de ce que suggère un déploiement autonome de ce
firmware** : dans CE dépôt, le routeur `zenohd` tourne sur le **PC opérateur**
(voir `config/router.json5`), pas sur le robot. Le PC robot (et donc
`mast_serial_bridge.py`) s'y connecte en mode client via la variable
d'environnement `OPERATOR_IP`, exactement comme `robot_agent.py`/`arm_agent.py`
(voir `robot/zenoh_config.py` — `OPERATOR_IP` est **requis**, pas de valeur par
défaut silencieuse). Le multicast est désactivé (comportement déterministe sur
WiFi).

- `mast_serial_bridge.py` (sur le robot) → `OPERATOR_IP=<ip> python3 robot/mast_serial_bridge.py`
  (ou via `scripts/start_robot.sh`, qui le fait automatiquement)
- Test au banc, sans le stack complet, routeur local → `--connect tcp/localhost:7447`
  (voir §6.4)

---

## 2. Matériel & port série

| Élément | Valeur |
|---|---|
| Port série Arduino | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (défaut `MAST_PORT`, voir `robot/mast_serial_bridge.py`) |
| Débit (baud) | `115200` |
| Format | 8N1, texte, une commande/réponse **par ligne** (`\n`) |

Utilise **toujours** le chemin `/dev/serial/by-id/...` plutôt que `/dev/ttyUSB0` :
il est stable même si l'ordre d'énumération USB change au reboot ou au rebranchement.

Permissions (une fois) — ajoute l'utilisateur au groupe `dialout` puis reconnecte
la session :

```bash
sudo usermod -aG dialout "$USER"      # se déconnecter/reconnecter ensuite
ls -l /dev/serial/by-id/              # vérifier que le lien existe
```

---

## 3. Topics Zenoh (le contrat)

Toutes les clés sont configurables via les options `--key-*` du bridge ; valeurs par défaut :

| Topic (clé) | Sens | Type de charge utile | QoS |
|---|---|---|---|
| `robot/mast/cmd` | **client → mât** (commandes) | JSON *ou* ligne série brute | (abonnement bridge) |
| `robot/mast/state` | **mât → client** (télémétrie) | JSON position/fins de course | DROP · BEST_EFFORT · DATA |
| `robot/mast/event` | **mât → client** (acquittements) | texte `ACK/MSG/WARN/ERR` | RELIABLE |
| `robot/mast/link` | **mât → client** (lien série) | texte `Connected`/`Disconnected` | RELIABLE |

### `robot/mast/state` — télémétrie (publiée ~60 Hz)

```json
{"position_mm": 342.5, "fdc_min": false, "fdc_max": false, "t": 1783616345.42}
```

| Champ | Sens |
|---|---|
| `position_mm` | Position courante du chariot en mm (source firmware : codeur si supervision active, sinon comptage de pas). |
| `fdc_min` / `fdc_max` | Fin de course bas / haut (`true` = déclenché). |
| `t` | Timestamp Unix (secondes, float) posé par le bridge à la réception. |

> **Vitesse :** le topic `state` ne contient **pas** la vitesse. Elle est déduite
> côté consommateur des couples (`t`, `position_mm`) — voir l'exemple §7,
> `VelocityEstimator`.

### `robot/mast/event` — acquittements & messages

Chaque commande reçue par le firmware est acquittée par `ACK <commande>`, suivie
éventuellement d'un `MSG:`/`WARN:`/`ERR:`. Route par préfixe :

| Préfixe | Signification |
|---|---|
| `ACK <cmd>` | Commande reçue (pas forcément exécutée — lire la ligne suivante). |
| `MSG:HOMING,<n>/4,<phase>` | Progression du homing (SEEK_MIN → BACKOFF → SEEK_MAX → RETURN_MIN). |
| `MSG:HOMING_OK,COURSE:<mm>` | Homing réussi ; `<mm>` = course utile mesurée. |
| `MSG:REFUS,<raison>` | Commande refusée : `HOMING_REQUIS`, `BUSY`, `BUSY,STOP_D_ABORD`, `FDC`. |
| `MSG:RETARGET` | Nouvelle consigne de position acceptée en vol. |
| `MSG:VEL_STOP` / `MSG:JOG_STOP` | Fin de mode vitesse / jog. |
| `WARN:ENC_ABSENT,SUPERVISION_OFF` | Codeur muet détecté au homing → boucle ouverte explicite. |
| `WARN:POS_TOL,ERR_MM:<x>` | Erreur résiduelle après rattrapage. |
| `ERR:STALL,POS:<mm>,DEV:<mm>` | Décrochage détecté (écart pas/codeur), arrêt + frein. |
| `ERR:ENC_OR_STALL,...` | Écart + codeur muet (blocage **ou** codeur mort) → re-homing requis. |
| `ERR:HOMING,<raison>` | Fin de course introuvable pendant le homing. |
| `MSG:REFUS_BRIDGE,<...>` | Refus **côté pont** (JSON invalide, série déconnectée). |

`robot/mast/event` n'est **pas** relayé jusqu'au navigateur par `web_server.py`
(volontairement — voir son docstring) : c'est un flux de debug CLI
(`z_sub -k robot/mast/event`), pas un élément de l'UI.

### `robot/mast/link`

`Connected` / `Disconnected` — publié seulement au **changement** d'état du lien série
(RELIABLE). `web_server.py` le relaie tel quel dans la tuile "Mât" de la page
opérateur (`linked`, voir §Structure du README racine).

---

## 4. Protocole de commande (`robot/mast/cmd`)

Le pont accepte **deux formes** de charge utile. La forme JSON est recommandée pour
les applications ; la forme brute est pratique en CLI.

### 4.1 Forme JSON (recommandée) — champ `action` obligatoire

| Charge utile publiée sur `robot/mast/cmd` | Ligne série générée | Effet |
|---|---|---|
| `{"action":"home"}` | `H` | Homing complet (requiert l'état repos). |
| `{"action":"position","mm":342.5}` | `POS:342.50` | Position absolue (mm), **requiert un homing**. Retarget en vol accepté. |
| `{"action":"velocity","mm_s":40}` | `VEL:40.0` | Mode vitesse : `>0` monte, `<0` descend, `0` arrête. **Watchdog 300 ms** (voir §5). C'est l'action utilisée par les boutons Monter/Descendre du front. |
| `{"action":"velocity","mm_s":-25}` | `VEL:-25.0` | Descente à 25 mm/s. |
| `{"action":"jog","dir":"up","state":"start"}` | `UP_START` | Jog continu vers le haut (libère le frein). |
| `{"action":"jog","dir":"up","state":"stop"}` | `UP_STOP` | Arrête le jog haut, resserre le frein. |
| `{"action":"jog","dir":"down","state":"start"}` | `DOWN_START` | Jog continu vers le bas. |
| `{"action":"jog","dir":"down","state":"stop"}` | `DOWN_STOP` | Arrête le jog bas. |
| `{"action":"stop"}` | `STOP` | **Arrêt d'urgence** (coupe le mouvement, serre le frein). |
| `{"action":"brake","engaged":true}` | `BRAKE:0` | Frein **serré**. |
| `{"action":"brake","engaged":false}` | `BRAKE:1` | Frein **libéré** (⚠ `WARN:...RISQUE_CHUTE` à l'arrêt). |
| `{"action":"fdc"}` | `FDC` | Test des fins de course pendant 10 s. |
| `{"action":"raw","cmd":"POS:100"}` | `POS:100` | Passthrough explicite d'une ligne série. |

**Alias tolérés :** `action` `pos`/`move` = `position` ; `vel`/`vitesse` = `velocity` ;
`estop`/`emergency_stop` = `stop` ; `homing` = `home` ; `test_fdc` = `fdc`.
Champs vitesse acceptés : `mm_s`, `v_z`, `v`, `speed`. Champ position : `mm` ou `target_z`.

### 4.2 Forme brute (CLI / debug)

Tout payload **ne commençant pas par `{`** est envoyé tel quel comme ligne série.
Exemples : `H`, `STOP`, `POS:342.5`, `VEL:30`, `UP_START`, `BRAKE:0`, `FDC`.

### 4.3 Protocole série firmware (référence bas niveau)

Ce que le firmware comprend réellement, si tu veux envoyer du brut :

| Ligne | État requis | Effet |
|---|---|---|
| `H` | repos | Homing (MIN → recul 3 mm → MAX en comptant → retour MIN → zéro). |
| `POS:<mm>` | homé | Déplacement asservi vers une position absolue, borné `[0, course]`. |
| `VEL:<mm/s>` | repos ou mode vitesse | Vitesse continue signée, bornée à `±VEL_MAX_MM_S`, **watchdog homme-mort**. |
| `UP_START`/`DOWN_START` | repos | Jog continu (inversion en vol possible). |
| `UP_STOP`/`DOWN_STOP` | — | Arrête le jog. |
| `STOP` | — | Arrêt d'urgence immédiat. |
| `BRAKE:1` / `BRAKE:0` | — | Libère / serre le frein. |
| `FDC` | repos | Test fins de course 10 s (non bloquant). |

---

## 5. Sécurité — à lire avant de piloter un axe vertical

- **Watchdog du mode vitesse (VEL), sur le firmware lui-même.** Après un
  `VEL:<v≠0>`, si le firmware ne reçoit **aucun nouveau `VEL` pendant 300 ms**,
  il arrête le chariot et serre le frein (`VEL_TIMEOUT_MS` dans `src/main.cpp`).
  → Pour un mouvement continu (bouton maintenu), **réémets la consigne
  ≥ 10 Hz** (période ≤ ~100 ms) ; `control.js` le fait à la fréquence de la
  boucle de contrôle (`control.rateHz`, 10–50 Hz, voir README racine). Si le
  lien tombe (WiFi, onglet fermé), le mât s'arrête seul **sans dépendre du
  navigateur ni de `web_server.py`** — même philosophie que le watchdog local
  de `robot_agent.py` pour la base. À la relâche du bouton, un `VEL:0` explicite
  est envoyé une fois (pas de spam en continu à l'arrêt, `VEL:0` au repos étant
  un no-op côté firmware).
- **`POS:` exige un homing** préalable, sinon `MSG:REFUS,HOMING_REQUIS`. `VEL:` et le
  jog fonctionnent **sans** homing (pratique pour dégager le chariot).
- **Le homing exige l'état repos.** Envoyer `H` pendant un mouvement →
  `MSG:REFUS,BUSY,STOP_D_ABORD` : fais un `STOP` d'abord.
- **Fins de course.** Démarrer un `VEL`/jog dans une butée déjà active est refusé
  (`MSG:REFUS,FDC`). Le firmware stoppe sur contact de fin de course.
- **Frein.** Géré automatiquement pendant les mouvements. `BRAKE:1` (libéré) à
  l'arrêt renvoie un avertissement de risque de chute — le chariot peut tomber si
  le driver n'assure pas le maintien.
- **`STOP`** est traité en priorité et annule mouvement/homing/jog/VEL en cours.
  Le bouton "ARRÊT D'URGENCE" de la page opérateur (`robot/cmd/stop`) est
  **partagé avec la base/le bras**, PAS avec le mât : couper le mât
  spécifiquement se fait en relâchant Monter/Descendre (watchdog VEL) ou via
  `{"action":"stop"}` sur `robot/mast/cmd` directement (envoyé automatiquement
  par `web_server.py` à la fermeture de la connexion navigateur, voir son
  docstring).

---

## 6. Lancement

### 6.1 Dépendances

```bash
pip install pyserial eclipse-zenoh
```

Sur le PC robot, si tu utilises le `.venv` partagé du dépôt (voir README
racine, § Installation), `pyserial` et `eclipse-zenoh` y sont déjà listés
(`requirements.txt`).

### 6.2 Stack complet (recommandé) — via `scripts/start_robot.sh`

```bash
OPERATOR_IP=192.168.15.111 scripts/start_robot.sh              # lance aussi mast_serial_bridge.py
NO_MAST=1 OPERATOR_IP=192.168.15.111 scripts/start_robot.sh    # ... sans le mât (Arduino débranché/en panne)
```

Le port série par défaut est `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
(override : `MAST_PORT=/dev/serial/by-id/... scripts/start_robot.sh`). Logs :
`logs/mast_serial_bridge.log`.

### 6.3 Le pont seul (sur le robot, branché à l'Arduino)

```bash
OPERATOR_IP=192.168.15.111 python3 robot/mast_serial_bridge.py
```

Options utiles : `--port`, `--baud` (défaut 115200), `--zenoh-config <fichier.json5>`,
`--key-cmd/--key-state/--key-event/--key-link`, `--silence-timeout` (délai sans trame
avant `Disconnected`), `--reconnect-delay`.

### 6.4 Test au banc (routeur local, sans le stack complet)

Utile pour tester le firmware/bridge seuls, sans `OPERATOR_IP` ni le reste du
robot :

```bash
zenohd &   # routeur local
python3 robot/mast_serial_bridge.py --port /dev/serial/by-id/usb-xxxx --connect tcp/localhost:7447
```

### 6.5 Test rapide en CLI (sans code)

```bash
# observer la télémétrie (sur le PC où tourne zenohd, ou avec --connect)
z_sub -k 'robot/mast/**' -e tcp/localhost:7447

# envoyer une commande (forme brute)
z_pub -k robot/mast/cmd -e tcp/localhost:7447 -v 'H'
z_pub -k robot/mast/cmd -e tcp/localhost:7447 -v 'VEL:20'
z_pub -k robot/mast/cmd -e tcp/localhost:7447 -v 'STOP'
```

---

## 7. Client Python complet (machine distante)

Client minimal mais complet : s'abonne à la télémétrie, calcule la vitesse, et
expose `home()`, `move_to()`, `jog()` (avec réémission dead-man) et `stop()`.
`ROUTER` pointe le **PC opérateur** (là où tourne `zenohd` dans ce dépôt — voir
§1) ; adapte l'IP à ton réseau.

```python
#!/usr/bin/env python3
"""Client de téléopération du mât via Zenoh. pip install eclipse-zenoh"""
import json, time, threading, collections
import zenoh

ROUTER = "tcp/192.168.15.111:7447"     # zenohd, PC OPÉRATEUR (pas le robot, voir §1)
KEY_CMD, KEY_STATE = "robot/mast/cmd", "robot/mast/state"
KEY_EVENT, KEY_LINK = "robot/mast/event", "robot/mast/link"

class Mast:
    def __init__(self, router=ROUTER):
        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("connect/endpoints", json.dumps([router]))
        conf.insert_json5("scouting/multicast/enabled", "false")
        self.session = zenoh.open(conf)
        self.pub = self.session.declare_publisher(KEY_CMD)
        self.state = {}          # dernier état connu
        self._buf = collections.deque()   # (t, pos) pour la vitesse
        self._subs = [
            self.session.declare_subscriber(KEY_STATE, self._on_state),
            self.session.declare_subscriber(KEY_EVENT, self._on_event),
            self.session.declare_subscriber(KEY_LINK,  self._on_link),
        ]

    # ---- réception ----
    def _on_state(self, s):
        d = json.loads(s.payload.to_bytes())
        d["velocity_mm_s"] = self._velocity(d.get("t"), d.get("position_mm"))
        self.state = d
    def _on_event(self, s): print("[event]", s.payload.to_bytes().decode())
    def _on_link(self, s):  print("[link ]", s.payload.to_bytes().decode())

    def _velocity(self, t, pos, win=0.25):
        if t is None or pos is None: return 0.0
        b = self._buf
        if b and (t <= b[-1][0] or t - b[-1][0] > 0.5): b.clear()
        b.append((t, pos))
        while len(b) > 2 and t - b[0][0] > win: b.popleft()
        return round((pos - b[0][1]) / (t - b[0][0]), 1) if len(b) >= 2 and t > b[0][0] else 0.0

    # ---- envoi ----
    def _send(self, obj): self.pub.put(json.dumps(obj))
    def home(self):            self._send({"action": "home"})
    def move_to(self, mm):     self._send({"action": "position", "mm": float(mm)})
    def stop(self):            self._send({"action": "stop"})
    def set_velocity(self, v): self._send({"action": "velocity", "mm_s": float(v)})

    def jog(self, speed_mm_s, seconds):
        """Déplacement continu 'dead-man' : réémet VEL à 10 Hz puis arrête.
        speed_mm_s > 0 monte, < 0 descend."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.set_velocity(speed_mm_s)     # rafraîchit le watchdog (300 ms)
            time.sleep(0.1)                    # 10 Hz
        self.set_velocity(0)                   # arrêt explicite

    def close(self):
        for s in self._subs:
            try: s.undeclare()
            except Exception: pass
        self.session.close()

if __name__ == "__main__":
    m = Mast()
    try:
        time.sleep(1.0)                        # laisse arriver la 1re télémétrie
        print("position:", m.state.get("position_mm"))
        m.jog(+30, 2.0)                        # monte 2 s à 30 mm/s
        time.sleep(0.5)
        m.jog(-30, 2.0)                        # descend 2 s à 30 mm/s
        # m.home(); time.sleep(0.1)            # (homing puis position absolue)
        # m.move_to(150)
    except KeyboardInterrupt:
        m.stop()
    finally:
        m.stop(); m.close()
```

---

## 8. Dépannage

| Symptôme | Cause probable | Action |
|---|---|---|
| `MSG:REFUS,HOMING_REQUIS` sur `POS:` | Pas encore homé | Envoyer `home()` et attendre `MSG:HOMING_OK`. |
| `MSG:REFUS,BUSY,STOP_D_ABORD` sur `H` | Mouvement en cours | `stop()` d'abord, puis `home()`. |
| `MSG:REFUS,FDC` sur `VEL`/jog | Démarrage dans une butée active | Repartir dans l'autre sens. |
| Le mât s'arrête tout seul en jog/VEL | Watchdog VEL (pas de réémission ≥10 Hz) | Vérifier `control.rateHz` (≥10 Hz) et que le navigateur envoie bien (onglet actif, WebSocket connecté). |
| `ERR:ENC_OR_STALL` au banc | Supervision active sans codeur câblé | Passer `ENCODER_SUPERVISION 0` (firmware) tant que le codeur n'est pas fiable. |
| Tuile "Mât" → DÉCONNECTÉ | Port série absent/débranché, mauvais `MAST_PORT`, ou `mast_serial_bridge.py` pas lancé (`NO_MAST=1` ?) | Vérifier `/dev/serial/by-id/...`, les droits `dialout`, `logs/mast_serial_bridge.log`. |
| `MSG:REFUS_BRIDGE,SERIE_DECONNECTEE` | Bridge lancé mais Arduino non connecté | Rebrancher / relancer `mast_serial_bridge.py` (ou `scripts/start_robot.sh`). |
| Rien ne bouge depuis la page opérateur | `OPERATOR_IP` mal réglé côté robot (voir `robot/zenoh_config.py`), ou deadman pas maintenu | Vérifier `logs/mast_serial_bridge.log` ; le mât est gaté par le **même** homme-mort que la base (voir README racine). |

---

## 9. Aide-mémoire

```
# Démarrer (stack complet, sur le robot)
OPERATOR_IP=<ip_pc_operateur> scripts/start_robot.sh

# Commandes (JSON sur robot/mast/cmd)
{"action":"home"}                      # homing
{"action":"position","mm":150}         # aller à 150 mm (après homing)
{"action":"velocity","mm_s":30}        # monter à 30 mm/s (réémettre >=10 Hz)
{"action":"velocity","mm_s":0}         # arrêter le mode vitesse
{"action":"stop"}                      # ARRÊT D'URGENCE (mât seul)

# Télémétrie (robot/mast/state) : {"position_mm","fdc_min","fdc_max","t"}
# Acquittements (robot/mast/event, debug CLI seulement) : ACK / MSG: / WARN: / ERR:
# Lien série (robot/mast/link)     : Connected / Disconnected
```

Port Arduino : `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` @ 115200 baud
(`MAST_PORT`). Routeur Zenoh : PC opérateur, `tcp/<OPERATOR_IP>:7447`.
