# Roby

Téléopération d'un robot (base mobile + bras) via **Zenoh**, avec joystick,
GELLO, caméra et interface web. Deux PC : opérateur et robot.

```text
[Joystick + GELLO]
        v
[PC opérateur]  input_agent.py · web_server.py · zenohd
        |  Zenoh TCP
        v
[PC robot]      robot_agent.py · camera_pub.py (watchdog local)
        v
[Robot + caméra]
```

## Structure

```text
config/
  router.json5          routeur zenohd (PC opérateur)
  operator_zenoh.json5  client Zenoh opérateur (-> localhost)
  robot_zenoh.json5     client Zenoh robot (-> IP opérateur)
operator/
  input_agent.py        joystick + GELLO -> commandes robot
  web_server.py         pont Zenoh <-> navigateur (FastAPI + WebSocket)
robot/
  robot_agent.py        applique les commandes + watchdog de sécurité LOCAL
  camera_pub.py         caméra -> JPEG -> Zenoh
scripts/
  start_operator.sh     lance zenohd + web_server + input_agent
  start_robot.sh        lance robot_agent + camera_pub
```

## Installation (sur les deux PC)

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Configuration réseau

Le routeur `zenohd` tourne sur le **PC opérateur**. Le PC robot s'y connecte.
Renseigne l'IP de l'opérateur, soit dans `config/robot_zenoh.json5`, soit via
la variable d'environnement `OPERATOR_IP` (prioritaire) :

```bash
OPERATOR_IP=192.168.1.50 python robot/robot_agent.py
```

## Démarrage

```bash
# PC opérateur
scripts/start_operator.sh          # zenohd + web_server + input_agent

# PC robot
OPERATOR_IP=192.168.1.50 scripts/start_robot.sh
```

Interface web : `http://IP_DU_PC_OPERATEUR:8080`

L'interface web est un **vrai poste de pilotage**, pas seulement un visualiseur :
flux caméra, tuiles d'état (robot / caméra / homme-mort / mouvement), jauges de
vitesse, et pilotage clavier + pavé à l'écran.

- **Espace** (maintenu) = homme-mort, obligatoire pour bouger
- **W/S** avant·arrière · **A/D** rotation · **Q/E** latéral
- **X** ou le gros bouton = arrêt d'urgence · curseurs vitesse max + pince

> N'utiliser **qu'une seule source de commande à la fois** : l'interface web *ou*
> `input_agent.py` (joystick) — les deux publient sur les mêmes topics.

## Topics Zenoh

| Clé                          | Sens | Contenu                                   |
| ---------------------------- | ---- | ----------------------------------------- |
| `robot/cmd/base`             | ->   | `{"vx","vy","wz"}` normalisés `[-1, 1]`   |
| `robot/cmd/arm`              | ->   | `{"joints":[...], "gripper", "mode"}`     |
| `robot/cmd/stop`             | ->   | arrêt d'urgence (latch)                   |
| `operator/deadman`           | ->   | `"true"` / `"false"`                      |
| `robot/heartbeat`            | <-   | vivacité robot (~5 Hz)                    |
| `robot/state`                | <-   | état robot (moving, estop, ...)           |
| `robot/camera/front/jpeg`    | <-   | image JPEG                                |

Le contrat `base` est **normalisé** `[-1, 1]` côté opérateur ; c'est le
`robot_agent` qui applique les limites physiques (`MAX_LINEAR`, `MAX_ANGULAR`).

## Sécurité

Le **watchdog est local au PC robot** (`robot_agent.py`) et ne dépend jamais du
web server. Le robot est mis à l'arrêt si :

- le deadman n'est pas `"true"` et récent (`DEADMAN_TIMEOUT_SEC`) ;
- aucune commande base fraîche (`CMD_TIMEOUT_SEC`) — couvre la perte réseau ;
- un `robot/cmd/stop` a été reçu (E-stop verrouillé, nécessite un redémarrage).

Vitesses bornées côté robot (`MAX_LINEAR` / `MAX_ANGULAR`). Avant tout essai
réel : ajouter un **arrêt d'urgence physique** en plus de ces protections
logicielles.

## Notes / limites

- `robot_agent.py` et `camera_pub.py` contiennent des `TODO` : branche ton
  driver robot réel dans `apply_base_command`, `apply_arm_command`, `stop_robot`.
- GELLO : implémente `read_gello()` dans un module `gello_reader.py` importable
  par `input_agent.py` (renvoie un dict `joints`/`gripper`/`mode`, ou `None`).
- Caméra en JPEG/Zenoh = MVP 10–20 FPS en 640×480. Pour de la basse latence
  haute résolution, passer la vidéo en H.264/WebRTC et garder Zenoh pour les
  commandes, l'état, le heartbeat et la supervision.

## Plan de réalisation

1. Ping Zenoh entre les deux PC
2. Joystick -> réception robot
3. Watchdog + stop robot
4. Caméra JPEG
5. Serveur web + affichage caméra
6. GELLO -> `robot/cmd/arm`
7. Limites de vitesse + deadman + arrêt d'urgence
