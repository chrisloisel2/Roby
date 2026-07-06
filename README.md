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

## Robot réel : pont vers phd_mobile_base (ROS 2)

Sur le robot actuel, la base mobile est pilotée par un projet ROS 2 distinct
et déjà existant : `~/02_RosBaseMobile/phd_mobile_base` (skid-steer 4 roues
DAMIAO DM-2325, machine d'états de sécurité `DISABLED/ENABLED/ESTOP/FAULT`).
**C'est ce projet, pas `robot/robot_agent.py` de ce dépôt, qui pilote
réellement le robot.**

`robot_agent.py` reste un gabarit générique (utile pour un robot sans stack
ROS 2 existante, ou pour un futur sous-système type bras/mât). Pour la base
mobile de ce robot, le pont Zenoh↔ROS 2 est un nœud dédié,
`zenoh_bridge_node`, ajouté dans le paquet `phd_mobile_base` (voir son
propre README, section « Téléopération réseau (Zenoh) »). **Ne pas lancer
`robot_agent.py` en même temps que `zenoh_bridge_node`** : les deux
publient sur les mêmes clés `robot/heartbeat` / `robot/state`.

```bash
# Sur le PC robot, après colcon build --symlink-install :
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
export OPERATOR_IP=<ip_pc_operateur>
ros2 launch phd_mobile_base simulation.launch.py &   # ou bringup.launch.py pour le matériel
~/02_RosBaseMobile/.venv-zenoh/bin/python3 -m phd_mobile_base.nodes.zenoh_bridge_node
```

`camera_pub.py` de ce dépôt reste valable tel quel (aucune caméra
disponible sur ce robot pour l'instant — `/dev/video*` absent).

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
| `robot/cmd/reset`            | ->   | réarme après ESTOP/FAULT (jamais auto-enable) |
| `operator/deadman`           | ->   | `"true"` / `"false"`                      |
| `robot/heartbeat`            | <-   | vivacité robot (~5 Hz)                    |
| `robot/state`                | <-   | état robot (schéma dépend du récepteur : voir ci-dessous) |
| `robot/camera/front/jpeg`    | <-   | image JPEG                                |

Le contrat `base` est **normalisé** `[-1, 1]` côté opérateur. Qui applique les
limites physiques dépend du récepteur : `robot_agent.py` générique
(`MAX_LINEAR`/`MAX_ANGULAR`) ou, sur ce robot, `command_mux_node` de
phd_mobile_base (`max_linear_speed`/`max_angular_speed` dans son
`robot.yaml`). Le schéma JSON de `robot/state` diffère aussi selon la
source : `{moving, estop, deadman_ok, fresh_cmd}` pour `robot_agent.py`
générique, ou `{state, reason, command_timeout, motion_allowed}` (le FSM
réel `DISABLED/ENABLED/ESTOP/FAULT`) via `zenoh_bridge_node`. L'UI web
affiche ce second schéma.

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
