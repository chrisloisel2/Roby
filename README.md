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

## Installation

PC opérateur :

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

PC robot — **ne pas** `pip install -r requirements.txt` tel quel : le wheel
PyPI générique `opencv-python` est compilé sans GStreamer, et son backend
V4L2 a échoué purement et simplement sur au moins une caméra USB utilisée
ici (`can't open camera by index`), alors que l'OpenCV système (apt,
GStreamer) ouvre la même caméra sans problème. Créer le venv avec
`--system-site-packages` pour hériter du cv2 système et n'installer que
`eclipse-zenoh` par-dessus :

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install eclipse-zenoh
```

## Configuration réseau

Le routeur `zenohd` tourne sur le **PC opérateur**. Le PC robot s'y connecte.
Renseigne l'IP de l'opérateur, soit dans `config/robot_zenoh.json5`, soit via
la variable d'environnement `OPERATOR_IP` (prioritaire) :

```bash
OPERATOR_IP=192.168.15.106 python robot/robot_agent.py
```

`zenohd` écoute sur `0.0.0.0:7447` (voir `config/router.json5`), donc
n'importe quelle IP joignable du PC opérateur fonctionne : IP locale
(192.168.x, plus faible latence, à utiliser quand les deux PC sont sur le
même LAN — cas actuel, IP par défaut ci-dessus) ou IP **Tailscale** (100.x,
quand robot et opérateur ne sont pas sur le même réseau). `ipconfig getifaddr
en0` (local) ou `tailscale ip -4` (Tailscale) sur le PC opérateur donne l'IP
à utiliser si elle change.

## Démarrage

```bash
# PC opérateur
scripts/start_operator.sh          # zenohd + web_server + input_agent

# PC robot
OPERATOR_IP=192.168.15.106 scripts/start_robot.sh
```

## Robot réel : pilotage moteur direct (sans ROS 2)

`robot_agent.py` pilote directement les 4 moteurs mecanum DAMIAO DMS2325 via
le SDK vendor `dmcan`/`damiao` (USB-CAN, libusb) — **pas de ROS 2**. C'est le
même chemin, validé sur ce matériel par un vrai test de rotation (feedback
mesuré conforme à la commande), que
`~/catkin_ws/u2canfd/mecanum_control.py` sur le robot.

`robot/damiao.py` (SDK modifié : type moteur `DMS2325` ajouté) et
`robot/dlls/libdm_device.so` (lib native) sont vendorisés dans ce dépôt —
pas besoin d'un projet externe sur le robot. `dmcan_sdk` et `pyusb` viennent
de pip (voir `requirements.txt`).

Le contrat `robot/cmd/base` est mixé en cinématique mecanum (`vx`=avant,
`vy`=latéral, `wz`=rotation, chacun normalisé `[-1, 1]`) puis mis à l'échelle
par `MAX_VEL`/`ROT_VEL` (rad/s roue, réglés empiriquement sur ce robot —
pas une conversion physique m/s, le rayon de roue n'a jamais été mesuré
fiablement). `robot/cmd/stop` déclenche un `disable_all()` matériel immédiat
(verrouillé) ; `robot/cmd/reset` lève le verrou et réactive les moteurs (le
homme-mort reste requis pour bouger réellement).

> **Bug connu, sans risque** : `Motor_Control.close()` /
> `DmCanContext.__del__` plantent le process (assertion libusb dans la lib
> native) — mais toujours *après* que `disable_all()` ait réussi. Le
> `finally` de `main()` appelle `disable_all()` puis `os._exit(0)` pour
> sauter ce nettoyage bugué plutôt que d'appeler `close()`.

```bash
# Sur le PC robot :
python3 -m venv --system-site-packages .venv
.venv/bin/pip install eclipse-zenoh dmcan_sdk pyusb
export OPERATOR_IP=<ip_pc_operateur>
.venv/bin/python3 robot/robot_agent.py
```

`camera_pub.py` de ce dépôt fonctionne avec la caméra USB branchée sur ce
robot (HSTD USB3.0). Deux pièges rencontrés, tous deux déjà corrigés dans le
code : ne pas forcer `CAP_PROP_FPS` via `cap.set()` (casse la pipeline
GStreamer sur cette caméra — le débit ~15 FPS est donc limité côté logiciel
via `time.sleep`), et l'index `/dev/videoN` n'est pas fiable d'un boot à
l'autre (renumérotation USB) — `camera_pub.py` sonde et retient
automatiquement le premier index qui délivre une vraie frame.

Interface web : `http://IP_DU_PC_OPERATEUR:8080`

L'interface web est un **vrai poste de pilotage**, pas seulement un visualiseur :
flux caméra, tuiles d'état (robot / caméra / homme-mort / mouvement), jauges de
vitesse, et pilotage clavier + pavé à l'écran + manette.

- **Espace** (maintenu) = homme-mort, obligatoire pour bouger
- **W/S** avant·arrière · **A/D** rotation · **Q/E** latéral
- **X** ou le gros bouton = arrêt d'urgence · curseurs vitesse max + pince

### Manette (Gamepad API)

Le panneau **Manette**, en bas de la colonne de droite, est rétractable (cliquer
sur son titre). Il utilise la Gamepad API du navigateur — testé avec une
**Thrustmaster T.Flight Stick X** (4 axes, 12 boutons, 1 hat) branchée en USB
sur le PC opérateur, mais fonctionne avec n'importe quelle manette reconnue par
le navigateur.

Le mapping est **entièrement dynamique**, pas câblé en dur : pour chaque
action (avant/arrière, latéral, rotation, vitesse max, homme-mort, arrêt
d'urgence, réarmer, pince ouvrir/fermer), cliquer « Assigner » puis bouger
l'axe ou appuyer sur le bouton physique voulu — l'index est capturé et
persisté dans le `localStorage` du navigateur. Un encart affiche les valeurs
brutes des axes/boutons pour aider à repérer les indices, et « réinitialiser
le mapping » revient aux valeurs par défaut (calibrées pour la T.Flight Stick
X : axe 1 = avant/arrière inversé, axe 0 = latéral, axe 2 = rotation, axe 3 =
slider de vitesse max, boutons 0–4 = homme-mort/stop/reset/pince).

La manette se **combine** avec clavier et pavé tactile (sommée et bornée à
`[-1, 1]`) plutôt que de les remplacer ; le homme-mort est actif si la manette
*ou* le clavier/pavé le tient enfoncé.

> N'utiliser **qu'une seule source de commande à la fois** : l'interface web *ou*
> `input_agent.py` (joystick) — les deux publient sur les mêmes topics.

## Topics Zenoh

| Clé                          | Sens | Contenu                                   |
| ---------------------------- | ---- | ----------------------------------------- |
| `robot/cmd/base`             | ->   | `{"vx","vy","wz"}` normalisés `[-1, 1]`   |
| `robot/cmd/arm`              | ->   | `{"joints":[...], "gripper", "mode"}`     |
| `robot/cmd/stop`             | ->   | arrêt d'urgence (latch)                   |
| `robot/cmd/reset`            | ->   | lève le verrou E-stop + réactive les moteurs (deadman toujours requis pour bouger) |
| `operator/deadman`           | ->   | `"true"` / `"false"`                      |
| `robot/heartbeat`            | <-   | vivacité robot (~5 Hz)                    |
| `robot/state`                | <-   | `{moving, estop, deadman_ok, fresh_cmd, ts}` |
| `robot/camera/front/jpeg`    | <-   | image JPEG                                |

Le contrat `base` est **normalisé** `[-1, 1]` côté opérateur ; `robot_agent.py`
le mixe en cinématique mecanum puis met à l'échelle par `MAX_VEL`/`ROT_VEL`
(rad/s roue) avant d'envoyer les trames MIT aux 4 moteurs.

## Sécurité

Le **watchdog est local au PC robot** (`robot_agent.py`) et ne dépend jamais du
web server. Le robot est mis à l'arrêt (vitesse roue nulle, moteurs actifs) si :

- le deadman n'est pas `"true"` et récent (`DEADMAN_TIMEOUT_SEC`) ;
- aucune commande base fraîche (`CMD_TIMEOUT_SEC`) — couvre la perte réseau ;
- un `robot/cmd/stop` a été reçu — **verrouillé** : `disable_all()` matériel
  immédiat, moteurs désactivés jusqu'à `robot/cmd/reset` (ou redémarrage).

Vitesses bornées côté robot (`MAX_VEL`/`ROT_VEL` dans `robot_agent.py`).
Avant tout essai réel : ajouter un **arrêt d'urgence physique** en plus de
ces protections logicielles.

## Notes / limites

- Pas de bras/mât/pince sur ce robot : `apply_arm_command`/
  `apply_gripper_command` dans `robot_agent.py` restent des points
  d'extension documentés (no-op), pas des TODO à combler pour que la base
  fonctionne.
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
