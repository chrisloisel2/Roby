# Roby

Téléopération d'un robot (base mobile + bras) via **Zenoh**, avec joystick,
GELLO, caméra et interface web. Deux PC : opérateur et robot.

```text
[Joystick + GELLO]
        v
[PC opérateur]  input_agent.py · web_server.py · zenohd
        |  Zenoh TCP (base/bras/état)      ^  WebSocket direct (caméra, port 8765)
        v                                  |
[PC robot]      robot_agent.py (base) · arm_agent.py (bras) · camera_pub.py
        v
[Robot + bras + caméra]
```

Seuls base/bras/état passent par Zenoh + web_server.py. La caméra est un
lien **WebSocket direct** navigateur <-> `camera_pub.py` (pas de hop Zenoh ni
web_server.py sur ce chemin) — voir [Structure](#structure) et la table des
clés Zenoh plus bas.

`robot_agent.py` (base) et `arm_agent.py` (bras) sont deux process **séparés**
sur le PC robot, avec chacun son propre watchdog de sécurité local — voir
[Sécurité](#sécurité) et [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower).

## Structure

```text
config/
  router.json5          routeur zenohd (PC opérateur)
  operator_zenoh.json5  client Zenoh opérateur (-> localhost)
  robot_zenoh.json5     client Zenoh robot (-> IP opérateur)
operator/
  input_agent.py         joystick + GELLO -> commandes robot
  web_server.py          pont Zenoh <-> navigateur pour base/bras/état (FastAPI + WebSocket) -- PAS la caméra
  web/
    index.html           UI opérateur (markup seul)
    static/app.css       design system (thème sombre poste de pilotage)
    static/js/           modules ES : config (store central), net, camera,
                         status, control, joystick, gello, settings, main
  gello_reader.py        lit le GELLO en série, produit des angles calibrés follower
  gello_calibration.json calibration GELLO déjà mesurée (copie de mon_gello.json)
robot/
  robot_agent.py         base mecanum : applique les commandes + watchdog LOCAL
  arm_agent.py            bras B601 : idem, process séparé (env conda lerobot)
  camera_pub.py           caméra -> JPEG -> WebSocket direct navigateur (port 8765, PAS Zenoh)
scripts/
  start_operator.sh      lance zenohd + web_server + input_agent
  start_robot.sh         lance robot_agent + camera_pub
  start_arm.sh            lance arm_agent (séparé, voir plus bas)
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
GELLO_PORT=/dev/tty.usbserial-XXXX scripts/start_operator.sh   # zenohd + web_server + input_agent

# PC robot
OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # base (Zenoh) + caméra (WebSocket direct, pas besoin d'OPERATOR_IP)
OPERATOR_IP=192.168.15.106 scripts/start_arm.sh     # bras -- voir section dédiée, séparé exprès
```

La page opérateur se connecte à la caméra via `?robotIp=<ip-robot>` dans
l'URL (défaut : `169.254.222.31`, voir `DEFAULT_ROBOT_IP` dans
`operator/web/static/js/camera.js`) — à ajuster si l'IP du robot diffère.

`GELLO_PORT` est optionnel : sans lui, `input_agent.py` tourne normalement
(base + web) mais le bras GELLO reste désactivé (`read_gello()` renvoie
toujours `None`).

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
.venv/bin/pip install eclipse-zenoh dmcan_sdk pyusb websockets
export OPERATOR_IP=<ip_pc_operateur>
.venv/bin/python3 robot/robot_agent.py
```

`camera_pub.py` de ce dépôt fonctionne avec la caméra USB branchée sur ce
robot (HSTD USB3.0), capturée en 1920x1200 et servie directement au
navigateur par son propre serveur WebSocket (`ws://<ip-robot>:8765`, pas de
Zenoh sur ce chemin — n'a donc pas besoin de `OPERATOR_IP`). Deux pièges
rencontrés, tous deux déjà corrigés dans le code : ne pas forcer
`CAP_PROP_FPS` via `cap.set()` (casse la négociation de la pipeline
GStreamer sur cette caméra à cette résolution), et l'index `/dev/videoN`
n'est pas fiable d'un boot à l'autre (renumérotation USB) —
`camera_pub.py` sonde et retient automatiquement le premier index qui
délivre une vraie frame.

## Bras GELLO -> reBot B601 (leader-follower)

Le bras follower (reBot B601-DM, 7 moteurs Damiao CAN) est piloté en
leader-follower par un GELLO maison (Arduino + 7 capteurs magnétiques
AS5600L) branché sur le **PC opérateur**. Le code de pilotage du bras
(classe `RebotB601Follower`, calibration déjà faite) vient d'un projet
externe déjà validé sur ce matériel : `~/03_JelloSoft/rebot_lerobot/` sur le
PC robot, basé sur le framework `lerobot` (env conda `lerobot`) — voir
`~/03_JelloSoft/rebot_lerobot/scripts/README.md` sur place pour le détail de
cette calibration.

```text
GELLO (série, PC opérateur)                    B601 (CAN, PC robot)
      |  gello_reader.py (lecture + calibration déjà faite)
      v
input_agent.py --robot/cmd/arm (Zenoh)--> arm_agent.py --send_action()--> follower
```

Points clés :

- **Deux process séparés côté robot**, volontairement : `arm_agent.py`
  réutilise `RebotB601Follower` tel quel, qui importe le package `lerobot`
  complet (dépendances lourdes, dont torch) — hors de question de mélanger
  ça avec la boucle 100Hz déjà validée de `robot_agent.py` (base). Doit donc
  tourner avec le python de l'env conda `lerobot`, pas le `.venv` du projet
  (voir `scripts/start_arm.sh`, override `ARM_PYTHON=` si le chemin de l'env
  diffère).
- **`gello_reader.py` ne dépend pas de lerobot** : réimplémentation autonome
  (pyserial) de la lecture série + de la formule de calibration de
  `GelloAs5600Leader.get_action()` (lissage -> clip aux butées -> sens ->
  échelle -> offset), avec les mêmes constantes et le même fichier de
  calibration déjà mesuré (`operator/gello_calibration.json`, copie de
  `mon_gello.json`). Nécessite `GELLO_PORT` (variable d'env, ex.
  `/dev/tty.usbserial-XXXX` sur macOS) — sans elle, désactivé proprement (pas
  d'auto-probe : le mauvais port série n'est pas un risque à prendre).
- **Gating indépendant de la base** : contrairement à `robot/cmd/base`, le
  bras n'est **pas** gaté par le homme-mort manette (piloter un bras 7 DOF
  demande les deux mains). `arm_agent.py` a son propre watchdog de fraîcheur
  (`ARM_CMD_TIMEOUT_SEC`) : sans commande fraîche, il arrête juste d'envoyer
  de nouvelles cibles (les moteurs Damiao tiennent seuls leur dernière
  consigne — rien d'équivalent au ramp-to-zero de la base n'est nécessaire).
  `robot/cmd/stop`/`robot/cmd/reset` restent **partagés** avec la base : un
  seul arrêt d'urgence coupe les deux.
- **Filet de sécurité supplémentaire** pour ce premier vrai essai sur ce
  chemin de pilotage : `max_relative_target` (quelques degrés par tick, voir
  `ARM_MAX_RELATIVE_TARGET_DEG` dans `arm_agent.py`), en plus du lissage déjà
  fait côté GELLO.

> Avant tout essai : `connect()` active le couple moteur du bras
> **immédiatement** — bras dégagé/soutenu, comme pour la base.

Interface web : `http://IP_DU_PC_OPERATEUR:8080`

L'interface web est un **vrai poste de pilotage**, pas seulement un visualiseur :
flux caméra (plein écran, contain/cover), tuiles d'état (robot / caméra /
homme-mort / base / bras + angles des joints), jauges de télémétrie, et
pilotage clavier + pavé à l'écran + manette + GELLO.

- **Espace** (maintenu) = homme-mort, obligatoire pour bouger
- Touches lettres détectées par **position physique** (`event.code`) : le même
  geste marche en QWERTY (WASD + Q/E) et en AZERTY (ZQSD + A/E)
- **X** ou le gros bouton = arrêt d'urgence · **R** réarmer · **F** plein
  écran · **?** aide raccourcis · curseurs vitesse max + pince

### Réglages (⚙)

Toute la configuration de l'UI passe par un **store central versionné**
(`roby.config.v2` en localStorage, modules `static/js/config.js` +
`settings.js`) exposé dans le panneau ⚙ du header, en quatre onglets :

- **Contrôle** : fréquence d'envoi des commandes (10–50 Hz), vitesse max au
  chargement, mémorisation de la vitesse, pas de la pince ;
- **Manette** : zone morte des axes (le mapping par action reste dans le
  panneau Manette de la page, avec les valeurs brutes en direct) ;
- **GELLO** : débit série, délai de boot Arduino, lissage, marge aux butées,
  reconnexion automatique au chargement (dernier port autorisé) ;
- **Interface** : ajustement vidéo, affichage télémétrie / angles bras, seuil
  « signal perdu ».

La config s'**exporte/importe en JSON** (bouton dans le panneau — pratique
pour répliquer un poste opérateur) ; l'import est validé clé par clé contre
les valeurs par défaut (types vérifiés, clés inconnues ignorées). Les
anciennes clés localStorage éparses (`roby.browserControl`,
`roby.joystick.mapping.v1`, …) sont migrées automatiquement au premier
chargement.

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
persisté dans le store de configuration central (voir [Réglages](#réglages-)).
Un encart affiche les valeurs
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
| `robot/cmd/arm`              | ->   | `{"joints": {nom: deg, ...}, "gripper", "mode"}` (déjà calibré côté follower, degrés) |
| `robot/cmd/stop`             | ->   | arrêt d'urgence (latch) — partagé base + bras |
| `robot/cmd/reset`            | ->   | lève le verrou E-stop + réactive les moteurs (deadman toujours requis pour bouger la base) — partagé base + bras |
| `operator/deadman`           | ->   | `"true"` / `"false"` (base uniquement, pas le bras) |
| `robot/heartbeat`            | <-   | vivacité robot base (~5 Hz)               |
| `robot/state`                | <-   | `{moving, estop, deadman_ok, fresh_cmd, ts}` (base) |
| `robot/arm/state`            | <-   | `{connected, moving, fresh_cmd, estop, joints, ts}` (bras) |

La caméra n'est **pas** un topic Zenoh : `camera_pub.py` sert le JPEG
directement au navigateur via son propre serveur WebSocket
(`ws://<ip-robot>:8765`), voir [Structure](#structure) et le diagramme en
tête de fichier.

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

Le bras a son **propre watchdog local**, dans `arm_agent.py`, indépendant de
celui de `robot_agent.py` (process séparé) : pas de nouvelle consigne
envoyée sans commande `robot/cmd/arm` fraîche (`ARM_CMD_TIMEOUT_SEC`), plus
`max_relative_target` comme filet en plus du lissage déjà fait côté GELLO.
`robot/cmd/stop` coupe aussi le bras (`disable_torque()`) ; `robot/cmd/reset`
le réarme. Détails : [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower).

## Notes / limites

- `apply_arm_command`/`apply_gripper_command` dans `robot_agent.py` restent
  des no-op **volontaires** : le bras est piloté par le process séparé
  `arm_agent.py` (voir [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower)),
  pas par `robot_agent.py`.
- Caméra : JPEG en 1920×1200, servi par `camera_pub.py` directement au
  navigateur en WebSocket (`ws://<ip-robot>:8765`), plus de hop Zenoh ni
  web_server.py sur ce chemin — Zenoh garde les commandes, l'état, le
  heartbeat et la supervision.

## Plan de réalisation

1. Ping Zenoh entre les deux PC
2. Joystick -> réception robot
3. Watchdog + stop robot
4. Caméra JPEG
5. Serveur web + affichage caméra
6. ~~GELLO -> `robot/cmd/arm`~~ fait — voir [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower)
7. Limites de vitesse + deadman + arrêt d'urgence
