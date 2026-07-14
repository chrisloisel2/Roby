# Roby

Téléopération d'un robot (base mobile + bras + mât) via **Zenoh**, avec
joystick, GELLO, caméra et interface web. Deux PC : opérateur et robot.

```text
[Joystick + GELLO]
        v
[PC opérateur]  input_agent.py · web_server.py · zenohd
        |  Zenoh TCP (base/bras/mât/état)  ^  WebSocket direct (N caméras, 1 connexion, port 8765)
        v                                  |
[PC robot]      robot_agent.py (base) · arm_agent.py (bras) ·
                mast_serial_bridge.py (mât) · camera_pub.py
        v
[Robot + bras + mât + N caméras USB, découvertes automatiquement]
```

Seuls base/bras/mât/état passent par Zenoh + web_server.py. TOUTES les caméras
détectées sont servies par `camera_pub.py` sur une seule **connexion
WebSocket directe** navigateur <-> robot (pas de hop Zenoh ni
web_server.py sur ce chemin) — voir [Structure](#structure) et la table
des clés Zenoh plus bas. Une seule connexion pour tous les flux,
volontairement (pas un socket par caméra) : chaque caméra reçoit alors
exactement le même traitement réseau (même connexion TCP, même fenêtre
d'écriture, même ordonnancement asyncio) au lieu de sockets indépendants
qui pourraient chacun dériver à leur rythme. Chaque message binaire est
`[1 octet cam_id][JPEG]` ; le navigateur démultiplexe par ce préfixe
(`operator/web/static/js/videoMux.js`), qui reçoit aussi un message texte
JSON listant les caméras actuellement détectées (id, nom V4L2,
résolution) -- **Réglages (⚙) > Caméras** dans la page opérateur laisse
choisir laquelle est affichée en grand cadre et laquelle en vignette (voir
§ Robot réel plus bas pour le détail de la découverte côté robot).

`robot_agent.py` (base), `arm_agent.py` (bras) et `mast_serial_bridge.py`
(mât) sont trois process **séparés** sur le PC robot, chacun avec son propre
watchdog de sécurité local (celui du mât vit sur le firmware Arduino
lui-même) — voir [Sécurité](#sécurité), [Bras GELLO -> reBot
B601](#bras-gello--rebot-b601-leader-follower) et [Mât (chariot
pas-à-pas)](#mât-chariot-pas-à-pas).

## Structure

```text
config/
  router.json5          routeur zenohd (PC opérateur)
  operator_zenoh.json5  client Zenoh opérateur (-> localhost)
  robot_zenoh.json5     client Zenoh robot (-> IP opérateur)
operator/
  input_agent.py         joystick + GELLO -> commandes robot
  web_server.py          pont Zenoh <-> navigateur pour base/état/stop-reset (FastAPI + WebSocket) -- PAS la caméra ni les commandes du bras
  web/
    index.html           UI opérateur (markup seul)
    static/app.css       design system (thème sombre poste de pilotage)
    static/js/           modules ES : config (store central), net, videoMux
                         (connexion caméra partagée), camera, camera2,
                         armLink (WebSocket direct bras), status, control,
                         joystick, gello, settings, birdview (vue spatiale
                         vue de dessus, voir § Vue spatiale), main
  gello_reader.py        lit le GELLO en série, relaie les lignes BRUTES (aucun calcul, voir plus bas)
  gello_calibration.json obsolète/sans usage -- la calibration se charge côté robot maintenant
robot/
  robot_agent.py         base mecanum : applique les commandes + watchdog LOCAL
  arm_agent.py            bras B601 : commandes en WebSocket direct (port 8767, PAS Zenoh), process séparé (env conda lerobot)
  mast_serial_bridge.py   mât (chariot pas-à-pas) : pont série Arduino <-> Zenoh (robot/mast/cmd|state|event|link)
  uvc_camera_server.py    capture caméra générique + serveur WebSocket multi-caméras
  camera_pub.py           config des 2 caméras -> JPEG -> WebSocket direct navigateur (port 8765, PAS Zenoh)
firmware/
  mast/                  firmware Arduino (PlatformIO) du chariot du mât -- voir firmware/mast/README.md
scripts/
  start_operator.sh      lance zenohd + web_server + input_agent
  start_robot.sh         lance robot_agent + camera_pub + mast_serial_bridge.py (+ arm_agent)
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
`eclipse-zenoh` (+ `pyserial` pour `mast_serial_bridge.py`) par-dessus :

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install eclipse-zenoh pyserial
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
GELLO_PORT=/dev/tty.usbserial-XXXX ROBOT_IP=192.168.15.107 scripts/start_operator.sh   # zenohd + web_server + input_agent

# PC robot
OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # base + caméra(s) + bras (défaut)
OPERATOR_IP=192.168.15.106 scripts/start_arm.sh     # bras seul -- voir section dédiée, séparé exprès
```

`start_robot.sh` a des flags d'opt-out, pour quand un adaptateur CAN est
débranché/en panne et que tu veux quand même le reste de la stack au lieu
d'être bloqué par les vérifications fail-fast. `camera_pub.py` tourne dans
tous les cas, y compris ces modes (pas besoin d'OPERATOR_IP) :

```bash
NO_ARM=1      OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # base + caméra(s) + mât, pas de bras
NO_BASE=1     OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # bras + caméra(s) + mât, pas de base (ex: CAN de la base en panne, tester le bras seul)
NO_MAST=1     OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # base + bras + caméra(s), pas de mât (ex: bridge Arduino débranché)
CAMERA_ONLY=1 scripts/start_robot.sh                              # caméra(s) seules (pas d'OPERATOR_IP requis)
```

`camera_pub.py` découvre automatiquement toutes les caméras UVC branchées
sur le PC robot (aucune configuration requise) -- voir plus bas. Quelle
caméra détectée s'affiche dans le grand cadre vs. la vignette se choisit
côté opérateur, dans **Réglages (⚙) > Caméras**.

La page opérateur se connecte à la connexion caméra partagée et au lien
GELLO direct (ports 8765/8767) via l'IP du robot, résolue par
`operator/web/static/js/robotIp.js` dans cet ordre : `?robotIp=<ip-robot>`
dans l'URL (override ponctuel) > **Réglages (⚙) > Caméras > IP du robot**
(mémorisée en local sur cette machine/ce navigateur, recommandé) > défaut
`169.254.222.31`. Sur une machine opérateur neuve (ou si l'IP du robot a
changé), renseigne-la une fois dans Réglages plutôt que de trimballer
`?robotIp=` dans chaque URL/onglet.

`GELLO_PORT` est optionnel : sans lui, `input_agent.py` tourne normalement
(base + web) mais le bras GELLO reste désactivé (`read_gello_raw_line()`
renvoie toujours `None`).

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

`camera_pub.py` de ce dépôt découvre **automatiquement** toutes les
caméras UVC branchées sur le PC robot et les sert directement au
navigateur (`ws://<ip-robot>:8765`, pas de Zenoh sur ce chemin — n'a donc
pas besoin de `OPERATOR_IP`). Deux pièges rencontrés sur la caméra avant
de ce robot (HSTD USB3.0), tous deux déjà corrigés dans le code
(`robot/uvc_camera_server.py`) : ne pas forcer `CAP_PROP_FPS` via
`cap.set()` (casse la négociation de la pipeline GStreamer sur cette
caméra à cette résolution), et l'index `/dev/videoN` n'est pas fiable d'un
boot à l'autre (renumérotation USB) — la découverte sonde et retient
automatiquement chaque index qui délivre une vraie frame.

**Découverte dynamique** (2026-07-10, `CameraManager` dans
`robot/uvc_camera_server.py`) : un unique thread de découverte sonde
toutes les quelques secondes (`discover_every_sec`, 3s par défaut) les
index `/dev/videoN` pas encore réclamés par une caméra déjà active, et
promeut automatiquement en caméra diffusée tout index qui s'ouvre **et**
délivre une vraie frame -- pas de redémarrage, pas de variable d'env à
configurer : brancher une seconde (ou une N-ième) caméra USB la fait
apparaître d'elle-même. Comme un seul thread sonde les index les uns après
les autres (contrairement à l'ancien design où chaque caméra avait son
propre thread d'auto-sondage), deux caméras ne peuvent plus se
"voler" silencieusement leur index l'une à l'autre -- ce risque de course
est ce qui forçait l'ancien réglage manuel `SECOND_CAMERA_ID`/
`SECOND_NAME_FILTER`, devenu inutile et supprimé. Une caméra débranchée
(plus aucune frame pendant `lost_after_sec`, 5s par défaut) est retirée de
la liste automatiquement, ce qui libère son index pour une redécouverte si
elle est rebranchée.

**Quelle caméra joue quel rôle** (grand cadre "principale" vs. vignette
"secondaire") est un réglage **côté opérateur**, pas robot : le serveur
envoie la liste des caméras détectées (id = index `/dev/videoN`, nom V4L2,
résolution négociée) sur la même connexion WebSocket que la vidéo, sous
forme d'un message JSON à part (`operator/web/static/js/videoMux.js`) --
**Réglages (⚙) > Caméras** dans la page opérateur liste les caméras
détectées par leur nom et laisse choisir laquelle est "principale" et
laquelle est "secondaire" (ou "Aucune" pour masquer la vignette).
`operator/web/static/js/cameraRoles.js` fait la résolution (préférence
sauvegardée -> retombe sur "Auto", la plus petite id disponible, si la
caméra choisie a été débranchée). Résolution/qualité JPEG de capture
(`CAMERA_WIDTH`/`CAMERA_HEIGHT`/`CAMERA_JPEG_QUALITY`, variables d'env
optionnelles de `camera_pub.py`, appliquées à toutes les caméras
indifféremment) sont juste un point de départ -- le bloc "Camera
configuration" du log affiche ce qui a été réellement négocié par chaque
caméra.

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
GELLO (série, RAW firmware)                    B601 (CAN, PC robot)
      |  lecture série BRUTE, aucun calcul cote navigateur/input_agent
      v
navigateur (gello.js -> armLink.js) ou input_agent.py (gello_reader.py -> ArmLink)
      |  WebSocket direct ws://<ip-robot>:8767, {"raw": "<ligne firmware>"} (PAS Zenoh)
      v
arm_agent.py -> start_teleoperationV2.run_teleoperation() (boucle de
      start_teleoperation.py, inchangée) -> follower
```

Les lignes série **brutes et non interprétées** du GELLO voyagent en
**WebSocket direct** navigateur/input_agent.py <-> `arm_agent.py`, comme la
caméra -- `robot/cmd/arm` (Zenoh) n'existe plus. `robot/cmd/stop` et
`robot/cmd/reset` (E-stop / réarmement) restent en revanche sur Zenoh,
**inchangés** : ils sont partagés avec la base, et les déplacer aurait
découplé cet arrêt d'urgence commun.

**Historique (2026-07-09/10)** — deux versions précédentes de `arm_agent.py`
ne fonctionnaient pas correctement en usage réel, malgré des tests
unitaires qui passaient :
1. une réimplémentation JS/Python de la calibration GELLO, qui portait en
   fait la MAUVAISE classe lerobot (`GelloAs5600Leader` au lieu de
   `GelloAs5600RawLeader` — firmware différent, pas de dépliage de la
   couture 0/360°) ;
2. après correction de (1), une version qui appelait quand même la bonne
   classe mais **sans jamais appeler `teleop.connect()`** (injection
   manuelle dans `teleop._raw_angles`, `teleop.leader_smooth` implicite à
   0.15 au lieu du `1` par défaut du script de référence).

L'utilisateur a alors validé une base de référence en dehors de ce dépôt :

```bash
# Sur le PC où le GELLO est branché (ici via socat, IP/port a adapter) :
socat TCP-LISTEN:9999,reuseaddr,fork OPEN:/dev/cu.usbserial-2130,raw,ispeed=115200,ospeed=115200,echo=0
# Sur le PC robot :
python start_teleoperation.py --teleop-port socket://<ip-pc-gello>:9999
```

Le script de référence **non modifié**, alimenté par un relais socat
TCP<->série. `robot/start_teleoperationV2.py` (dans ce dépôt, PAS dans
`~/03_JelloSoft/rebot_lerobot/scripts/` — copie unique et suivie par git,
pour ne jamais risquer un doublon qui dérive en silence, comme
`operator/gello_calibration.json` avant lui) reproduit cette base
**exactement**, avec un seul changement structurel : `teleop.connect()`
compose vers un petit serveur TCP local (`SerialBridge`) que NOUS
alimentons depuis un WebSocket, au lieu de composer vers socat.
`socket://` est le MÊME mécanisme pyserial qu'un relais socat -- vérifié
directement (`SerialBridge` + `serial.serial_for_url("socket://...")`
livre des lignes identiques bit à bit) -- donc `connect()` et son
`_reader_loop()` tournent réellement, comme avec socat. La boucle de
téléopération elle-même (`obs -> teleop.get_action() ->
teleop_action_processor -> robot_action_processor -> send_action()`) est
celle de `start_teleoperation.py`, non modifiée.

`arm_agent.py` ne fait plus AUCUNE logique de téléopération lui-même : il
importe `start_teleoperationV2.run_teleoperation()` et l'enrobe juste du
nécessaire pour l'E-stop/reset Zenoh (partagés avec la base) et le
heartbeat `robot/arm/state`, via trois hooks optionnels de
`run_teleoperation()` (`stop_event`, `on_tick`, `on_ready`) qui ne
modifient pas sa boucle -- ils sont des no-op quand absents, donc lancer
`start_teleoperationV2.py` seul (sans arm_agent.py) se comporte comme le
script de référence.

**Pourquoi pas la même connexion que la caméra** : `arm_agent.py` doit
tourner sous l'env conda `lerobot` (RebotB601Follower/torch) alors que
`camera_pub.py` a besoin du `cv2` système (GStreamer). Confirmé
empiriquement (2026-07-09) : le `cv2` de l'env conda `lerobot` n'a **pas**
de support GStreamer -- le même problème que le opencv-python générique de
PyPI déjà documenté plus haut. Les deux ne peuvent donc pas partager un seul
process/une seule connexion sans casser l'un des deux ; `arm_agent.py` ouvre
donc sa propre connexion WebSocket sur le port 8767, avec la même
philosophie ("direct, pas de hop Zenoh") mais pas le même socket.

Points clés :

- **Deux process séparés côté robot**, volontairement : `arm_agent.py`
  importe le package `lerobot` complet (dépendances lourdes, dont torch) —
  hors de question de mélanger ça avec la boucle 100Hz déjà validée de
  `robot_agent.py` (base). Doit donc tourner avec le python de l'env conda
  `lerobot`, pas le `.venv` du projet (voir `scripts/start_arm.sh`,
  override `ARM_PYTHON=` si le chemin de l'env diffère). Cet env a
  maintenant aussi besoin du paquet `websockets`
  (`~/miniconda3/envs/lerobot/bin/pip install websockets`) -- absent par
  défaut, `arm_agent.py` ne démarrera pas sans.
- **`gello_reader.py` ne dépend pas de lerobot et ne fait plus aucun calcul** :
  juste un lecteur série minimal (pyserial) qui garde la dernière ligne
  brute reçue et la relaie telle quelle. Nécessite `GELLO_PORT` (variable
  d'env, ex. `/dev/tty.usbserial-XXXX` sur macOS) — sans elle, désactivé
  proprement (pas d'auto-probe : le mauvais port série n'est pas un risque
  à prendre). `input_agent.py` a aussi besoin de `ROBOT_IP` (IP directe du
  PC robot, pas routée par Zenoh) pour joindre `arm_agent.py` -- non requis
  tant qu'aucun GELLO n'est détecté (pilotage base seul au joystick reste
  possible sans). Même chose côté navigateur : `gello.js` ne fait plus
  aucun calcul, juste `readLoop()` -> `armLink.sendRawLine()`, à chaque
  ligne série (~60Hz, plus couplé à `control.rateHz` puisqu'il n'y a plus
  de filtre de lissage côté navigateur).
- **Gating indépendant de la base** : contrairement à `robot/cmd/base`, le
  bras n'est **pas** gaté par le homme-mort manette (piloter un bras 7 DOF
  demande les deux mains). L'E-stop (`stop_event`, mis à jour par
  `robot/cmd/stop`/`robot/cmd/reset`) coupe l'envoi de nouvelles cibles et
  appelle `disable_torque()` une fois (edge-triggered) ; contrairement aux
  versions précédentes, **il n'y a plus de watchdog de fraîcheur sur les
  données GELLO elles-mêmes** -- `start_teleoperation.py` n'en a pas non
  plus, et c'est la configuration confirmée fonctionner par l'utilisateur.
  `robot/cmd/stop`/`robot/cmd/reset` restent **partagés** avec la base : un
  seul arrêt d'urgence coupe les deux.
- **Pas de `max_relative_target`** non plus (le filet de sécurité logiciel
  des versions précédentes) : `start_teleoperation.py` n'en configure pas
  non plus. Facile à réintroduire (kwarg de `RebotB601FollowerRobotConfig`
  dans `start_teleoperationV2.py`) si besoin, mais volontairement absent
  pour l'instant plutôt que réintroduit en silence -- ça ne faisait pas
  partie de ce qui a été concrètement validé.

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
`settings.js`) exposé dans le panneau ⚙ du header, en cinq onglets :

- **Contrôle** : fréquence d'envoi des commandes (10–50 Hz), vitesse max au
  chargement, mémorisation de la vitesse, pas de la pince ;
- **Caméras** : quelle caméra détectée (par nom, découverte automatiquement
  côté robot — voir § Robot réel) s'affiche en grand cadre ("principale")
  et laquelle en vignette ("secondaire", ou "Aucune" pour la masquer) ;
  liste tenue à jour en direct par `videoMux.js`, aucun redémarrage requis
  quand une caméra est branchée/débranchée ;
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

### Vue spatiale (vue de dessus)

Overlay « radar » en haut à droite du panneau vidéo (agrandissable ⤢ ou
double-clic, masquable dans Réglages > Vue spatiale) : une représentation du
robot et de son environnement **vue de dessus**, composée dans le navigateur
à partir des flux existants — aucun capteur, process ou connexion en plus.
Module `operator/web/static/js/birdview.js`, deux techniques classiques de
la perception automobile :

- **IPM (Inverse Perspective Mapping)** : chaque image des caméras avant et
  arrière est reprojetée sur le plan du sol par un shader WebGL (projection
  inverse par pixel, corrigée en perspective) — la technique des « surround
  view » des voitures. La calibration (hauteur, inclinaison, FOV, déport de
  montage de chaque caméra) se règle dans **Réglages (⚙) > Vue spatiale** :
  c'est bon quand un carrelage/damier au sol reste droit et à la bonne
  taille dans la vue.
- **Dead-reckoning odométrique** : la pose (x, y, cap) est intégrée en
  continu à partir des vitesses **réellement appliquées** par la base,
  republiées par `robot_agent.py` dans `robot/state` (champ `vel` :
  cinématique mecanum inverse des consignes roues post-rampe — pas l'écho
  de la commande opérateur, qui ignorerait rampe/deadman/watchdog). Les
  deux échelles (m/s et rad/s à pleine commande) se calibrent une fois en
  chronométrant 1 m puis un tour sur place — la base est commandée en
  normalisé `[-1, 1]`, jamais en unités physiques (rayon de roue jamais
  mesuré fiablement, voir plus haut). Si le robot ne publie pas encore
  `vel`, la vue se replie sur la dernière commande émise par ce navigateur
  (badge « odom. navigateur » dans le HUD).

Les projections sol successives s'accumulent dans une **mosaïque monde
persistante** : l'environnement se peint autour du robot au fil du
déplacement, avec la trace du chemin parcouru. Un **fondu temporel**
réglable matérialise la confiance décroissante (l'odométrie dérive : une
zone vue il y a longtemps n'est plus garantie exacte) ; le bouton ⌫ remet
la mémoire et la pose à zéro. La caméra pince, non projetable au sol (elle
vise la zone de travail), s'affiche en **médaillon circulaire** accroché à
l'avant du glyphe robot. Le glyphe montre châssis + roues mecanum, flèche
de cap, vecteur vitesse instantané et arc de rotation ; le HUD affiche
x/y/cap et la hauteur du mât.

Deux modes (🧭) : **cap** (robot fixe au centre, l'avant en haut — comme un
GPS voiture) ou **carte** (nord en haut, panoramique à la souris, ⌖
recentre). Molette = zoom, persisté. Quelles caméras jouent les rôles
avant / arrière / pince se choisit dans Réglages > Vue spatiale (défaut :
suivre les rôles principale / secondaire / tertiaire).

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

## Mât (chariot pas-à-pas)

Chariot vertical sur vis sans fin, motorisé par un moteur pas-à-pas NEMA23
boucle fermée (driver CL57T, frein électromagnétique 24 V, fins de course
haut/bas), piloté par un firmware Arduino dédié (`firmware/mast/`,
PlatformIO) relié en série au PC robot.

```text
[navigateur]  bouton Monter/Descendre (maintenu) ou touches PgUp/PgDn
      |  /ws/control -> web_server.py -> Zenoh (robot/mast/cmd)
      v
[PC robot]  robot/mast_serial_bridge.py  --série 115200 bauds-->  [Arduino, firmware/mast/]
      ^  robot/mast/state (position_mm, fdc_min/max) + robot/mast/link
      |
[navigateur]  tuile "Mât" (/ws/status)
```

`mast_serial_bridge.py` est un process robot **séparé** de plus (comme
`arm_agent.py`), lancé par `scripts/start_robot.sh` (flag `NO_MAST=1` pour
l'exclure) — même contrat `OPERATOR_IP` que `robot_agent.py`/`arm_agent.py`
(voir `robot/zenoh_config.py`). Il traduit les commandes JSON de
`robot/mast/cmd` en lignes série pour le firmware et republie sa télémétrie
sur `robot/mast/state`/`robot/mast/link` (relayées par `web_server.py` vers
la tuile "Mât" de la page) ; `robot/mast/event` (acquittements bruts du
firmware) reste un flux de debug CLI, pas relayé au navigateur.

Le front envoie une commande `{"action":"velocity","mm_s":±30}` en continu
(≥10 Hz, à la fréquence de `control.rateHz`) tant que le bouton Monter/
Descendre (ou `PgUp`/`PgDn`) est maintenu **et** que le homme-mort est actif
— **le même homme-mort que la base**, contrairement au bras (qui en est
volontairement indépendant, voir plus haut) : le mât est un actionneur
vertical, la sémantique "maintenir le homme-mort + maintenir la direction"
reproduit exactement le pavé directionnel de la base. Un `mm_s:0` explicite
est envoyé une seule fois au relâchement (pas de réémission continue à
l'arrêt, contrairement à `vx`/`vy`/`wz`). Le firmware a son propre watchdog
homme-mort (300 ms sans nouveau `VEL:` → arrêt + frein serré), **indépendant**
de celui de `robot_agent.py` : le mât s'arrête tout seul même si
`web_server.py`/`robot_agent.py` sont down, tant que le lien série
Arduino ↔ PC robot tient.

Protocole complet (position absolue, homing, jog, frein, fins de course,
dépannage) : voir `firmware/mast/README.md`.

## Topics Zenoh

| Clé                          | Sens | Contenu                                   |
| ---------------------------- | ---- | ----------------------------------------- |
| `robot/cmd/base`             | ->   | `{"vx","vy","wz"}` normalisés `[-1, 1]`   |
| `robot/cmd/stop`             | ->   | arrêt d'urgence (latch) — partagé base + bras |
| `robot/cmd/reset`            | ->   | lève le verrou E-stop + réactive les moteurs (deadman toujours requis pour bouger la base) — partagé base + bras |
| `operator/deadman`           | ->   | `"true"` / `"false"` (base uniquement, pas le bras) |
| `robot/heartbeat`            | <-   | vivacité robot base (~5 Hz)               |
| `robot/state`                | <-   | `{moving, estop, deadman_ok, fresh_cmd, vel, ts}` (base) — `vel` = `{vx, vy, wz}` normalisés réellement appliqués (post-rampe, mecanum inverse), pour la vue spatiale |
| `robot/arm/state`            | <-   | `{connected, moving, fresh_cmd, estop, joints, ts}` (bras) |
| `robot/mast/cmd`             | ->   | commande JSON mât, ex. `{"action":"velocity","mm_s":30}` — voir `firmware/mast/README.md` |
| `robot/mast/state`           | <-   | `{"position_mm","fdc_min","fdc_max","t"}` (mât, ~60 Hz) |
| `robot/mast/link`            | <-   | `"Connected"` / `"Disconnected"` (liaison série Arduino du mât) |
| `robot/mast/event`           | <-   | acquittements firmware mât (`ACK`/`MSG:`/`WARN:`/`ERR:`) — debug CLI, pas relayé au navigateur |

Les caméras ne sont **pas** des topics Zenoh : `camera_pub.py` sert le JPEG
des deux caméras directement au navigateur sur une seule connexion
WebSocket (`ws://<ip-robot>:8765`), voir [Structure](#structure) et le
diagramme en tête de fichier. Les commandes de position du bras non plus :
`arm_agent.py` les reçoit directement en WebSocket (`ws://<ip-robot>:8767`,
connexion séparée de celle des caméras -- voir [Bras GELLO -> reBot
B601](#bras-gello--rebot-b601-leader-follower) pour pourquoi). Seuls
`robot/cmd/stop`/`robot/cmd/reset`/`robot/arm/state` restent sur Zenoh pour
le bras.

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

Le bras **n'a pas** de watchdog de fraîcheur sur les données GELLO
elles-mêmes (contrairement à la base) : `arm_agent.py` délègue toute sa
boucle à `start_teleoperationV2.run_teleoperation()`, qui reproduit
`start_teleoperation.py` (le script de référence confirmé fonctionner sur
le matériel réel) exactement, et celui-ci n'en a pas non plus. Seul
`robot/cmd/stop` (Zenoh, partagé avec la base) coupe le bras
(`disable_torque()`, edge-triggered) ; `robot/cmd/reset` le réarme
(`configure()`). Détails : [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower).

Le mât, lui, a son watchdog homme-mort **sur le firmware Arduino
lui-même** (300 ms sans nouveau `VEL:` reçu en série -> arrêt + frein serré),
indépendant du watchdog de `robot_agent.py` : il s'arrête tout seul même si
`web_server.py`/`robot_agent.py` sont down, tant que la liaison série
Arduino ↔ PC robot est vivante. `robot/cmd/stop` (E-stop partagé base/bras)
**ne coupe pas** le mât — voir [Mât (chariot pas-à-pas)](#mât-chariot-pas-à-pas)
et `firmware/mast/README.md` §5 pour le détail.

## Notes / limites

- `apply_arm_command`/`apply_gripper_command` dans `robot_agent.py` restent
  des no-op **volontaires** : le bras est piloté par le process séparé
  `arm_agent.py` (voir [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower)),
  pas par `robot_agent.py`.
- Caméras : JPEG de toutes les caméras découvertes automatiquement
  (voir § Robot réel plus haut), servi directement au navigateur par
  `camera_pub.py` sur une seule connexion WebSocket (`:8765`), plus de hop
  Zenoh ni web_server.py sur ce chemin — Zenoh garde les commandes,
  l'état, le heartbeat et la supervision. Rôle principale/secondaire
  choisi côté opérateur (Réglages > Caméras), pas figé côté robot.

## Plan de réalisation

1. Ping Zenoh entre les deux PC
2. Joystick -> réception robot
3. Watchdog + stop robot
4. Caméra JPEG
5. Serveur web + affichage caméra
6. ~~GELLO -> `robot/cmd/arm`~~ fait — voir [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower)
7. Limites de vitesse + deadman + arrêt d'urgence
8. ~~Mât (chariot pas-à-pas) -> `robot/mast/cmd`~~ fait — voir [Mât (chariot pas-à-pas)](#mât-chariot-pas-à-pas)
