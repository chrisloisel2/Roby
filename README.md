# Roby

Téléopération d'un robot (base mobile + bras) via **Zenoh**, avec joystick,
GELLO, caméra et interface web. Deux PC : opérateur et robot.

```text
[Joystick + GELLO]
        v
[PC opérateur]  input_agent.py · web_server.py · zenohd
        |  Zenoh TCP (base/bras/état)      ^  WebSocket direct (2 caméras, 1 connexion, port 8765)
        v                                  |
[PC robot]      robot_agent.py (base) · arm_agent.py (bras) · camera_pub.py
        v
[Robot + bras + 2 caméras]
```

Seuls base/bras/état passent par Zenoh + web_server.py. Les DEUX caméras
sont servies par `camera_pub.py` sur une seule **connexion WebSocket
directe** navigateur <-> robot (pas de hop Zenoh ni web_server.py sur ce
chemin) — voir [Structure](#structure) et la table des clés Zenoh plus bas.
Une seule connexion pour les deux flux, volontairement (pas deux sockets
séparés) : les deux caméras reçoivent alors exactement le même traitement
réseau (même connexion TCP, même fenêtre d'écriture, même ordonnancement
asyncio) au lieu de deux sockets indépendants qui pourraient chacun dériver
à leur rythme. Chaque message est `[1 octet cam_id][JPEG]` ; le navigateur
démultiplexe par ce préfixe (`operator/web/static/js/videoMux.js`).

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
  web_server.py          pont Zenoh <-> navigateur pour base/état/stop-reset (FastAPI + WebSocket) -- PAS la caméra ni les commandes du bras
  web/
    index.html           UI opérateur (markup seul)
    static/app.css       design system (thème sombre poste de pilotage)
    static/js/           modules ES : config (store central), net, videoMux
                         (connexion caméra partagée), camera, camera2,
                         armLink (WebSocket direct bras), status, control,
                         joystick, gello, settings, main
  gello_reader.py        lit le GELLO en série, produit des angles calibrés follower
  gello_calibration.json calibration GELLO déjà mesurée (copie de mon_gello.json)
robot/
  robot_agent.py         base mecanum : applique les commandes + watchdog LOCAL
  arm_agent.py            bras B601 : commandes en WebSocket direct (port 8767, PAS Zenoh), process séparé (env conda lerobot)
  uvc_camera_server.py    capture caméra générique + serveur WebSocket multi-caméras
  camera_pub.py           config des 2 caméras -> JPEG -> WebSocket direct navigateur (port 8765, PAS Zenoh)
scripts/
  start_operator.sh      lance zenohd + web_server + input_agent
  start_robot.sh         lance robot_agent + camera_pub (+ arm_agent)
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
GELLO_PORT=/dev/tty.usbserial-XXXX ROBOT_IP=192.168.15.107 scripts/start_operator.sh   # zenohd + web_server + input_agent

# PC robot
OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # base + caméra(s) + bras (défaut)
OPERATOR_IP=192.168.15.106 scripts/start_arm.sh     # bras seul -- voir section dédiée, séparé exprès
```

`start_robot.sh` a deux flags d'opt-out, pour quand un adaptateur CAN est
débranché/en panne et que tu veux quand même le reste de la stack au lieu
d'être bloqué par les vérifications fail-fast. `camera_pub.py` tourne dans
tous les cas, y compris ces deux modes (pas besoin d'OPERATOR_IP) :

```bash
NO_ARM=1      OPERATOR_IP=192.168.15.106 scripts/start_robot.sh   # base + caméra(s), pas de bras
CAMERA_ONLY=1 scripts/start_robot.sh                              # caméra(s) seules (pas d'OPERATOR_IP requis)
```

`camera_pub.py` sert par défaut la caméra avant seule ; la seconde caméra
ne démarre que si `SECOND_CAMERA_ID` ou `SECOND_NAME_FILTER` est configuré
(voir plus bas) -- sinon un message dans `logs/camera_pub.log` te le
rappelle à chaque démarrage.

La page opérateur se connecte à la connexion caméra partagée via
`?robotIp=<ip-robot>` dans l'URL (port unique 8765 pour les deux flux --
défaut `169.254.222.31`, voir `DEFAULT_ROBOT_IP` dans
`operator/web/static/js/videoMux.js`) — à ajuster si l'IP du robot diffère.

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

`camera_pub.py` de ce dépôt fonctionne avec la caméra USB avant branchée
sur ce robot (HSTD USB3.0), capturée en 1920x1200 et servie directement au
navigateur (`ws://<ip-robot>:8765`, pas de Zenoh sur ce chemin — n'a donc
pas besoin de `OPERATOR_IP`). Deux pièges rencontrés, tous deux déjà
corrigés dans le code (`robot/uvc_camera_server.py`) : ne pas forcer
`CAP_PROP_FPS` via `cap.set()` (casse la négociation de la pipeline
GStreamer sur cette caméra à cette résolution), et l'index `/dev/videoN`
n'est pas fiable d'un boot à l'autre (renumérotation USB) — chaque caméra
sonde et retient automatiquement le premier index qui délivre une vraie
frame.

Une **seconde caméra USB** générique (n'importe quel UVC standard) peut
être branchée sur le même PC robot et servie sur la même connexion,
multiplexée avec la caméra avant (voir le diagramme en tête de fichier).
Comme les deux caméras partagent le même sondage d'index, `camera_pub.py`
**refuse par défaut de démarrer la seconde en sondage non filtré** (une
course entre les deux threads de capture pourrait leur faire échanger
silencieusement leurs caméras d'un lancement à l'autre) -- tant que
`SECOND_CAMERA_ID` ou `SECOND_NAME_FILTER` n'est pas configuré, seule la
caméra avant tourne, avec un rappel dans `logs/camera_pub.log` à chaque
démarrage. Pour l'activer : branche la seconde caméra, relance, regarde les
lignes `probe /dev/videoN` du log (elles listent le vrai nom V4L2 de
**chaque** index, y compris ceux qu'elle ignore -- ce nom n'est PAS le même
que celui affiché par `lsusb`, confirmé empiriquement le 2026-07-09), puis
règle dans `robot/camera_pub.py` soit `SECOND_NAME_FILTER` (si le nom
diffère de celui de la caméra avant) soit directement
`SECOND_CAMERA_ID=<index>` (si les deux caméras partagent un nom
générique identique -- le sondage par nom ne peut alors pas les
distinguer). Résolution et qualité JPEG de la seconde caméra dans
`camera_pub.py` sont un point de départ -- le bloc "Camera configuration"
du log affiche ce qui a été réellement négocié.

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
GELLO (série)                                  B601 (CAN, PC robot)
      |  lecture + calibration déjà faite
      v
navigateur (armLink.js) ou input_agent.py (ArmLink)
      |  WebSocket direct ws://<ip-robot>:8767 (PAS Zenoh)
      v
arm_agent.py --send_action()--> follower
```

Les commandes de position articulaire (joints + gripper) voyagent en
**WebSocket direct** navigateur/input_agent.py <-> `arm_agent.py`, comme la
caméra -- `robot/cmd/arm` (Zenoh) n'existe plus. `robot/cmd/stop` et
`robot/cmd/reset` (E-stop / réarmement) restent en revanche sur Zenoh,
**inchangés** : ils sont partagés avec la base, et les déplacer aurait
découplé cet arrêt d'urgence commun. `arm_agent.py` construit le follower
via `make_robot_from_config()`, comme
`~/03_JelloSoft/rebot_lerobot/scripts/start_teleoperation.py` (le script de
téléop de référence sur ce matériel).

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
  réutilise `RebotB601Follower` tel quel, qui importe le package `lerobot`
  complet (dépendances lourdes, dont torch) — hors de question de mélanger
  ça avec la boucle 100Hz déjà validée de `robot_agent.py` (base). Doit donc
  tourner avec le python de l'env conda `lerobot`, pas le `.venv` du projet
  (voir `scripts/start_arm.sh`, override `ARM_PYTHON=` si le chemin de l'env
  diffère). Cet env a maintenant aussi besoin du paquet `websockets`
  (`~/miniconda3/envs/lerobot/bin/pip install websockets`) -- absent par
  défaut, `arm_agent.py` ne démarrera pas sans.
- **`gello_reader.py` ne dépend pas de lerobot** : réimplémentation autonome
  (pyserial) de la lecture série + de la formule de calibration de
  `GelloAs5600Leader.get_action()` (lissage -> clip aux butées -> sens ->
  échelle -> offset), avec les mêmes constantes et le même fichier de
  calibration déjà mesuré (`operator/gello_calibration.json`, copie de
  `mon_gello.json`). Nécessite `GELLO_PORT` (variable d'env, ex.
  `/dev/tty.usbserial-XXXX` sur macOS) — sans elle, désactivé proprement (pas
  d'auto-probe : le mauvais port série n'est pas un risque à prendre).
  `input_agent.py` a aussi besoin de `ROBOT_IP` (IP directe du PC robot, pas
  routée par Zenoh) pour joindre `arm_agent.py` -- non requis tant qu'aucun
  GELLO n'est détecté (pilotage base seul au joystick reste possible sans).
- **Gating indépendant de la base** : contrairement à `robot/cmd/base`, le
  bras n'est **pas** gaté par le homme-mort manette (piloter un bras 7 DOF
  demande les deux mains). `arm_agent.py` a son propre watchdog de fraîcheur
  (`ARM_CMD_TIMEOUT_SEC`), maintenant basé sur la dernière commande reçue
  par WebSocket : sans commande fraîche, il arrête juste d'envoyer de
  nouvelles cibles (les moteurs Damiao tiennent seuls leur dernière consigne
  — rien d'équivalent au ramp-to-zero de la base n'est nécessaire).
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
| `robot/cmd/stop`             | ->   | arrêt d'urgence (latch) — partagé base + bras |
| `robot/cmd/reset`            | ->   | lève le verrou E-stop + réactive les moteurs (deadman toujours requis pour bouger la base) — partagé base + bras |
| `operator/deadman`           | ->   | `"true"` / `"false"` (base uniquement, pas le bras) |
| `robot/heartbeat`            | <-   | vivacité robot base (~5 Hz)               |
| `robot/state`                | <-   | `{moving, estop, deadman_ok, fresh_cmd, ts}` (base) |
| `robot/arm/state`            | <-   | `{connected, moving, fresh_cmd, estop, joints, ts}` (bras) |

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

Le bras a son **propre watchdog local**, dans `arm_agent.py`, indépendant de
celui de `robot_agent.py` (process séparé) : pas de nouvelle consigne
envoyée sans commande WebSocket fraîche (`ARM_CMD_TIMEOUT_SEC`), plus
`max_relative_target` comme filet en plus du lissage déjà fait côté GELLO.
`robot/cmd/stop` coupe aussi le bras (`disable_torque()`) ; `robot/cmd/reset`
le réarme. Détails : [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower).

## Notes / limites

- `apply_arm_command`/`apply_gripper_command` dans `robot_agent.py` restent
  des no-op **volontaires** : le bras est piloté par le process séparé
  `arm_agent.py` (voir [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower)),
  pas par `robot_agent.py`.
- Caméras : JPEG des deux caméras (avant 1920×1200 + seconde optionnelle)
  servi directement au navigateur par `camera_pub.py` sur une seule
  connexion WebSocket (`:8765`), plus de hop Zenoh ni web_server.py sur ce
  chemin — Zenoh garde les commandes, l'état, le heartbeat et la
  supervision.

## Plan de réalisation

1. Ping Zenoh entre les deux PC
2. Joystick -> réception robot
3. Watchdog + stop robot
4. Caméra JPEG
5. Serveur web + affichage caméra
6. ~~GELLO -> `robot/cmd/arm`~~ fait — voir [Bras GELLO -> reBot B601](#bras-gello--rebot-b601-leader-follower)
7. Limites de vitesse + deadman + arrêt d'urgence
