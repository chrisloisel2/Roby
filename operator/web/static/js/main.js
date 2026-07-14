// Entry point: wires the modules together.
//
//   config.js    central settings store (localStorage, versioned, exportable)
//   net.js       auto-reconnecting WebSockets
//   videoMux.js  ws://<robot-ip>:8765 (direct to robot, ONE shared connection for every camera,
//                auto-discovered robot-side -- see robot/uvc_camera_server.py's CameraManager)
//   cameraRoles.js  resolves which discovered camera plays the primary/secondary/tertiary UI
//                role, shared by camera.js/cameraPip.js (rendering) and settings.js (the picker)
//   camera.js    primary camera -> canvas (latest-frame rendering)
//   cameraPip.js secondary + tertiary cameras -> the two picture-in-picture canvases
//                (initPipCamera instantiated twice below; each hides itself if none assigned)
//   popoutCanvas.js  shared "detach to another screen" popup window helper,
//                used by both camera.js and cameraPip.js
//   armLink.js   ws://<robot-ip>:8767 (direct to robot/arm_agent.py) -> raw GELLO lines
//   status.js    /ws/status -> tiles, banner, arm joint gauges
//   birdview.js  vue spatiale (vue de dessus) : IPM WebGL des caméras avant/arrière
//                + dead-reckoning depuis robot/state.vel -> overlay radar du panneau vidéo
//   control.js   keyboard/d-pad/deadman + the command loop -> /ws/control (base/stop/reset/gripper)
//   joystick.js  Gamepad API + dynamic mapping
//   gello.js     GELLO leader arm over Web Serial -> armLink (raw lines, its own read-loop rate)
//   settings.js  settings modal bound to config.js (incl. the camera-role pickers, fed by videoMux.js)

import { config } from "./config.js";
import { createVideoMux } from "./videoMux.js";
import { initCamera } from "./camera.js";
import { initPipCamera } from "./cameraPip.js";
import { resolvePrimaryId, resolveSecondaryId, resolveTertiaryId } from "./cameraRoles.js";
import { createArmLink } from "./armLink.js";
import { initBirdview } from "./birdview.js";
import { initStatus, setTile } from "./status.js";
import { initControl } from "./control.js";
import { initJoystick } from "./joystick.js";
import { initGello } from "./gello.js";
import { initSettings } from "./settings.js";

const $ = (id) => document.getElementById(id);

const videoMux = createVideoMux();
const camera = initCamera({ setTile, mux: videoMux });
initPipCamera({
	mux: videoMux,
	ids: { canvas: "cam2", noSignal: "noSignal2", detachedOverlay: "detachedOverlay2", btnDetach: "btnDetach2", container: "videoPip", name: "camName2" },
	resolveTargetId: (cameras) => resolveSecondaryId(cameras, resolvePrimaryId(cameras)),
	rotateConfigPath: "cameras.secondaryRotate180",
	defaultName: "caméra secondaire",
	popoutTitle: "Roby — Caméra secondaire",
});
initPipCamera({
	mux: videoMux,
	ids: { canvas: "cam3", noSignal: "noSignal3", detachedOverlay: "detachedOverlay3", btnDetach: "btnDetach3", container: "videoPip3", name: "camName3" },
	resolveTargetId: (cameras) => {
		const primaryId = resolvePrimaryId(cameras);
		return resolveTertiaryId(cameras, primaryId, resolveSecondaryId(cameras, primaryId));
	},
	rotateConfigPath: "cameras.tertiaryRotate180",
	defaultName: "caméra tertiaire",
	popoutTitle: "Roby — Caméra tertiaire",
});
const armLink = createArmLink();
const status = initStatus();
const control = initControl({ onFullscreen: camera.toggleFullscreen });
const joystick = initJoystick({
	onStop: control.triggerStop,
	onReset: control.sendReset,
	onGripDelta: control.adjustGripper,
});
initGello({ armLink });
control.start({ joystick });
initSettings({ mux: videoMux });
initBirdview({ mux: videoMux, status, control });

// ---- Connection badge (server link) ----
setInterval(() => {
	const up = camera.isAlive() || status.isAlive() || control.isAlive();
	const c = $("conn");
	c.className = "conn " + (up ? "live" : "down");
	$("connText").textContent = up ? "serveur connecté" : "reconnexion…";
}, 500);

// ---- Clock ----
setInterval(() => {
	$("clock").textContent = new Date().toLocaleTimeString("fr-FR");
}, 250);

// ---- Panel visibility from config ----
const applyVisibility = () => {
	$("telePanel").hidden = !config.get("ui.showTelemetry");
};
applyVisibility();
config.subscribe((path) => { if (path === "ui.showTelemetry") applyVisibility(); });

// ---- Help modal ----
const helpModal = $("helpModal");
const toggleHelp = (show) => { helpModal.hidden = show === undefined ? !helpModal.hidden : !show; };
$("btnHelp").addEventListener("click", () => toggleHelp());
$("btnCloseHelp").addEventListener("click", () => toggleHelp(false));
helpModal.addEventListener("click", (e) => { if (e.target === helpModal) toggleHelp(false); });
document.addEventListener("keydown", (e) => {
	if (e.code === "Escape" && !helpModal.hidden) { toggleHelp(false); return; }
	if (e.key === "?" && !e.target.closest?.("input, select, textarea")) toggleHelp();
});
