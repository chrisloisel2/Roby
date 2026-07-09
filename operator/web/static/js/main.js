// Entry point: wires the modules together.
//
//   config.js    central settings store (localStorage, versioned, exportable)
//   net.js       auto-reconnecting WebSockets
//   videoMux.js  ws://<robot-ip>:8765 (direct to robot, ONE shared connection for both cameras)
//   camera.js    front camera (cam_id 0) -> canvas (latest-frame rendering)
//   camera2.js   second camera (cam_id 1) -> picture-in-picture canvas
//   armLink.js   ws://<robot-ip>:8767 (direct to robot/arm_agent.py) -> raw GELLO lines
//   status.js    /ws/status -> tiles, banner, arm joint gauges
//   control.js   keyboard/d-pad/deadman + the command loop -> /ws/control (base/stop/reset/gripper)
//   joystick.js  Gamepad API + dynamic mapping
//   gello.js     GELLO leader arm over Web Serial -> armLink (raw lines, its own read-loop rate)
//   settings.js  settings modal bound to config.js

import { config } from "./config.js";
import { createVideoMux } from "./videoMux.js";
import { initCamera } from "./camera.js";
import { initCamera2 } from "./camera2.js";
import { createArmLink } from "./armLink.js";
import { initStatus, setTile } from "./status.js";
import { initControl } from "./control.js";
import { initJoystick } from "./joystick.js";
import { initGello } from "./gello.js";
import { initSettings } from "./settings.js";

const $ = (id) => document.getElementById(id);

const videoMux = createVideoMux();
const camera = initCamera({ setTile, mux: videoMux });
initCamera2({ mux: videoMux });
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
initSettings();

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
