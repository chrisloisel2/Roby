// Picture-in-picture camera stream: renders whichever discovered camera is
// assigned a secondary UI role into a small PiP thumbnail with no fps/age/
// fit/fullscreen controls (just detach, see popoutCanvas.js) -- deliberately
// NOT sharing camera.js's code, since that divergence in UI role (fps/age/
// fit/fullscreen only make sense for the one big tile) would mean a pile of
// unused options on one side or the other.
//
// One factory (initPipCamera), not one file per PiP box: up to two
// simultaneous PiP slots (index.html's videoPip/videoPip3, "secondary"/
// "tertiary" in cameraRoles.js) are structurally identical -- only which DOM
// ids and config.js paths they bind to differs -- so main.js instantiates
// this factory twice instead of maintaining a second near-duplicate file.
//
// Frames arrive over the SAME shared WebSocket connection as camera.js
// (videoMux.js), demuxed by cam_id -- see that file for why one connection,
// not two: both cameras get identical connection-level treatment (latency,
// backpressure, scheduling) instead of drifting apart independently. Which
// cam_id a given instance shows is resolved dynamically by the caller
// (resolveTargetId, built from cameraRoles.js in main.js) against the
// robot's live camera list (uvc_camera_server.py's CameraManager) -- the PiP
// box hides itself when that resolves to nothing (camera unplugged, or the
// operator explicitly picked "aucune" in Réglages > Caméras), since that's
// now an ordinary, expected state rather than an error condition.

import { config } from "./config.js";
import { createPopout, rafOn } from "./popoutCanvas.js";

export function initPipCamera({ mux, ids, resolveTargetId, rotateConfigPath, defaultName, popoutTitle }) {
	const cam = document.getElementById(ids.canvas);
	const noSignal = document.getElementById(ids.noSignal);
	const detachedOverlay = document.getElementById(ids.detachedOverlay);
	const btnDetach = document.getElementById(ids.btnDetach);
	const container = document.getElementById(ids.container);
	const camNameEl = document.getElementById(ids.name);
	if (!cam || !noSignal) return null;
	const ctx = cam.getContext("2d", { alpha: false });
	const popout = createPopout({ title: popoutTitle, ctxOptions: { alpha: false } });

	let lastFrame = 0;
	let latestData = null, dataSeq = 0, displayedSeq = -1, rendering = false;
	let targetId = null;
	// Caméra montée à l'envers -> retourner l'image de 180° au dessin, voir
	// camera.js pour le même mécanisme sur la caméra principale.
	let rotated180 = false;
	const applyRotation = () => { rotated180 = !!config.get(rotateConfigPath); };
	applyRotation();

	function recomputeTarget() {
		const cameras = mux.getCameras();
		targetId = resolveTargetId(cameras);
		if (container) container.hidden = targetId === null;
		if (camNameEl) {
			const found = cameras.find((c) => c.id === targetId);
			camNameEl.textContent = found ? found.name : defaultName;
		}
	}
	mux.onCameraList(recomputeTarget);
	config.subscribe((path) => {
		if (path.startsWith("cameras.")) recomputeTarget();
		if (path === rotateConfigPath) applyRotation();
	});

	mux.onAnyFrame((camId, jpegBytes) => {
		if (camId !== targetId) return;
		latestData = jpegBytes;
		dataSeq++;
		lastFrame = performance.now();
	});

	async function renderLoop() {
		if (dataSeq !== displayedSeq && latestData && !rendering) {
			displayedSeq = dataSeq;
			rendering = true;
			try {
				const bitmap = await createImageBitmap(new Blob([latestData], { type: "image/jpeg" }));
				const detached = popout.isOpen();
				const targetCanvas = detached ? popout.getCanvas() : cam;
				const targetCtx = detached ? popout.getCtx() : ctx;
				if (targetCanvas.width !== bitmap.width || targetCanvas.height !== bitmap.height) {
					targetCanvas.width = bitmap.width;
					targetCanvas.height = bitmap.height;
				}
				if (rotated180) {
					targetCtx.save();
					targetCtx.translate(targetCanvas.width, targetCanvas.height);
					targetCtx.rotate(Math.PI);
					targetCtx.drawImage(bitmap, 0, 0);
					targetCtx.restore();
				} else {
					targetCtx.drawImage(bitmap, 0, 0);
				}
				bitmap.close();
			} finally {
				rendering = false;
			}
		}
		rafOn(popout, renderLoop);
	}
	rafOn(popout, renderLoop);

	setInterval(() => {
		const age = lastFrame ? performance.now() - lastFrame : Infinity;
		noSignal.classList.toggle("show", age > config.get("ui.staleMs") && !popout.isOpen());
	}, 250);

	// ---- Détacher sur un autre écran (popup fenêtre séparée) ----
	if (btnDetach) {
		popout.onChange((open) => {
			btnDetach.classList.toggle("active", open);
			btnDetach.title = open ? "Réattacher à cette page" : "Détacher sur un autre écran";
			if (detachedOverlay) detachedOverlay.classList.toggle("show", open);
			if (open) noSignal.classList.remove("show");
		});
		btnDetach.addEventListener("click", () => popout.toggle());
	}

	return { isAlive: mux.isAlive };
}
