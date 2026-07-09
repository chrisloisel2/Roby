// Secondary camera stream: a second UVC webcam plugged into the robot PC,
// served by the same robot/camera_pub.py process as the front camera (see
// robot/uvc_camera_server.py). Rendered as a small picture-in-picture
// thumbnail with no fps/age/fit/fullscreen controls (just detach, see
// popoutCanvas.js) -- deliberately NOT sharing camera.js's code, since that
// divergence in UI role would mean a pile of unused options on one side or
// the other.
//
// Frames arrive over the SAME shared WebSocket connection as camera.js
// (videoMux.js), demuxed by cam_id -- see that file for why one connection,
// not two: both cameras get identical connection-level treatment (latency,
// backpressure, scheduling) instead of drifting apart independently.

import { config } from "./config.js";
import { CAM_SECOND } from "./videoMux.js";
import { createPopout, rafOn } from "./popoutCanvas.js";

export function initCamera2({ mux }) {
	const cam2 = document.getElementById("cam2");
	const noSignal2 = document.getElementById("noSignal2");
	const detachedOverlay2 = document.getElementById("detachedOverlay2");
	const btnDetach2 = document.getElementById("btnDetach2");
	if (!cam2 || !noSignal2) return null;
	const ctx = cam2.getContext("2d", { alpha: false });
	const popout = createPopout({ title: "Roby — Caméra 2", ctxOptions: { alpha: false } });

	let lastFrame = 0;
	let latestData = null, dataSeq = 0, displayedSeq = -1, rendering = false;

	mux.onFrame(CAM_SECOND, (jpegBytes) => {
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
				const targetCanvas = detached ? popout.getCanvas() : cam2;
				const targetCtx = detached ? popout.getCtx() : ctx;
				if (targetCanvas.width !== bitmap.width || targetCanvas.height !== bitmap.height) {
					targetCanvas.width = bitmap.width;
					targetCanvas.height = bitmap.height;
				}
				targetCtx.drawImage(bitmap, 0, 0);
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
		noSignal2.classList.toggle("show", age > config.get("ui.staleMs") && !popout.isOpen());
	}, 250);

	// ---- Détacher sur un autre écran (popup fenêtre séparée) ----
	if (btnDetach2) {
		popout.onChange((open) => {
			btnDetach2.classList.toggle("active", open);
			btnDetach2.title = open ? "Réattacher à cette page" : "Détacher sur un autre écran";
			if (detachedOverlay2) detachedOverlay2.classList.toggle("show", open);
			if (open) noSignal2.classList.remove("show");
		});
		btnDetach2.addEventListener("click", () => popout.toggle());
	}

	return { isAlive: mux.isAlive };
}
