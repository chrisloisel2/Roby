// Camera stream: binary JPEG frames over WebSocket, rendered to a canvas.
//
// Frames arrive over a SHARED WebSocket connection (videoMux.js, direct to
// robot/camera_pub.py -- not relayed through web_server.py's /ws/*, one
// fewer hop cuts real latency at this resolution) also used by camera2.js
// for the second camera -- see videoMux.js for why that's one connection,
// not two. This module just subscribes to its own cam_id.
//
// Receiving a frame and displaying it are deliberately decoupled: the mux
// callback just stashes the latest frame bytes (cheap), a
// requestAnimationFrame loop does the actual decode/paint work. If the main
// thread is ever busy (gamepad polling, other WS handlers, a GC pause) and
// several frames pile up in the WS event queue, the callback still drains
// them all in order -- but only the LAST one is ever rendered, so a
// momentary stall can no longer turn into a growing backlog of stale frames
// displayed late one after another (that was the actual cause of latency
// drifting from ~200ms up to ~0.5s instead of staying flat).

import { config } from "./config.js";
import { CAM_FRONT } from "./videoMux.js";

export function initCamera({ setTile, mux }) {
	const cam = document.getElementById("cam");
	const noSignal = document.getElementById("noSignal");
	const chipAge = document.getElementById("chipAge");
	const fpsEl = document.getElementById("fps");
	const ageEl = document.getElementById("age");
	const videoPanel = document.getElementById("videoPanel");
	const camCtx = cam.getContext("2d");

	let lastFrame = 0;
	const frameStamps = [];
	let latestCamData = null, camDataSeq = 0, displayedSeq = -1, renderingCam = false;

	mux.onFrame(CAM_FRONT, (jpegBytes) => {
		latestCamData = jpegBytes;
		camDataSeq++;
		const now = performance.now();
		lastFrame = now;
		frameStamps.push(now);
		if (frameStamps.length > 30) frameStamps.shift();
	});

	async function renderCamFrame() {
		if (camDataSeq !== displayedSeq && latestCamData && !renderingCam) {
			displayedSeq = camDataSeq;
			renderingCam = true;
			// Binary frames straight into createImageBitmap + canvas: skips both
			// the ~33% base64 size overhead of a data URI AND the Blob-URL
			// create/revoke bookkeeping an <img> would need.
			try {
				const bitmap = await createImageBitmap(new Blob([latestCamData], { type: "image/jpeg" }));
				if (cam.width !== bitmap.width || cam.height !== bitmap.height) {
					cam.width = bitmap.width;
					cam.height = bitmap.height;
				}
				camCtx.drawImage(bitmap, 0, 0);
				bitmap.close();
			} finally {
				renderingCam = false;
			}
		}
		requestAnimationFrame(renderCamFrame);
	}
	requestAnimationFrame(renderCamFrame);

	// ---- Freshness / FPS readouts (client-side clock) ----
	setInterval(() => {
		const now = performance.now();
		const age = lastFrame ? now - lastFrame : Infinity;
		const lost = age > config.get("ui.staleMs");
		noSignal.classList.toggle("show", lost);
		setTile("t-cam", lost ? "critical" : "good", lost ? "PERDUE" : "OK");
		ageEl.textContent = lost ? "—" : Math.round(age) + " ms";
		chipAge.dataset.q = lost || age >= 500 ? "critical" : age >= 200 ? "warning" : "";
		// fps over the retained window
		if (frameStamps.length > 1 && !lost) {
			const span = (frameStamps[frameStamps.length - 1] - frameStamps[0]) / 1000;
			const fps = span > 0 ? (frameStamps.length - 1) / span : 0;
			fpsEl.textContent = span > 0 ? (fps < 1 ? "<1" : Math.round(fps)) : "–";
		} else {
			fpsEl.textContent = "–";
		}
	}, 250);

	// ---- Video fit (contain / cover) ----
	const applyFit = () => { cam.style.objectFit = config.get("ui.videoFit"); };
	applyFit();
	config.subscribe((path) => { if (path === "ui.videoFit") applyFit(); });
	document.getElementById("btnFit").addEventListener("click", () => {
		config.set("ui.videoFit", config.get("ui.videoFit") === "contain" ? "cover" : "contain");
	});

	// ---- Fullscreen ----
	const toggleFullscreen = () => {
		if (document.fullscreenElement) document.exitFullscreen();
		else videoPanel.requestFullscreen && videoPanel.requestFullscreen();
	};
	document.getElementById("btnFullscreen").addEventListener("click", toggleFullscreen);
	cam.addEventListener("dblclick", toggleFullscreen);

	return { isAlive: mux.isAlive, toggleFullscreen };
}
