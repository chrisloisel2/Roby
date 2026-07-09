// Camera stream: binary JPEG frames over WebSocket, rendered to a canvas.
//
// Frames arrive over a SHARED WebSocket connection (videoMux.js, direct to
// robot/camera_pub.py -- not relayed through web_server.py's /ws/*, one
// fewer hop cuts real latency at this resolution) also used by camera2.js
// for the second camera -- see videoMux.js for why that's one connection,
// not two. Which physical camera (cam_id) this tile shows is resolved
// dynamically (cameraRoles.js: config.js's `cameras.primaryId` preference
// against the robot's live, auto-discovered camera list) rather than a
// fixed id -- see videoMux.js/uvc_camera_server.py for why there's no more
// fixed "front" cam_id at all.
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
import { resolvePrimaryId } from "./cameraRoles.js";
import { createPopout, rafOn } from "./popoutCanvas.js";
import { toast } from "./toast.js";

export function initCamera({ setTile, mux }) {
	const cam = document.getElementById("cam");
	const noSignal = document.getElementById("noSignal");
	const detachedOverlay = document.getElementById("detachedOverlay");
	const btnDetach = document.getElementById("btnDetach");
	const chipAge = document.getElementById("chipAge");
	const fpsEl = document.getElementById("fps");
	const ageEl = document.getElementById("age");
	const camNameEl = document.getElementById("camName");
	const videoPanel = document.getElementById("videoPanel");
	const camCtx = cam.getContext("2d");
	const popout = createPopout({ title: "Roby — Caméra principale" });

	let lastFrame = 0;
	const frameStamps = [];
	let latestCamData = null, camDataSeq = 0, displayedSeq = -1, renderingCam = false;
	let targetId = null;

	function recomputeTarget() {
		const cameras = mux.getCameras();
		targetId = resolvePrimaryId(cameras);
		if (camNameEl) {
			const found = cameras.find((c) => c.id === targetId);
			camNameEl.textContent = found ? found.name : "caméra principale";
		}
	}
	mux.onCameraList(recomputeTarget);
	config.subscribe((path) => { if (path.startsWith("cameras.")) recomputeTarget(); });

	mux.onAnyFrame((camId, jpegBytes) => {
		if (camId !== targetId) return;
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
				// Detached: draw into the popup's canvas instead of the tile's
				// -- still exactly one decode+draw per frame either way, never
				// both (see popoutCanvas.js for why it's not just mirrored).
				const detached = popout.isOpen();
				const targetCanvas = detached ? popout.getCanvas() : cam;
				const targetCtx = detached ? popout.getCtx() : camCtx;
				if (targetCanvas.width !== bitmap.width || targetCanvas.height !== bitmap.height) {
					targetCanvas.width = bitmap.width;
					targetCanvas.height = bitmap.height;
				}
				targetCtx.drawImage(bitmap, 0, 0);
				bitmap.close();
			} finally {
				renderingCam = false;
			}
		}
		rafOn(popout, renderCamFrame);
	}
	rafOn(popout, renderCamFrame);

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
	const applyFit = () => {
		const fit = config.get("ui.videoFit");
		cam.style.objectFit = fit;
		if (popout.isOpen()) popout.getCanvas().style.objectFit = fit;
	};
	applyFit();
	config.subscribe((path) => { if (path === "ui.videoFit") applyFit(); });
	document.getElementById("btnFit").addEventListener("click", () => {
		config.set("ui.videoFit", config.get("ui.videoFit") === "contain" ? "cover" : "contain");
	});

	// ---- Détacher sur un autre écran (popup fenêtre séparée) ----
	if (btnDetach) {
		popout.onChange((open) => {
			btnDetach.classList.toggle("active", open);
			btnDetach.title = open
				? "Réattacher le flux à cette page"
				: "Détacher sur un autre écran";
			if (detachedOverlay) detachedOverlay.classList.toggle("show", open);
			applyFit(); // le popup vient d'apparaître avec son canvas encore à object-fit par défaut
		});
		btnDetach.addEventListener("click", () => {
			const wasOpen = popout.isOpen();
			popout.toggle();
			if (!wasOpen && !popout.isOpen()) {
				toast("Fenêtre bloquée par le navigateur — autorisez les popups pour ce site.", "warning");
			}
		});
	}

	// ---- Fullscreen ----
	const toggleFullscreen = () => {
		if (document.fullscreenElement) document.exitFullscreen();
		else videoPanel.requestFullscreen && videoPanel.requestFullscreen();
	};
	document.getElementById("btnFullscreen").addEventListener("click", toggleFullscreen);
	cam.addEventListener("dblclick", toggleFullscreen);

	return { isAlive: mux.isAlive, toggleFullscreen };
}
