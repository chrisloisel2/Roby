// Secondary camera stream: the Insta360, plugged into the robot PC in USB
// webcam mode and served by robot/insta360_pub.py -- same architecture as
// camera.js's front-camera stream (direct WebSocket to the robot, latest-
// frame-only rendering), but deliberately NOT sharing camera.js's code: this
// is a small picture-in-picture thumbnail with no fps/age/fit/fullscreen
// controls, so the two modules diverge enough that sharing would mean a
// pile of unused options on one side or the other.
//
// Same port convention as camera.js: robot IP from ?robotIp= (falls back to
// DEFAULT_ROBOT_IP), but a different port (8766, not camera.js's 8765) --
// insta360_pub.py is a separate process/server from camera_pub.py.

import { config } from "./config.js";
import { createSocket } from "./net.js";

const DEFAULT_ROBOT_IP = "169.254.222.31";
const INSTA360_PORT = 8766;

export function initCamera2() {
	const cam2 = document.getElementById("cam2");
	const noSignal2 = document.getElementById("noSignal2");
	if (!cam2 || !noSignal2) return null;
	const ctx = cam2.getContext("2d", { alpha: false });

	let lastFrame = 0;
	let latestData = null, dataSeq = 0, displayedSeq = -1, rendering = false;

	const sock = createSocket(`ws://${new URLSearchParams(location.search).get("robotIp") || DEFAULT_ROBOT_IP}:${INSTA360_PORT}`, {
		binary: true,
		onMessage: (e) => {
			latestData = e.data;
			dataSeq++;
			lastFrame = performance.now();
		},
	});

	async function renderLoop() {
		if (dataSeq !== displayedSeq && latestData && !rendering) {
			displayedSeq = dataSeq;
			rendering = true;
			try {
				const bitmap = await createImageBitmap(new Blob([latestData], { type: "image/jpeg" }));
				if (cam2.width !== bitmap.width || cam2.height !== bitmap.height) {
					cam2.width = bitmap.width;
					cam2.height = bitmap.height;
				}
				ctx.drawImage(bitmap, 0, 0);
				bitmap.close();
			} finally {
				rendering = false;
			}
		}
		requestAnimationFrame(renderLoop);
	}
	requestAnimationFrame(renderLoop);

	setInterval(() => {
		const age = lastFrame ? performance.now() - lastFrame : Infinity;
		noSignal2.classList.toggle("show", age > config.get("ui.staleMs"));
	}, 250);

	return { isAlive: sock.isAlive };
}
