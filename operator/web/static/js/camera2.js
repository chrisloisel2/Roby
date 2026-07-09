// Secondary camera stream: a second UVC webcam plugged into the robot PC,
// served by the same robot/camera_pub.py process as the front camera (see
// robot/uvc_camera_server.py). Rendered as a small picture-in-picture
// thumbnail with no fps/age/fit/fullscreen controls -- deliberately NOT
// sharing camera.js's code, since that divergence in UI role would mean a
// pile of unused options on one side or the other.
//
// Frames arrive over the SAME shared WebSocket connection as camera.js
// (videoMux.js), demuxed by cam_id -- see that file for why one connection,
// not two: both cameras get identical connection-level treatment (latency,
// backpressure, scheduling) instead of drifting apart independently.

import { config } from "./config.js";
import { CAM_SECOND } from "./videoMux.js";

export function initCamera2({ mux }) {
	const cam2 = document.getElementById("cam2");
	const noSignal2 = document.getElementById("noSignal2");
	if (!cam2 || !noSignal2) return null;
	const ctx = cam2.getContext("2d", { alpha: false });

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

	return { isAlive: mux.isAlive };
}
