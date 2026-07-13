// Shared WebSocket connection for EVERY camera stream, demuxed by a 1-byte
// camera-id prefix on every binary message ([1 byte cam_id][JPEG bytes] --
// see robot/uvc_camera_server.py's MultiCameraServer, the sender side of
// this protocol). Text messages on the same connection carry a JSON camera
// list (`{"type": "camera_list", "cameras": [{id, name, width, height}]}`)
// instead of a frame -- sent on connect and again whenever the robot's set
// of discovered cameras changes (see uvc_camera_server.py's CameraManager),
// so the browser's camera-role settings (cameraRoles.js) stay in sync with
// whatever's actually plugged in without a reconnect.
//
// Deliberately ONE socket, not one per camera: every feed then gets
// byte-for-byte identical connection-level treatment (same TCP connection,
// same backpressure/ordering, same event-loop scheduling) instead of
// independent WebSockets that could each stall/reconnect/jitter on their
// own schedule. camera.js and each cameraPip.js instance resolve which
// cam_id they currently want (see cameraRoles.js) and filter onAnyFrame()
// themselves, rather than each opening their own connection or assuming a
// fixed id -- cam_id is just the discovered camera's own /dev/videoN index
// now, not a fixed "front"/"second"/"third" role.

import { createSocket } from "./net.js";
import { resolveRobotIp } from "./robotIp.js";

const PORT = 8765;

export function createVideoMux() {
	const robotIp = resolveRobotIp();
	const frameListeners = new Set(); // Set<(camId, jpegBytes) => void>, fired for every incoming frame regardless of camId
	const listListeners = new Set(); // Set<(cameras) => void>
	let cameras = []; // last camera_list snapshot: [{id, name, width, height}, ...]

	const sock = createSocket(`ws://${robotIp}:${PORT}`, {
		binary: true,
		onMessage: (e) => {
			if (typeof e.data === "string") {
				let msg;
				try { msg = JSON.parse(e.data); } catch { return; }
				if (msg.type !== "camera_list" || !Array.isArray(msg.cameras)) return;
				cameras = msg.cameras;
				for (const fn of listListeners) fn(cameras);
				return;
			}
			const buf = new Uint8Array(e.data);
			if (buf.length < 1) return;
			const camId = buf[0];
			const jpeg = buf.subarray(1);
			for (const fn of frameListeners) fn(camId, jpeg);
		},
	});

	return {
		isAlive: sock.isAlive,
		// Registers fn(camId, jpegBytes: Uint8Array), called for every
		// frame regardless of which camera it came from -- callers filter
		// by whichever camId they currently care about (see
		// cameraRoles.js), since that can change at runtime (settings, or
		// a camera being plugged/unplugged) unlike a one-time subscription.
		onAnyFrame(fn) {
			frameListeners.add(fn);
			return () => frameListeners.delete(fn);
		},
		// fn(cameras) on every camera_list update; called immediately with
		// the last known list too (empty array before the first message).
		onCameraList(fn) {
			listListeners.add(fn);
			fn(cameras);
			return () => listListeners.delete(fn);
		},
		getCameras: () => cameras,
	};
}
