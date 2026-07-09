// Shared WebSocket connection for BOTH camera streams, demuxed by a 1-byte
// camera-id prefix on every binary message ([1 byte cam_id][JPEG bytes] --
// see robot/uvc_camera_server.py's MultiCameraServer, the sender side of
// this protocol).
//
// Deliberately ONE socket, not one per camera: both feeds then get
// byte-for-byte identical connection-level treatment (same TCP connection,
// same backpressure/ordering, same event-loop scheduling) instead of two
// independent WebSockets that could each stall/reconnect/jitter on their
// own schedule. camera.js and camera2.js each subscribe to their own
// cam_id via onFrame() rather than opening their own connection.

import { createSocket } from "./net.js";

export const CAM_FRONT = 0;
export const CAM_SECOND = 1;

const DEFAULT_ROBOT_IP = "169.254.222.31";
const PORT = 8765;

export function createVideoMux() {
	const robotIp = new URLSearchParams(location.search).get("robotIp") || DEFAULT_ROBOT_IP;
	const listeners = new Map(); // cam_id -> Set<(jpegBytes) => void>

	const sock = createSocket(`ws://${robotIp}:${PORT}`, {
		binary: true,
		onMessage: (e) => {
			const buf = new Uint8Array(e.data);
			if (buf.length < 1) return;
			const camId = buf[0];
			const fns = listeners.get(camId);
			if (!fns) return;
			const jpeg = buf.subarray(1);
			for (const fn of fns) fn(jpeg);
		},
	});

	return {
		isAlive: sock.isAlive,
		// Registers fn(jpegBytes: Uint8Array) for every frame tagged camId.
		onFrame(camId, fn) {
			if (!listeners.has(camId)) listeners.set(camId, new Set());
			listeners.get(camId).add(fn);
		},
	};
}
