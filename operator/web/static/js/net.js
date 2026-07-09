// Auto-reconnecting WebSocket helper.
//
// Backoff starts fast (a page reload of web_server.py should reconnect almost
// instantly) and grows to a 3s ceiling so a long outage doesn't hammer the
// network — reset to fast again on every successful open.

const BACKOFF_MIN_MS = 400;
const BACKOFF_MAX_MS = 3000;

// `path` is normally a same-origin path relayed through web_server.py (e.g.
// "/ws/status"), resolved against location.host. Pass a full "ws://host:port"
// URL instead to talk to a different host directly (e.g. camera.js's direct
// connection to robot/camera_pub.py's own WebSocket server, bypassing
// web_server.py entirely).
export function createSocket(path, { onMessage, onOpen, binary = false } = {}) {
	let sock = null;
	let alive = false;
	let backoff = BACKOFF_MIN_MS;
	const url = /^wss?:\/\//.test(path) ? path : `ws://${location.host}${path}`;

	const connect = () => {
		sock = new WebSocket(url);
		if (binary) sock.binaryType = "arraybuffer";
		sock.onopen = () => {
			alive = true;
			backoff = BACKOFF_MIN_MS;
			onOpen && onOpen(sock);
		};
		sock.onmessage = onMessage || null;
		sock.onclose = () => {
			alive = false;
			setTimeout(connect, backoff);
			backoff = Math.min(backoff * 1.6, BACKOFF_MAX_MS);
		};
		sock.onerror = () => sock.close();
	};
	connect();

	return {
		send(obj) { if (alive) sock.send(JSON.stringify(obj)); },
		isAlive: () => alive,
	};
}
