// Direct WebSocket link to robot/arm_agent.py: bypasses Zenoh +
// web_server.py's /ws/control relay, same "direct browser<->robot, one
// fewer hop" pattern as camera_pub.py's video (see videoMux.js) -- but its
// OWN connection/port, not the shared camera one: robot/arm_agent.py needs
// the `lerobot` conda env (RebotB601Follower, torch), whose own
// opencv-python has NO GStreamer support (confirmed empirically
// 2026-07-09), so it can't share a process/socket with camera_pub.py
// (needs system cv2 with GStreamer) without breaking one of the two.
//
// robot/cmd/stop and robot/cmd/reset (E-stop / re-arm) stay on Zenoh via
// /ws/control, UNCHANGED -- they're shared with the base's own E-stop
// (see control.js), and moving them here would decouple that shared
// button. Only the GELLO data itself moves to this link.
//
// Carries RAW, unprocessed GELLO firmware lines -- {"raw": "<line>"} --
// not a calibrated action. arm_agent.py feeds each line straight into a
// REAL lerobot GelloAs5600RawLeader instance's internal state and calls
// its own get_action() -- see that file's module docstring for why: an
// earlier version computed the calibrated action here in the browser (a
// hand-ported reimplementation of GelloAs5600RawLeader's math) and sent
// that instead, which had a real bug (ported the wrong lerobot class
// entirely). Relaying raw data and doing calibration in exactly one place
// (the real lerobot code, server-side) removes that whole class of bug.

import { createSocket } from "./net.js";
import { resolveRobotIp } from "./robotIp.js";

const ARM_PORT = 8767;

export function createArmLink() {
	const robotIp = resolveRobotIp();
	const sock = createSocket(`ws://${robotIp}:${ARM_PORT}`);

	return {
		isAlive: sock.isAlive,
		sendRawLine(line) {
			sock.send({ raw: line });
		},
	};
}
