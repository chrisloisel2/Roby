// Direct WebSocket link for arm joint-position commands: bypasses Zenoh +
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
// button. Only the joint-position command data itself moves to this link.
//
// Message shape sent here must be the full contract arm_agent.py expects
// -- {"joints": {...}, "gripper": value, "mode": "joint_position"} -- since
// there's no more web_server.py relay step to fill in "mode" the way
// _handle_control() used to.

import { createSocket } from "./net.js";

const DEFAULT_ROBOT_IP = "169.254.222.31";
const ARM_PORT = 8767;

export function createArmLink() {
	const robotIp = new URLSearchParams(location.search).get("robotIp") || DEFAULT_ROBOT_IP;
	const sock = createSocket(`ws://${robotIp}:${ARM_PORT}`);

	return {
		isAlive: sock.isAlive,
		sendJoints(joints, gripper) {
			sock.send({ joints, gripper, mode: "joint_position" });
		},
	};
}
