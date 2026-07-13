// Resolves which robot IP the browser talks to for the two direct-to-robot
// links that bypass Zenoh (videoMux.js's camera WebSocket, armLink.js's
// GELLO relay) -- both need the SAME address since they're the same robot
// PC, just different ports.
//
// Precedence: `?robotIp=` in the URL (explicit, one-off override, e.g. to
// point at a second robot without touching this machine's saved default) >
// config.js's `robot.ip` (persisted per browser/machine in localStorage, set
// in Réglages > Caméras -- so a given operator machine only has to be told
// the robot's IP once, instead of every fresh browser/tab needing the URL
// param) > the hardcoded fallback below (a link-local address from this
// project's original direct-Ethernet-cable bring-up -- almost certainly
// wrong on any other network, see README's `?robotIp=` section).

import { config } from "./config.js";

const DEFAULT_ROBOT_IP = "169.254.222.31";

export function resolveRobotIp() {
	return new URLSearchParams(location.search).get("robotIp") || config.get("robot.ip") || DEFAULT_ROBOT_IP;
}
