// Resolves which discovered camera (by cam_id, i.e. its /dev/videoN index
// on the robot -- see videoMux.js) plays the "primary" (main tile),
// "secondary" and "tertiary" (both picture-in-picture) UI roles, from
// config.js's persisted preference plus the robot's live camera list. Up to
// three simultaneous roles is a UI/layout choice (index.html has one main
// tile + two PiP boxes), not a robot-side limit -- CameraManager will
// happily discover more than three, they're just not assigned a role.
//
// Pure functions, shared by camera.js/cameraPip.js (which render) and
// settings.js (which lets the operator pick by name) -- kept separate from
// both since it's the one bit of logic they all need identically, unlike
// the rendering code itself which camera.js/cameraPip.js deliberately don't
// share (see their own module docstrings).

import { config } from "./config.js";

export const AUTO = -1;
export const NONE = -2; // secondary/tertiary only: operator explicitly wants no picture-in-picture

export function resolvePrimaryId(cameras) {
	const configured = config.get("cameras.primaryId");
	if (configured !== AUTO && cameras.some((c) => c.id === configured)) return configured;
	if (!cameras.length) return null;
	return Math.min(...cameras.map((c) => c.id));
}

export function resolveSecondaryId(cameras, primaryId) {
	const configured = config.get("cameras.secondaryId");
	if (configured === NONE) return null;
	if (configured !== AUTO && cameras.some((c) => c.id === configured)) return configured;
	const rest = cameras.map((c) => c.id).filter((id) => id !== primaryId).sort((a, b) => a - b);
	return rest.length ? rest[0] : null;
}

export function resolveTertiaryId(cameras, primaryId, secondaryId) {
	const configured = config.get("cameras.tertiaryId");
	if (configured === NONE) return null;
	if (configured !== AUTO && cameras.some((c) => c.id === configured)) return configured;
	const rest = cameras.map((c) => c.id).filter((id) => id !== primaryId && id !== secondaryId).sort((a, b) => a - b);
	return rest.length ? rest[0] : null;
}
