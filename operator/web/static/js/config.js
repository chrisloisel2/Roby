// Central configuration store for the operator UI.
//
// Single source of truth for every tunable of the page, persisted as ONE
// versioned localStorage document — instead of the historical scattering of
// ad-hoc keys (roby.browserControl, roby.joystick.mapping.v1,
// roby.joystick.selectedIndex), which are migrated below on first load.
//
// API kept deliberately tiny:
//   get("a.b.c")          read (missing path -> default value)
//   set("a.b.c", v)       write + persist + notify subscribers
//   subscribe(fn)         fn(path, value) after every set()
//   exportJson()          pretty JSON string of the current config
//   importJson(text)      validate + deep-merge over defaults + persist
//   reset()               back to DEFAULTS (persisted)

const STORE_KEY = "roby.config.v2";

export const DEFAULTS = Object.freeze({
	version: 2,
	control: {
		// Matches arm_agent.py's own 50Hz control loop and stays well under
		// the robot watchdog's CMD_TIMEOUT_SEC (0.3s base / 0.3s arm). Only
		// governs base (vx/vy/wz + deadman) now -- GELLO relaying moved off
		// this tick entirely (gello.js sends raw lines straight from its own
		// serial read loop, ~60Hz, whenever they arrive; see gello.js). Used
		// to default to 20Hz, which used to also cap the GELLO relay rate
		// before that decoupling; the base's own loop (100Hz, robot_agent.py)
		// has plenty of headroom at 50Hz either way.
		rateHz: 50,

		defaultSpeed: 0.6,
		rememberSpeed: true,
		speed: 0.6,          // last slider value (used when rememberSpeed)
		gripStep: 0.02,      // gripper increment per tick (gamepad buttons)
		browserControl: false, // opt-in, see control.js for why the default is off
	},
	joystick: {
		deadzone: 0.08,
		selected: "auto",
		// Defaults calibrated for the Thrustmaster T.Flight Stick X
		// (4 axes, 12 buttons, 1 hat) — fully remappable from the UI.
		mapping: {
			axisVx: 1, invVx: true,
			axisVy: 0, invVy: false,
			axisWz: 2, invWz: false,
			axisSpeed: 3, invSpeed: false,
			btnDeadman: 0, btnStop: 1, btnReset: 2, btnGripOpen: 3, btnGripClose: 4,
			// -1 = unassigned: unlike the buttons above (calibrated defaults for
			// the Thrustmaster T.Flight Stick X), there's no sensible physical
			// default for the mast -- must be assigned per-pad, see joystick.js's
			// "Détection auto mât" (hold each button ~2s).
			btnMastUp: -1, btnMastDown: -1,
			// Axe unique bidirectionnel (bascule/palonnier, tableau `axes`
			// du Gamepad API -- PAS `buttons`) dont la VALEUR BRUTE encode le
			// sens -- voir joystick.js poll() / MAST_AXIS_UP_VALUE /
			// MAST_AXIS_DOWN_VALUE. Défaut axe 9, mesuré sur la manette de
			// l'opérateur : valeur -1 = monter, 0.14 = descendre (pas
			// symétrique -- comportement brut du hat/bascule, pas une
			// convention qu'on a choisie). -1 (ce champ-ci, l'INDEX) = désactivé.
			axisMastToggle: 9,
		},
	},
	cameras: {
		// Which discovered camera (cam_id = its /dev/videoN index on the
		// robot, see videoMux.js's camera_list messages) plays which UI
		// role. -1 = auto (lowest id / next-lowest id -- see
		// cameraRoles.js), -2 = none (secondary only: no
		// picture-in-picture box at all). An explicit id that's no longer
		// in the discovered list (camera unplugged/renamed) also falls
		// back to auto rather than showing nothing.
		primaryId: -1,
		secondaryId: -1,
		// Third role (second PiP box, index.html's videoPip3) -- same AUTO
		// default as secondaryId, so a 3rd plugged-in camera just shows up
		// without a settings visit, consistent with the rest of this app's
		// auto-discovery-first philosophy (see robot/camera_pub.py).
		tertiaryId: -1,
		// Caméra montée à l'envers -> retourner l'image de 180°. Réglage
		// navigateur (canvas transform, coût négligeable) plutôt que côté
		// robot, cohérent avec le reste de "quel rôle joue quelle caméra"
		// qui est déjà un réglage opérateur/navigateur (voir cameraRoles.js)
		// et pas quelque chose qu'on encode dans robot/uvc_camera_server.py.
		primaryRotate180: false,
		secondaryRotate180: false,
		tertiaryRotate180: false,
	},
	gello: {
		baudRate: 115200,
		bootDelayMs: 2500,   // opening the port resets the Arduino (DTR) — same wait as gello_reader.py
		// No smoothing/rangeMarginDeg here anymore (2026-07-09): calibration
		// (including smoothing) now runs server-side in arm_agent.py's real
		// lerobot GelloAs5600RawLeader instance, not in this file -- see
		// gello.js's module docstring.
		autoConnect: false,
		// USB ids of the last successfully-opened port, so autoConnect can find
		// the same device again among the already-authorized ports (-1 = none).
		lastPort: { vendorId: -1, productId: -1 },
	},
	ui: {
		videoFit: "contain",
		showTelemetry: true,
		showArmJoints: true,
		staleMs: 1000,
	},
});

function isPlainObject(v) {
	return v !== null && typeof v === "object" && !Array.isArray(v);
}

function deepClone(obj) {
	return JSON.parse(JSON.stringify(obj));
}

// Merge `src` over `base`, but only keep keys that exist in `base` with a
// matching type: an imported/hand-edited config can't inject unknown keys or
// corrupt a number into a string that would then NaN a control loop.
function mergeValidated(base, src) {
	const out = deepClone(base);
	if (!isPlainObject(src)) return out;
	for (const [k, v] of Object.entries(src)) {
		if (!(k in out)) continue;
		if (isPlainObject(out[k])) {
			out[k] = mergeValidated(out[k], v);
		} else if (typeof v === typeof out[k]) {
			out[k] = v;
		}
	}
	return out;
}

// One-time migration of the legacy scattered keys into the unified document.
// Legacy keys are left in place (harmless) so an old build of the page still
// works if someone rolls back.
function migrateLegacy(cfg) {
	try {
		const legacyMap = JSON.parse(localStorage.getItem("roby.joystick.mapping.v1") || "null");
		if (isPlainObject(legacyMap)) cfg.joystick.mapping = mergeValidated(cfg.joystick.mapping, legacyMap);
	} catch { /* ignore malformed legacy data */ }
	const legacySel = localStorage.getItem("roby.joystick.selectedIndex");
	if (legacySel != null) cfg.joystick.selected = legacySel;
	const legacyCtrl = localStorage.getItem("roby.browserControl");
	if (legacyCtrl != null) cfg.control.browserControl = legacyCtrl === "1";
	return cfg;
}

class ConfigStore {
	constructor() {
		this._subs = new Set();
		let stored = null;
		try { stored = JSON.parse(localStorage.getItem(STORE_KEY) || "null"); } catch { /* corrupt -> defaults */ }
		if (stored) {
			this._data = mergeValidated(DEFAULTS, stored);
		} else {
			this._data = migrateLegacy(deepClone(DEFAULTS));
			this._persist();
		}
	}

	_persist() {
		try { localStorage.setItem(STORE_KEY, JSON.stringify(this._data)); } catch { /* quota/private mode */ }
	}

	get(path) {
		let node = this._data;
		for (const part of path.split(".")) {
			if (node == null) return undefined;
			node = node[part];
		}
		return node;
	}

	set(path, value) {
		const parts = path.split(".");
		const last = parts.pop();
		let node = this._data;
		for (const part of parts) {
			if (!isPlainObject(node[part])) node[part] = {};
			node = node[part];
		}
		node[last] = value;
		this._persist();
		for (const fn of this._subs) fn(path, value);
	}

	subscribe(fn) {
		this._subs.add(fn);
		return () => this._subs.delete(fn);
	}

	exportJson() {
		return JSON.stringify(this._data, null, 2);
	}

	importJson(text) {
		const parsed = JSON.parse(text); // throws on invalid JSON — caller handles
		this._data = mergeValidated(DEFAULTS, parsed);
		this._persist();
	}

	reset() {
		this._data = deepClone(DEFAULTS);
		this._persist();
	}
}

export const config = new ConfigStore();
