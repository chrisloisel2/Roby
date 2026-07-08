// GELLO leader arm over Web Serial (Arduino + 7 AS5600L sensors).
//
// Reproduces, in JS, exactly the serial read + calibration math of
// operator/gello_reader.py (Python) -- same constants, same
// gello_calibration.json file (fetched below) -- so the browser can read the
// GELLO the same way it reads the gamepad (Gamepad API) and fully replace
// input_agent.py as the normal command source.
//
// Intentional mirror: if these constants change on the Python side
// (config_gello_as5600_leader.py on the robot PC), port them here by hand --
// there is no automatic sync between the two.
//
// The hardware truths (joint ids / directions / scales) are constants below;
// the *tuning* knobs (baud rate, Arduino boot delay, smoothing, range margin,
// auto-reconnect) live in the central config store, adjustable from the
// settings panel.

import { config } from "./config.js";
import { toast } from "./toast.js";

const GELLO_JOINT_IDS = {
	shoulder_pan: 1, shoulder_lift: 2, elbow_flex: 3,
	wrist_flex: 4, wrist_yaw: 5, wrist_roll: 6, gripper: 7,
};
const GELLO_JOINT_DIRECTIONS = {
	shoulder_pan: -1, shoulder_lift: -1, elbow_flex: -1,
	wrist_flex: 1, wrist_yaw: -1, wrist_roll: -1, gripper: -1,
};
const GELLO_JOINT_SCALES = {
	shoulder_pan: 1.0, shoulder_lift: 1.0, elbow_flex: 1.0,
	wrist_flex: 1.0, wrist_yaw: 1.0, wrist_roll: 1.0, gripper: 3.4,
};
const GELLO_LINE_RE = /J(\d+):(-?\d+\.\d+|ERR)/g;

export function initGello() {
	const $ = (id) => document.getElementById(id);
	const gelloHead = $("gelloHead"), gelloBody = $("gelloBody"), gelloChevron = $("gelloChevron");
	const gelloStatusName = $("gelloStatusName"), gelloRawEl = $("gelloRaw"), gelloKnownPortsEl = $("gelloKnownPorts");

	let gelloCalibration = null;
	let gelloConnected = false;
	let gelloActivePort = null;
	const gelloFiltered = {};   // joint name -> smoothed value (before direction/scale/offset)
	const GELLO_JOINT_NAME_BY_ID = Object.fromEntries(
		Object.entries(GELLO_JOINT_IDS).map(([name, jid]) => [jid, name]));

	gelloHead.addEventListener("click", () => {
		const open = gelloBody.classList.toggle("open");
		gelloChevron.classList.toggle("open", open);
		if (open) refreshKnownPorts();
	});

	// ---- Already-authorized ports: Web Serial remembers permissions per
	// origin, so we can re-list/re-open without going through the browser's
	// native picker (requestPort()) on every page load.
	function portLabel(port) {
		const info = port.getInfo ? port.getInfo() : {};
		if (info.usbVendorId != null) {
			return "USB " + info.usbVendorId.toString(16).padStart(4, "0")
				+ ":" + (info.usbProductId ?? 0).toString(16).padStart(4, "0");
		}
		return "port série";
	}

	async function refreshKnownPorts() {
		gelloKnownPortsEl.innerHTML = "";
		if (!("serial" in navigator) || !navigator.serial.getPorts) return;
		const ports = await navigator.serial.getPorts();
		if (!ports.length) {
			gelloKnownPortsEl.innerHTML = '<div class="joy-raw">aucun port déjà autorisé -- '
				+ 'utiliser "connecter un nouveau port" ci-dessous.</div>';
			return;
		}
		const wrap = document.createElement("div");
		wrap.className = "joy-known-ports";
		ports.forEach((port) => {
			const btn = document.createElement("button");
			btn.className = "joy-reset";
			btn.textContent = "🔌 " + portLabel(port) + (port === gelloActivePort ? " (connecté)" : "");
			btn.disabled = port === gelloActivePort;
			btn.addEventListener("click", () => connectToPort(port));
			wrap.appendChild(btn);
		});
		gelloKnownPortsEl.appendChild(wrap);
	}
	refreshKnownPorts();

	if ("serial" in navigator) {
		// Physical unplug: Web Serial does not close the port by itself on the
		// API side, so detect it via this event instead of waiting for the read
		// loop to fail silently.
		navigator.serial.addEventListener("disconnect", (e) => {
			if (e.target === gelloActivePort) {
				gelloConnected = false;
				gelloActivePort = null;
				gelloStatusName.textContent = "— débranché";
				toast("GELLO débranché", "warning");
			}
			refreshKnownPorts();
		});
	}

	// Reproduces GelloReader.get_action() (operator/gello_reader.py): clip to
	// measured limits (±margin) -> exponential smoothing -> direction -> scale
	// -> offset.
	//
	// The smoothing step runs HERE, once per raw serial line (~60Hz, the
	// GELLO firmware's own streaming rate) -- NOT inside computeAction() below.
	// It used to: computeAction() was only called once per control tick
	// (control.rateHz, 20Hz by default), so the exponential filter only ever
	// advanced 20 times/s instead of 60, which -- for a first-order EMA --
	// directly stretches its settling time by the same factor (roughly 300ms
	// becomes ~1s to reach 95% of a step change). Coupling "how often do we
	// smooth" to "how often do we publish over the network" made the arm feel
	// laggy independently of any real network/CAN latency (measured ~1.2ms
	// round-trip on this hardware -- not the bottleneck). Filtering at the
	// sensor's native rate and simply publishing whatever the filter's latest
	// output is, at whatever rate control.rateHz allows, decouples the two.
	function updateFiltered(jid, raw) {
		if (!gelloCalibration) return;
		const name = GELLO_JOINT_NAME_BY_ID[jid];
		const calib = name && gelloCalibration[name];
		if (!calib) return;
		const margin = config.get("gello.rangeMarginDeg");
		const clipped = Math.max(calib.range_min - margin, Math.min(calib.range_max + margin, raw));
		const prev = gelloFiltered[name];
		const smoothing = config.get("gello.smoothing");
		gelloFiltered[name] = (prev == null) ? clipped : prev + smoothing * (clipped - prev);
	}

	// Called once per control tick: just reads the already-filtered values
	// (see updateFiltered above) and applies direction/scale/offset. Omits a
	// key until a valid reading arrived for that joint (never a fabricated 0.0).
	function computeAction() {
		if (!gelloCalibration) return null;
		const action = {};
		for (const name of Object.keys(GELLO_JOINT_IDS)) {
			const filtered = gelloFiltered[name];
			const calib = gelloCalibration[name];
			if (filtered == null || !calib) continue;
			const direction = GELLO_JOINT_DIRECTIONS[name];
			const scale = GELLO_JOINT_SCALES[name] ?? 1.0;
			const offsetDeg = calib.homing_offset / 100.0;  // stored in centidegrees
			action[name] = scale * (direction * filtered + offsetDeg);
		}
		return Object.keys(action).length ? action : null;
	}

	async function readLoop(port) {
		const decoder = new TextDecoderStream();
		const piped = port.readable.pipeTo(decoder.writable).catch(() => { });
		const reader = decoder.readable.getReader();
		let buf = "";
		try {
			while (gelloConnected) {
				const { value, done } = await reader.read();
				if (done) break;
				buf += value;
				let idx;
				while ((idx = buf.indexOf("\n")) >= 0) {
					const line = buf.slice(0, idx);
					buf = buf.slice(idx + 1);
					GELLO_LINE_RE.lastIndex = 0;
					let m;
					while ((m = GELLO_LINE_RE.exec(line))) {
						if (m[2] === "ERR") continue;
						updateFiltered(parseInt(m[1], 10), parseFloat(m[2]));
					}
				}
			}
		} catch (e) {
			console.error("[gello] read loop error", e);
		} finally {
			reader.releaseLock();
			await piped;
		}
		gelloConnected = false;
		gelloStatusName.textContent = "— déconnecté";
	}

	// Common path for both ways of obtaining a SerialPort: an
	// already-authorized port (list button, no popup) or a new port picked via
	// the browser's native selector (requestPort()).
	async function connectToPort(port) {
		try {
			if (!gelloCalibration) {
				gelloCalibration = await (await fetch("/gello_calibration.json")).json();
			}
			await port.open({ baudRate: config.get("gello.baudRate") });
			gelloActivePort = port;
			gelloConnected = true;
			gelloStatusName.textContent = "— connexion…";
			const info = port.getInfo ? port.getInfo() : {};
			config.set("gello.lastPort", {
				vendorId: info.usbVendorId ?? -1,
				productId: info.usbProductId ?? -1,
			});
			// Opening the port resets the Arduino (DTR) -- let the firmware boot
			// before writing, same delay as gello_reader.py (Python), so we don't
			// land on the recalibration prompt.
			await new Promise((r) => setTimeout(r, config.get("gello.bootDelayMs")));
			const writer = port.writable.getWriter();
			await writer.write(new TextEncoder().encode("n\n"));
			writer.releaseLock();
			gelloStatusName.textContent = "— connecté";
			toast("GELLO connecté (" + portLabel(port) + ")", "good");
			refreshKnownPorts();
			readLoop(port);
		} catch (e) {
			console.error("[gello] connect failed", e);
			gelloConnected = false;
			gelloActivePort = null;
			gelloStatusName.textContent = "— erreur: " + e.message;
		}
	}

	$("gelloConnect").addEventListener("click", async () => {
		if (!("serial" in navigator)) {
			gelloRawEl.textContent = "Web Serial non supporté par ce navigateur -- utiliser Chrome ou Edge, "
				+ "page ouverte en http://localhost:8080 sur le PC où le GELLO est branché.";
			return;
		}
		try {
			const port = await navigator.serial.requestPort();
			await connectToPort(port);
		} catch (e) {
			// User closed the picker without choosing a port -- not an error to
			// surface, just a silent cancellation.
			if (e.name !== "NotFoundError") {
				console.error("[gello] requestPort failed", e);
				gelloStatusName.textContent = "— erreur: " + e.message;
			}
		}
	});

	// ---- Auto-reconnect on page load (opt-in, settings panel) ----
	// Web Serial allows re-opening an already-granted port without a user
	// gesture: match the last successfully-used device by USB ids among the
	// authorized ports, falling back to the only port if a single one is known.
	async function autoConnect() {
		if (!config.get("gello.autoConnect")) return;
		if (!("serial" in navigator) || !navigator.serial.getPorts) return;
		const ports = await navigator.serial.getPorts();
		if (!ports.length) return;
		const last = config.get("gello.lastPort");
		let port = ports.find((p) => {
			const info = p.getInfo ? p.getInfo() : {};
			return last.vendorId >= 0
				&& info.usbVendorId === last.vendorId
				&& info.usbProductId === last.productId;
		});
		if (!port && ports.length === 1) port = ports[0];
		if (port) {
			gelloStatusName.textContent = "— reconnexion auto…";
			await connectToPort(port);
		}
	}
	autoConnect();

	return {
		computeAction,
		isConnected: () => gelloConnected,
		showAction(action) {
			gelloRawEl.textContent = Object.entries(action)
				.map(([k, v]) => `${k}:${v.toFixed(1)}°`).join("  ");
		},
	};
}
