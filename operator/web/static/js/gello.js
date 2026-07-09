// GELLO leader arm over Web Serial (Arduino + 7 AS5600L sensors, RAW firmware).
//
// Reads the GELLO's raw serial stream and relays each line, UNPROCESSED,
// straight to robot/arm_agent.py over armLink.js -- no calibration math
// happens in the browser. arm_agent.py feeds each line into a REAL lerobot
// GelloAs5600RawLeader instance and calls its own get_action(), so
// calibration (unwrap the 0/360 seam, clip to measured range, smooth,
// direction/scale/offset) happens in exactly the ONE place that runs the
// real, authoritative lerobot code -- not a hand-ported reimplementation
// here. An earlier version of this file DID reimplement that math
// client-side, and had a real bug (ported the wrong lerobot teleoperator
// class entirely, no angle-unwrap -- see robot/arm_agent.py's module
// docstring for the full story); relaying raw data removes that whole
// class of bug for good.
//
// Relaying happens directly from the serial read loop below (as fast as
// the firmware streams, ~60Hz), NOT from control.js's tick() -- decoupled
// from control.rateHz entirely now that there's no smoothing filter here
// whose settling time that rate could stretch.

import { config } from "./config.js";
import { toast } from "./toast.js";

const GELLO_JOINT_IDS = {
	shoulder_pan: 1, shoulder_lift: 2, elbow_flex: 3,
	wrist_flex: 4, wrist_yaw: 5, wrist_roll: 6, gripper: 7,
};
const GELLO_JOINT_NAME_BY_ID = Object.fromEntries(
	Object.entries(GELLO_JOINT_IDS).map(([name, jid]) => [jid, name]));
const GELLO_LINE_RE = /J(\d+):(-?\d+\.\d+|ERR)/g;

export function initGello({ armLink }) {
	const $ = (id) => document.getElementById(id);
	const gelloHead = $("gelloHead"), gelloBody = $("gelloBody"), gelloChevron = $("gelloChevron");
	const gelloStatusName = $("gelloStatusName"), gelloRawEl = $("gelloRaw"), gelloKnownPortsEl = $("gelloKnownPorts");

	let gelloConnected = false;
	let gelloActivePort = null;

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

	// Raw-value readout only (no calibration applied) -- just live
	// confirmation the GELLO is connected and moving. Joint names are
	// looked up for readability; the actual value sent to arm_agent.py is
	// the untouched line, not this parsed/relabeled version.
	function showRawLine(line) {
		const parts = [];
		GELLO_LINE_RE.lastIndex = 0;
		let m;
		while ((m = GELLO_LINE_RE.exec(line))) {
			if (m[2] === "ERR") continue;
			const name = GELLO_JOINT_NAME_BY_ID[parseInt(m[1], 10)] ?? `J${m[1]}`;
			parts.push(`${name}:${parseFloat(m[2]).toFixed(1)}°`);
		}
		if (parts.length) gelloRawEl.textContent = parts.join("  ") + "  (brut, non calibré)";
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
					showRawLine(line);
					// Gated the same way the old computeAction()->armLink.sendJoints()
					// path was: only relay to the robot when the operator has
					// explicitly opted into browser control (see control.js) --
					// otherwise a connected-but-not-driving GELLO would still spam
					// arm_agent.py and fight whatever else is in control.
					if (config.get("control.browserControl")) {
						armLink.sendRawLine(line);
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
			// before reading, same delay as gello_reader.py (Python). The RAW
			// firmware has no serial command interface (no zero/recalibrate
			// prompt to answer, unlike the older EEPROM-zeroed GELLO firmware)
			// -- it just starts streaming, so there's nothing to write here.
			await new Promise((r) => setTimeout(r, config.get("gello.bootDelayMs")));
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
		isConnected: () => gelloConnected,
	};
}
