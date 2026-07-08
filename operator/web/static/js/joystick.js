// Gamepad (Gamepad API) — tested with a Thrustmaster T.Flight Stick X
// (4 axes, 12 buttons, 1 hat). Mapping is fully dynamic: click "Assigner"
// then move the axis / press the physical button. Persisted in the central
// config store (joystick.mapping), like every other tunable.

import { config, DEFAULTS } from "./config.js";
import { toast } from "./toast.js";

const JOY_ACTIONS = [
	{ key: "axisVx", label: "Avant / arrière", kind: "axis", invKey: "invVx" },
	{ key: "axisVy", label: "Latéral", kind: "axis", invKey: "invVy" },
	{ key: "axisWz", label: "Rotation", kind: "axis", invKey: "invWz" },
	{ key: "axisSpeed", label: "Vitesse max (slider)", kind: "axis", invKey: "invSpeed" },
	{ key: "btnDeadman", label: "Homme-mort", kind: "button" },
	{ key: "btnStop", label: "Arrêt d'urgence", kind: "button" },
	{ key: "btnReset", label: "Réarmer", kind: "button" },
	{ key: "btnGripOpen", label: "Pince : ouvrir", kind: "button" },
	{ key: "btnGripClose", label: "Pince : fermer", kind: "button" },
];

export function initJoystick({ onStop, onReset, onGripDelta }) {
	const $ = (id) => document.getElementById(id);
	const joyHead = $("joyHead"), joyBody = $("joyBody"), joyChevron = $("joyChevron");
	const joyName = $("joyName"), joyRaw = $("joyRaw"), joySelect = $("joySelect");

	const mapping = () => config.get("joystick.mapping");
	const setMapping = (key, value) => config.set(`joystick.mapping.${key}`, value);

	let learningKey = null;
	let joyIndex = null;
	let prevJoyButtons = [];

	joyHead.addEventListener("click", () => {
		const open = joyBody.classList.toggle("open");
		joyChevron.classList.toggle("open", open);
		if (open) refreshJoyOptions();
	});

	// ---- Active gamepad: manual pick when several are plugged in ----
	// "auto" (default) = historical behaviour, first one detected. An explicit
	// choice is persisted and wins as long as that pad stays plugged in; if it
	// disappears we fall back to "auto" rather than blocking (the app must
	// keep working even with no pad selected).
	joySelect.value = config.get("joystick.selected");
	joySelect.addEventListener("change", () => {
		// A focused <select> keeps capturing the keyboard (game keys are — by
		// design — inert there); give focus back to the page once chosen.
		joySelect.blur();
		config.set("joystick.selected", joySelect.value);
	});

	function refreshJoyOptions() {
		const pads = navigator.getGamepads ? navigator.getGamepads() : [];
		const found = Array.prototype.filter.call(pads, (p) => p);
		joySelect.innerHTML = '<option value="auto">auto (première détectée)</option>';
		for (const p of found) {
			const opt = document.createElement("option");
			opt.value = String(p.index);
			opt.textContent = p.index + ": " + p.id;
			joySelect.appendChild(opt);
		}
		const selected = config.get("joystick.selected");
		const known = Array.prototype.some.call(joySelect.options, (o) => o.value === selected);
		joySelect.value = known ? selected : "auto";
	}
	refreshJoyOptions();

	// ---- Mapping rows: built once, then only their value spans mutate ----
	const rowRefs = new Map(); // action key -> value span
	function renderJoyRows() {
		const wrap = $("joyRows");
		wrap.innerHTML = "";
		rowRefs.clear();
		for (const a of JOY_ACTIONS) {
			const row = document.createElement("div");
			row.className = "joy-row";
			const cur = mapping()[a.key];
			const curText = (cur == null || cur < 0) ? "—" : (a.kind === "axis" ? "axe " + cur : "bouton " + cur);
			const invHtml = a.kind === "axis"
				? '<label class="inv"><input type="checkbox" id="joyinv-' + a.key + '"' +
				(mapping()[a.invKey] ? " checked" : "") + '> inv.</label>'
				: "";
			row.innerHTML =
				'<span class="lbl">' + a.label + '</span>' +
				'<span class="val" id="joyval-' + a.key + '">' + curText + '</span>' +
				invHtml +
				'<button class="assign" data-key="' + a.key + '">Assigner</button>';
			wrap.appendChild(row);
			rowRefs.set(a.key, row.querySelector(".val"));
			if (a.kind === "axis") {
				$("joyinv-" + a.key).addEventListener("change", (e) => setMapping(a.invKey, e.target.checked));
			}
		}
		wrap.querySelectorAll(".assign").forEach((btn) => {
			btn.addEventListener("click", () => {
				wrap.querySelectorAll(".assign").forEach((b) => { b.classList.remove("listening"); b.textContent = "Assigner"; });
				learningKey = btn.dataset.key;
				btn.classList.add("listening");
				btn.textContent = "…appuyer / bouger";
			});
		});
	}
	renderJoyRows();

	$("joyResetMap").addEventListener("click", () => {
		config.set("joystick.mapping", { ...DEFAULTS.joystick.mapping });
		renderJoyRows();
		toast("Mapping manette réinitialisé");
	});

	window.addEventListener("gamepadconnected", (e) => {
		refreshJoyOptions();
		toast("Manette détectée : " + e.gamepad.id, "good");
		if (config.get("joystick.selected") === "auto" && joyIndex == null) {
			joyIndex = e.gamepad.index;
			joyName.textContent = "— " + e.gamepad.id;
		}
	});
	window.addEventListener("gamepaddisconnected", (e) => {
		refreshJoyOptions();
		if (joyIndex === e.gamepad.index) {
			joyIndex = null;
			joyName.textContent = "— non détectée";
			joyRaw.textContent = "en attente d'une manette…";
		}
	});

	function pollGamepad() {
		const pads = navigator.getGamepads ? navigator.getGamepads() : [];
		const selected = config.get("joystick.selected");
		let gp = null;
		if (selected !== "auto") gp = pads[parseInt(selected, 10)] || null;
		if (!gp) gp = joyIndex != null ? pads[joyIndex] : null;
		if (!gp) gp = Array.prototype.find.call(pads, (p) => p) || null;
		if (gp && joyIndex !== gp.index) { joyIndex = gp.index; joyName.textContent = "— " + gp.id; }
		if (!gp) joyIndex = null;
		return gp;
	}

	function finishAssign(key, value) {
		setMapping(key, value);
		const kind = JOY_ACTIONS.find((a) => a.key === key).kind;
		const val = rowRefs.get(key);
		if (val) val.textContent = kind === "axis" ? "axe " + value : "bouton " + value;
		const btn = document.querySelector('.assign[data-key="' + key + '"]');
		if (btn) { btn.classList.remove("listening"); btn.textContent = "Assigner"; }
		learningKey = null;
	}

	// Polled once per control tick — reads gamepad state, drives the
	// learn-mode capture, and returns the normalized contribution to send.
	function poll() {
		const gp = pollGamepad();
		const out = { vx: 0, vy: 0, wz: 0, speed: null, deadman: false };
		if (!gp) { joyRaw.textContent = "en attente d'une manette…"; return out; }

		const map = mapping();
		const deadzone = config.get("joystick.deadzone");
		const axes = gp.axes, buttons = gp.buttons;
		const activeBtns = [];
		for (let i = 0; i < buttons.length; i++) if (buttons[i].pressed) activeBtns.push(i);

		// Raw dump only when the panel is open — no string building for a
		// hidden element on every tick.
		if (joyBody.classList.contains("open")) {
			let raw = "axes: ";
			for (let i = 0; i < axes.length; i++) raw += i + ":" + axes[i].toFixed(2) + "  ";
			raw += "  boutons actifs: " + (activeBtns.length ? activeBtns.join(",") : "—");
			joyRaw.textContent = raw;
		}

		if (learningKey) {
			const action = JOY_ACTIONS.find((a) => a.key === learningKey);
			if (action.kind === "axis") {
				const idx = axes.findIndex((v) => Math.abs(v) > 0.6);
				if (idx >= 0) finishAssign(learningKey, idx);
			} else if (activeBtns.length) {
				finishAssign(learningKey, activeBtns[0]);
			}
		}

		const axisVal = (idx, inv) => {
			if (idx == null || idx < 0 || idx >= axes.length) return 0;
			let v = axes[idx];
			if (Math.abs(v) < deadzone) v = 0;
			return inv ? -v : v;
		};
		const btnHeld = (idx) => idx != null && idx >= 0 && idx < buttons.length && buttons[idx].pressed;

		out.vx = axisVal(map.axisVx, map.invVx);
		out.vy = axisVal(map.axisVy, map.invVy);
		out.wz = axisVal(map.axisWz, map.invWz);
		if (map.axisSpeed != null && map.axisSpeed >= 0 && map.axisSpeed < axes.length) {
			let v = axes[map.axisSpeed];
			if (map.invSpeed) v = -v;
			out.speed = Math.min(1, Math.max(0, (v + 1) / 2));
		}
		out.deadman = btnHeld(map.btnDeadman);

		const pressedNow = buttons.map((b) => b.pressed);
		const rising = (idx) => idx != null && idx >= 0 && pressedNow[idx] && !prevJoyButtons[idx];
		if (rising(map.btnStop)) onStop();
		if (rising(map.btnReset)) onReset();
		const gripStep = config.get("control.gripStep");
		if (btnHeld(map.btnGripOpen)) onGripDelta(-gripStep);
		if (btnHeld(map.btnGripClose)) onGripDelta(+gripStep);
		prevJoyButtons = pressedNow;

		return out;
	}

	return { poll };
}
