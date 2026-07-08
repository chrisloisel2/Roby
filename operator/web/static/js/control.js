// Command source: keyboard + on-screen d-pad + gamepad + GELLO, merged and
// published to the server over /ws/control at a fixed, configurable rate.
//
// Keyboard letters are matched by PHYSICAL position (event.code, KeyW/KeyA/…)
// instead of the produced character: the same finger positions work on QWERTY
// (WASD+QE) and AZERTY (ZQSD+AE) without any layout setting.

import { config } from "./config.js";
import { createSocket } from "./net.js";
import { toast } from "./toast.js";
import { setTile } from "./status.js";

const KEYMAP = {
	KeyW: "fwd", KeyS: "back", KeyQ: "left", KeyE: "right", KeyA: "rotL", KeyD: "rotR",
	ArrowUp: "fwd", ArrowDown: "back", ArrowLeft: "rotL", ArrowRight: "rotR",
};

export function initControl({ onFullscreen }) {
	const $ = (id) => document.getElementById(id);
	const ctrlSock = createSocket("/ws/control");

	const keys = { fwd: 0, back: 0, left: 0, right: 0, rotL: 0, rotR: 0 };
	let deadman = false;
	let speed = config.get("control.rememberSpeed")
		? config.get("control.speed")
		: config.get("control.defaultSpeed");
	let gripper = 0.0;

	// ---- Speed slider ----
	const spd = $("spd"), spdVal = $("spdVal");
	const showSpeed = () => {
		spd.value = Math.round(speed * 100);
		spdVal.textContent = Math.round(speed * 100) + " %";
	};
	showSpeed();
	spd.addEventListener("input", (e) => {
		speed = e.target.value / 100;
		spdVal.textContent = e.target.value + " %";
		if (config.get("control.rememberSpeed")) config.set("control.speed", speed);
	});

	// ---- Gripper slider ----
	let gripTimer = 0;
	function setGripper(v) {
		gripper = Math.min(1, Math.max(0, v));
		$("grip").value = Math.round(gripper * 100);
		$("gripVal").textContent = gripper < 0.05 ? "ouverte" : gripper > 0.95 ? "fermée" : Math.round(gripper * 100) + " %";
		clearTimeout(gripTimer);
		gripTimer = setTimeout(() => ctrlSock.send({ type: "gripper", value: gripper }), 40);
	}
	$("grip").addEventListener("input", (e) => setGripper(e.target.value / 100));

	// ---- Deadman ----
	const dm = $("deadman");
	const setDeadman = (v) => {
		deadman = v;
		dm.classList.toggle("armed", v);
	};
	dm.addEventListener("pointerdown", (e) => { e.preventDefault(); setDeadman(true); });
	dm.addEventListener("pointerup", () => setDeadman(false));
	dm.addEventListener("pointerleave", () => setDeadman(false));
	dm.addEventListener("pointercancel", () => setDeadman(false));

	// ---- E-stop / reset ----
	const triggerStop = () => { ctrlSock.send({ type: "stop" }); };
	const sendReset = () => { ctrlSock.send({ type: "reset" }); };
	$("estop").addEventListener("click", (e) => { e.currentTarget.blur(); triggerStop(); });
	$("reset").addEventListener("click", (e) => { e.currentTarget.blur(); sendReset(); });
	// Sliders too: released focus = the keyboard always drives the robot.
	spd.addEventListener("change", () => spd.blur());
	$("grip").addEventListener("change", (e) => e.target.blur());

	// ---- Keyboard ----
	// Game keys are ignored ONLY while a modal is open or while typing in a
	// text-entry control: there, Space/arrows must keep their native meaning.
	const modalOpen = () => document.querySelector(".modal-overlay:not([hidden])");
	const isTextEntry = (el) => el.closest?.("select, textarea, input[type=text], input[type=number]");

	const GAME_KEYS = new Set(["Space", "KeyX", "KeyR", "KeyF"]);
	const isGameKey = (code) => GAME_KEYS.has(code) || code in KEYMAP;

	const highlight = (k, on) =>
		document.querySelectorAll(`.dpad button[data-k="${k}"]`).forEach(b => b.classList.toggle("on", on));

	document.addEventListener("keydown", (ev) => {
		if (modalOpen() || isTextEntry(ev.target)) return;
		// A main-page control keeps focus after a click (the "Piloter depuis ce
		// navigateur" toggle, a slider, the E-stop button…). It must NOT capture
		// the keyboard: a focused checkbox toggles on Espace — which silently
		// switched browser control back OFF right after the operator enabled it,
		// so the robot "mysteriously" ignored every key — and a focused slider
		// eats the arrows. For any driving key, take the focus back (blur also
		// kills the pending Space-keyup activation of a checkbox/button) and
		// handle the key normally; other keys (Tab…) keep native behaviour.
		const focused = ev.target.closest?.("input, button");
		if (focused) {
			if (!isGameKey(ev.code)) return;
			ev.preventDefault();
			focused.blur();
		}
		if (ev.code === "Space") { ev.preventDefault(); setDeadman(true); return; }
		if (ev.code === "KeyX") { triggerStop(); return; }
		if (ev.code === "KeyR") { sendReset(); return; }
		if (ev.code === "KeyF") { onFullscreen && onFullscreen(); return; }
		if (KEYMAP[ev.code]) { ev.preventDefault(); keys[KEYMAP[ev.code]] = 1; highlight(KEYMAP[ev.code], true); }
	});
	document.addEventListener("keyup", (ev) => {
		if (ev.code === "Space") { setDeadman(false); return; }
		if (KEYMAP[ev.code]) { keys[KEYMAP[ev.code]] = 0; highlight(KEYMAP[ev.code], false); }
	});
	// Release everything if the tab loses focus (safety).
	window.addEventListener("blur", () => {
		for (const k in keys) keys[k] = 0;
		setDeadman(false);
		document.querySelectorAll(".dpad button").forEach(b => b.classList.remove("on"));
	});

	// ---- On-screen d-pad (pointer = works on touch + mouse) ----
	document.querySelectorAll(".dpad button[data-k]").forEach(btn => {
		const k = btn.dataset.k;
		const down = (e) => { e.preventDefault(); keys[k] = 1; btn.classList.add("on"); };
		const up = () => { keys[k] = 0; btn.classList.remove("on"); };
		btn.addEventListener("pointerdown", down);
		btn.addEventListener("pointerup", up);
		btn.addEventListener("pointerleave", up);
		btn.addEventListener("pointercancel", up);
	});

	// ---- Browser-as-controller toggle ----
	// Off by default: the control loop below runs unconditionally the moment
	// the page is open (e.g. just to watch the camera), and used to publish
	// deadman/base at 20Hz regardless -- which silently fights any other
	// command source (input_agent.py's joystick reader, running at 50Hz)
	// publishing to the exact same Zenoh topics. Two sources racing on
	// "deadman" means it flips true/false every other message, so deadman_ok
	// reads false most of the time and the robot never actually moves, even
	// though someone IS correctly holding the physical deadman button on
	// their own controller. Gating all of it behind an explicit opt-in makes
	// "just watching the camera" safe.
	const browserCtrlBox = $("browserCtrl");
	browserCtrlBox.checked = config.get("control.browserControl");
	browserCtrlBox.addEventListener("change", (e) => {
		// Drop focus right away so the very next Espace arms the deadman
		// instead of re-toggling this checkbox (see the keydown handler).
		e.target.blur();
		config.set("control.browserControl", e.target.checked);
		if (!e.target.checked) {
			// Release explicitly so this browser's last command doesn't linger
			// as the most-recent one at the robot for CMD_TIMEOUT_SEC.
			ctrlSock.send({ type: "deadman", value: false });
			ctrlSock.send({ type: "base", vx: 0, vy: 0, wz: 0 });
			toast("Pilotage navigateur désactivé");
		} else {
			toast("Pilotage navigateur activé — homme-mort requis pour bouger", "warning");
		}
	});

	// ---- Meters (composited transform, no layout per tick) ----
	const meter = (fillId, valId) => ({ fill: $(fillId), val: $(valId), last: NaN });
	const meters = { vx: meter("m-vx", "v-vx"), vy: meter("m-vy", "v-vy"), wz: meter("m-wz", "v-wz") };
	const setMeter = (m, v) => {
		if (v === m.last) return;
		m.last = v;
		m.fill.style.transform = `scaleX(${Math.max(-1, Math.min(1, v))})`;
		m.val.textContent = (v >= 0 ? "+" : "") + v.toFixed(2);
	};

	// ---- Control loop, rate configurable (restarted when rateHz changes) ----
	let loopTimer = 0;
	function start({ joystick, gello }) {
		const tick = () => {
			// The base command send below is safety-relevant (the robot-side
			// watchdog stops on stale commands): a crash in an input source
			// (gamepad quirk, serial hiccup) must degrade to "that source reads
			// zero", never to "the whole loop is dead and nothing is sent".
			let joy;
			try {
				joy = joystick.poll();
			} catch (err) {
				console.error("[control] joystick.poll() failed", err);
				joy = { vx: 0, vy: 0, wz: 0, speed: null, deadman: false };
			}
			const browserControlEnabled = config.get("control.browserControl");
			const activeDeadman = (deadman || joy.deadman) && browserControlEnabled;
			if (joy.speed != null && Math.abs(joy.speed - speed) > 0.005) {
				speed = joy.speed;
				showSpeed();
			}

			let vx = Math.max(-1, Math.min(1, (keys.fwd - keys.back) + joy.vx)) * speed;
			let vy = Math.max(-1, Math.min(1, (keys.right - keys.left) + joy.vy)) * speed;   // Q/E strafe
			let wz = Math.max(-1, Math.min(1, (keys.rotR - keys.rotL) + joy.wz)) * speed;    // A/D rotate
			if (!activeDeadman) { vx = vy = wz = 0; }
			setMeter(meters.vx, vx);
			setMeter(meters.vy, vy);
			setMeter(meters.wz, wz);
			if (browserControlEnabled) {
				ctrlSock.send({ type: "deadman", value: activeDeadman });
				ctrlSock.send({ type: "base", vx, vy, wz });
			}
			dm.classList.toggle("armed", activeDeadman);
			setTile("t-dead", activeDeadman ? "good" : "warning", activeDeadman ? "ARMÉ" : "RELÂCHÉ");

			// GELLO: no deadman here (driving a 7-DOF arm takes both hands) --
			// gated only by the "Piloter depuis ce navigateur" toggle, same
			// principle as operator/input_agent.py which publishes the arm
			// independently of the base deadman.
			try {
				if (gello.isConnected()) {
					const action = gello.computeAction();
					if (action) {
						gello.showAction(action);
						if (browserControlEnabled) {
							const grip = action.gripper;
							const joints = { ...action };
							delete joints.gripper;
							ctrlSock.send({ type: "arm", joints, gripper: grip });
						}
					}
				}
			} catch (err) {
				console.error("[control] gello tick failed", err);
			}
		};
		const restart = () => {
			clearInterval(loopTimer);
			loopTimer = setInterval(tick, 1000 / config.get("control.rateHz"));
		};
		restart();
		config.subscribe((path) => { if (path === "control.rateHz") restart(); });
	}

	return {
		isAlive: ctrlSock.isAlive,
		start,
		triggerStop,
		sendReset,
		setGripper,
		adjustGripper: (d) => setGripper(gripper + d),
	};
}
