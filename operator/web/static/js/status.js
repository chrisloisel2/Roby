// Robot / arm status: heartbeat, reported state, tiles, E-stop banner and
// the arm joint mini-gauges.

import { config } from "./config.js";
import { createSocket } from "./net.js";
import { toast } from "./toast.js";

export function setTile(id, status, text) {
	const t = document.getElementById(id);
	t.dataset.s = status;
	t.querySelector(".v span:last-child").textContent = text;
}

export function initStatus() {
	const banner = document.getElementById("banner");
	const brandDot = document.getElementById("brandDot");
	const reasonLine = document.getElementById("reasonLine");
	const armJointsEl = document.getElementById("armJoints");

	let robotState = {};
	let lastSnapshot = {}; // dernier message /ws/status complet ({robot, state, arm, mast})
	let prevEstop = null;

	// Joint rows are created once per joint name then only updated — no
	// innerHTML rebuild at 5 Hz.
	const jointRows = new Map(); // name -> {fill, val}
	function renderJoints(joints) {
		if (!config.get("ui.showArmJoints") || !Object.keys(joints).length) {
			armJointsEl.hidden = true;
			return;
		}
		armJointsEl.hidden = false;
		for (const [name, deg] of Object.entries(joints)) {
			let row = jointRows.get(name);
			if (!row) {
				const el = document.createElement("div");
				el.className = "arm-joint";
				el.innerHTML = "<span></span><span class='jtrack'><span class='jfill'></span></span><span class='jval'></span>";
				el.firstChild.textContent = name.replace(/_/g, " ");
				armJointsEl.appendChild(el);
				row = { fill: el.querySelector(".jfill"), val: el.querySelector(".jval") };
				jointRows.set(name, row);
			}
			// Indicative scale only: reported angles are follower degrees, whose
			// true per-joint ranges live robot-side — a fixed ±180° window keeps
			// the bar honest without pretending to know the real limits.
			const norm = Math.max(-1, Math.min(1, deg / 180));
			row.fill.style.transform = `scaleX(${norm})`;
			row.val.textContent = deg.toFixed(0) + "°";
		}
	}

	// state schema published by robot/robot_agent.py:
	//   {moving, estop, deadman_ok, fresh_cmd, ts}
	const statusSock = createSocket("/ws/status", {
		onMessage: (e) => {
			const s = JSON.parse(e.data);
			lastSnapshot = s;
			robotState = s.state || {};
			setTile("t-robot", s.robot ? "good" : "critical", s.robot ? "ACTIF" : "PERDU");
			brandDot.style.background = s.robot ? "var(--good)" : "var(--critical)";
			brandDot.style.boxShadow = `0 0 0 4px color-mix(in srgb, ${s.robot ? "var(--good)" : "var(--critical)"} 22%, transparent)`;

			const estop = !!robotState.estop;
			banner.classList.toggle("show", estop);
			banner.textContent = estop
				? "⚠ ARRÊT D'URGENCE ACTIF — cliquer RÉARMER puis ré-activer le homme-mort"
				: "";
			if (prevEstop !== null && estop !== prevEstop) {
				toast(estop ? "Arrêt d'urgence déclenché" : "E-stop réarmé", estop ? "critical" : "good");
			}
			prevEstop = estop;

			setTile("t-fsm",
				estop ? "critical" : (robotState.moving ? "good" : "warning"),
				estop ? "E-STOP" : (robotState.moving ? "EN MOUVEMENT" : "ARRÊT"));
			reasonLine.textContent = (!estop && !robotState.moving)
				? `homme-mort : ${robotState.deadman_ok ? "oui" : "non"} · commande : ${robotState.fresh_cmd ? "fraîche" : "périmée"}`
				: "";

			// arm state schema published by robot/arm_agent.py (relayed as-is,
			// empty object once stale -- see web_server.py):
			//   {connected, moving, fresh_cmd, estop, joints, ts}
			const armState = s.arm || {};
			const armEstop = !!armState.estop;
			setTile("t-arm",
				!armState.connected ? "critical" : (armEstop ? "critical" : (armState.moving ? "good" : "warning")),
				!armState.connected ? "DÉCONNECTÉ" : (armEstop ? "E-STOP" : (armState.moving ? "EN MOUVEMENT" : "INACTIF")));
			renderJoints(armState.joints || {});

			// mast state schema published by robot/mast_serial_bridge.py, relayed
			// as-is by web_server.py (see that module's on_mast_state/on_mast_link):
			//   {linked, position_mm, fdc_min, fdc_max, t}
			// "linked" always reflects robot/mast/link, even once position_mm etc.
			// have gone stale -- see web_server.py's status_ws for why.
			const mastState = s.mast || {};
			const mastLinked = !!mastState.linked;
			const mastAtLimit = !!(mastState.fdc_min || mastState.fdc_max);
			const mastPos = mastState.position_mm;
			setTile("t-mast",
				!mastLinked ? "critical" : (mastAtLimit ? "warning" : "good"),
				!mastLinked ? "DÉCONNECTÉ" : (
					(mastPos != null ? `${mastPos.toFixed(0)} mm` : "en attente")
					+ (mastAtLimit ? " · FDC" : "")));
		},
	});

	config.subscribe((path) => {
		if (path === "ui.showArmJoints" && !config.get("ui.showArmJoints")) armJointsEl.hidden = true;
	});

	return {
		isAlive: statusSock.isAlive,
		getRobotState: () => robotState,
		// Snapshot /ws/status complet (dont s.robot, la vivacité heartbeat,
		// et s.mast) -- consommé par birdview.js pour le dead-reckoning.
		getSnapshot: () => lastSnapshot,
	};
}
