// Settings modal: every input carries a data-cfg="a.b.c" attribute and is
// bound generically to the central config store — adding a new tunable is one
// line of HTML plus its DEFAULTS entry, no per-field JS.
//
// Display attributes on the input:
//   data-scale="100"  multiply for display (0.6 -> "60")
//   data-unit=" %"    suffix for display
//   data-fixed="2"    toFixed() digits for display
//   data-type="int"   parse select values as integers

import { config } from "./config.js";
import { toast } from "./toast.js";

function formatFor(input, value) {
	const scale = parseFloat(input.dataset.scale || "1");
	const fixed = input.dataset.fixed;
	const unit = input.dataset.unit || "";
	const v = value * scale;
	return (fixed != null ? v.toFixed(parseInt(fixed, 10)) : Math.round(v * 1000) / 1000) + unit;
}

export function initSettings({ mux } = {}) {
	const $ = (id) => document.getElementById(id);
	const modal = $("settingsModal");

	// ---- Caméras: populate the primary/secondary/tertiary <select>s from
	// the robot's live, auto-discovered camera list (videoMux.js) -- their
	// data-cfg/data-type bindings below are otherwise identical to any
	// other <select>, this just keeps their <option>s in sync with
	// whatever's actually plugged in instead of a fixed list. Runs before
	// loadValues() below so the "auto"/"aucune" sentinel options always
	// exist by the time it tries to select a value.
	if (mux) {
		const primarySelect = $("camPrimarySelect");
		const secondarySelect = $("camSecondarySelect");
		const tertiarySelect = $("camTertiarySelect");
		const hint = $("camListHint");
		mux.onCameraList((cameras) => {
			if (!primarySelect || !secondarySelect || !tertiarySelect) return;
			const fillOptions = (select, includeNone) => {
				const current = select.value;
				select.innerHTML = "";
				select.add(new Option("Auto", "-1"));
				if (includeNone) select.add(new Option("Aucune", "-2"));
				for (const cam of cameras) {
					select.add(new Option(`${cam.name} (id ${cam.id}, ${cam.width}x${cam.height})`, String(cam.id)));
				}
				if ([...select.options].some((o) => o.value === current)) select.value = current;
			};
			fillOptions(primarySelect, false);
			fillOptions(secondarySelect, true);
			fillOptions(tertiarySelect, true);
			// Re-apply the persisted config value now that matching
			// <option>s exist (a plain fillOptions() above only preserves
			// whatever was already visually selected, not the config on
			// first run before loadValues() has ever set it).
			primarySelect.value = String(config.get("cameras.primaryId"));
			secondarySelect.value = String(config.get("cameras.secondaryId"));
			tertiarySelect.value = String(config.get("cameras.tertiaryId"));
			if (hint) {
				hint.textContent = cameras.length
					? `${cameras.length} caméra(s) détectée(s) sur le robot.`
					: "Aucune caméra détectée pour l'instant — branche-la sur le PC robot, elle apparaît ici automatiquement en quelques secondes (pas de redémarrage nécessaire).";
			}
		});
	}

	// ---- Tabs ----
	const tabs = $("settingsTabs");
	tabs.addEventListener("click", (e) => {
		const btn = e.target.closest("button[data-tab]");
		if (!btn) return;
		tabs.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b === btn));
		modal.querySelectorAll(".pane").forEach((p) =>
			p.classList.toggle("on", p.dataset.pane === btn.dataset.tab));
	});

	// ---- Generic binding ----
	const outs = new Map(); // path -> output <b> element
	modal.querySelectorAll("[data-out]").forEach((el) => outs.set(el.dataset.out, el));

	const refreshOut = (path, input, value) => {
		const out = outs.get(path);
		if (out && input) out.textContent = formatFor(input, value);
	};

	const inputs = modal.querySelectorAll("[data-cfg]");
	const loadValues = () => {
		inputs.forEach((input) => {
			const path = input.dataset.cfg;
			const value = config.get(path);
			if (input.type === "checkbox") input.checked = !!value;
			else input.value = String(value);
			if (input.type === "range") refreshOut(path, input, value);
		});
	};
	loadValues();

	inputs.forEach((input) => {
		const path = input.dataset.cfg;
		// data-reload fields (e.g. robot.ip) commit on "change" (blur/Enter)
		// like a select/checkbox, not "input" (every keystroke) -- reloading
		// the page mid-keystroke on a half-typed IP would be actively
		// counterproductive, see the reload block below.
		const event = input.tagName === "SELECT" || input.type === "checkbox" || input.dataset.reload ? "change" : "input";
		input.addEventListener(event, () => {
			let value;
			if (input.type === "checkbox") value = input.checked;
			else if (input.type === "range") value = parseFloat(input.value);
			else if (input.dataset.type === "int") value = parseInt(input.value, 10);
			else value = input.value.trim();
			config.set(path, value);
			if (input.type === "range") refreshOut(path, input, value);
			if (input.dataset.reload) {
				toast("Réglage appliqué — rechargement…", "good");
				setTimeout(() => location.reload(), 600);
			}
		});
	});

	// ---- Open / close ----
	const open = () => { loadValues(); modal.hidden = false; };
	const close = () => { modal.hidden = true; };
	$("btnSettings").addEventListener("click", open);
	$("btnCloseSettings").addEventListener("click", close);
	modal.addEventListener("click", (e) => { if (e.target === modal) close(); });
	document.addEventListener("keydown", (e) => {
		if (e.code === "Escape" && !modal.hidden) close();
	});

	// ---- Export / import / reset ----
	$("btnExport").addEventListener("click", () => {
		const blob = new Blob([config.exportJson()], { type: "application/json" });
		const a = document.createElement("a");
		a.href = URL.createObjectURL(blob);
		a.download = "roby-config.json";
		a.click();
		URL.revokeObjectURL(a.href);
		toast("Configuration exportée");
	});

	const importFile = $("importFile");
	$("btnImport").addEventListener("click", () => importFile.click());
	importFile.addEventListener("change", async () => {
		const file = importFile.files[0];
		importFile.value = "";
		if (!file) return;
		try {
			config.importJson(await file.text());
			toast("Configuration importée — rechargement…", "good");
			// Full reload: the simplest way to guarantee every module re-reads a
			// wholesale-replaced config (loop rates, video fit, mappings…).
			setTimeout(() => location.reload(), 600);
		} catch (e) {
			toast("Import impossible : JSON invalide", "critical");
		}
	});

	$("btnResetCfg").addEventListener("click", () => {
		if (!confirm("Réinitialiser toute la configuration (mapping manette inclus) ?")) return;
		config.reset();
		toast("Configuration réinitialisée — rechargement…", "good");
		setTimeout(() => location.reload(), 600);
	});
}
