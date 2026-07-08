// Minimal toast notifications — transient operator feedback (E-stop, GELLO
// connect, config import…) that doesn't deserve a permanent slot in the layout.

const DEFAULT_MS = 3200;

export function toast(text, kind = "", ms = DEFAULT_MS) {
	const host = document.getElementById("toasts");
	if (!host) return;
	const el = document.createElement("div");
	el.className = "toast" + (kind ? " " + kind : "");
	el.textContent = text;
	host.appendChild(el);
	setTimeout(() => {
		el.classList.add("out");
		setTimeout(() => el.remove(), 300);
	}, ms);
}
