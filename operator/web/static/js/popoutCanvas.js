// Detaches a video tile's rendering into a separate same-origin popup
// window, so the feed can be dragged onto a second monitor independently of
// the rest of the page. Deliberately a plain window.open() popup, not the
// newer Document Picture-in-Picture API -- that one's Chromium-only and
// this needs to work in whatever browser is on the operator PC; a regular
// popup can still be dragged/resized/fullscreened on another display like
// any other window.
//
// Does NOT move the original <canvas> node into the popup's document:
// adopting a canvas across windows is inconsistent across browser engines
// (some reset its bitmap/2D context on the move). Instead this creates a
// fresh <canvas> that belongs to the popup's own document from the start
// and hands back its 2D context -- camera.js/camera2.js's existing render
// loop just points its one decode+draw call at whichever context is
// current, so nothing is duplicated or double-decoded either way.

export function createPopout({ title, ctxOptions }) {
	let win = null;
	let canvas = null;
	let ctx = null;
	let pollId = null;
	const listeners = new Set();

	function isOpen() {
		return !!win && !win.closed;
	}

	function notify() {
		for (const fn of listeners) fn(isOpen());
	}

	function teardown() {
		if (pollId) { clearInterval(pollId); pollId = null; }
		win = null;
		canvas = null;
		ctx = null;
		notify();
	}

	function open() {
		if (isOpen()) { win.focus(); return; }
		win = window.open("", "_blank", "width=960,height=600");
		if (!win) return; // blocked by the browser's popup blocker
		win.document.title = title;
		win.document.head.insertAdjacentHTML("beforeend", `<style>
			html, body { margin: 0; height: 100%; background: #000; overflow: hidden; }
			canvas { width: 100vw; height: 100vh; object-fit: contain; display: block; }
		</style>`);
		canvas = win.document.createElement("canvas");
		win.document.body.appendChild(canvas);
		ctx = canvas.getContext("2d", ctxOptions);
		// Polling, not just a listener on the popup itself: it's the standard
		// robust way to detect a popup closing (unload/pagehide events on a
		// cross-window popup aren't consistently deliverable in every
		// browser, especially when the user closes it via the OS titlebar).
		pollId = setInterval(() => { if (win.closed) teardown(); }, 400);
		notify();
	}

	function close() {
		if (isOpen()) win.close();
		teardown();
	}

	function toggle() {
		if (isOpen()) close(); else open();
	}

	// Never leave an orphaned popup behind if the main tab/page goes away.
	window.addEventListener("beforeunload", () => { if (isOpen()) win.close(); });

	return {
		toggle,
		isOpen,
		getCanvas: () => canvas,
		getCtx: () => ctx,
		// The window whose requestAnimationFrame should currently drive
		// drawing -- see rafOn() below for why this matters.
		getWindow: () => win,
		onChange: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
	};
}

// Schedules fn via whichever window is presently visible to the user:
// popout's own window once detached, the main window otherwise. Browsers
// throttle requestAnimationFrame hard (down to ~1fps or fully paused) for a
// hidden/backgrounded document -- and the main page DOES become hidden the
// moment its popup takes focus (confirmed directly: document.visibilityState
// flips to "hidden" on the opener as soon as the popup opens). Scheduling
// off the main window unconditionally would make the whole point of
// detaching -- smooth video on a second monitor while working elsewhere on
// the first -- fall apart the instant the operator looks away from the now-
// empty original tile. Scheduling off whichever window currently holds the
// canvas keeps that window's own visibility (not the other one's) in
// control of the frame rate, which is what should determine it.
export function rafOn(popout, fn) {
	(popout.isOpen() ? popout.getWindow() : window).requestAnimationFrame(fn);
}
