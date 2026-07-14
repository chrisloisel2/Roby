// Vue spatiale : représentation du robot et de son environnement vue de
// dessus, composée dans le navigateur à partir des flux caméra existants --
// aucun capteur ni process robot supplémentaire, aucune connexion en plus
// (même mux vidéo que les tuiles, videoMux.js).
//
// Deux algorithmes classiques de la perception automobile, portés au
// navigateur :
//
//  * IPM (Inverse Perspective Mapping) -- chaque image des caméras avant /
//    arrière est reprojetée sur le plan du sol par un shader WebGL
//    (projection inverse par pixel, corrigée en perspective) à partir d'une
//    calibration simple hauteur / inclinaison / FOV (Réglages > Vue
//    spatiale). C'est la technique des "surround view" des voitures.
//  * Dead-reckoning odométrique -- la pose (x, y, cap) est intégrée en
//    continu à partir des vitesses RÉELLEMENT appliquées par la base,
//    republiées par robot_agent.py dans robot/state ("vel" : cinématique
//    mecanum inverse des consignes roues post-rampe). Repli sur la dernière
//    commande émise par ce navigateur si le robot ne publie pas encore ce
//    champ.
//
// Les projections sol successives s'accumulent dans une MOSAÏQUE MONDE
// persistante : l'environnement se "peint" autour du robot au fil du
// déplacement. Un fondu temporel réglable matérialise la confiance
// décroissante (l'odométrie dérive : une zone vue il y a longtemps n'est
// plus garantie exacte). La caméra pince, elle, n'est pas projetable au sol
// (elle vise la zone de travail) : elle s'affiche en médaillon circulaire
// accroché à l'avant du glyphe robot.
//
// Conventions géométriques (tout le dessin travaille en MÈTRES monde) :
//   monde : x vers l'est, y vers le nord, cap yaw en rad CCW depuis le nord
//           -> avant robot = (-sin yaw, cos yaw), droite = (cos yaw, sin yaw)
//   local robot : x = droite, y = avant.

import { config } from "./config.js";
import { resolvePrimaryId, resolveSecondaryId, resolveTertiaryId } from "./cameraRoles.js";
import { toast } from "./toast.js";

const DEG = Math.PI / 180;
const GL_W = 384, GL_H = 512;        // cible de rendu IPM (latéral x avant), par caméra
const MAP_SIZE = 2048;               // px de la mosaïque monde
const MAP_EXTENT_M = 16;             // m couverts par la mosaïque (128 px/m)
const MAP_PPM = MAP_SIZE / MAP_EXTENT_M;
const MAP_MARGIN_M = 3;              // recentrage quand le robot approche du bord
const FAR_CAP_M = 5;                 // portée max projetée au sol (au-delà : trop écrasé)
const STAMP_EVERY_MS = 120;          // cadence max d'accumulation dans la mosaïque
const DECODE_EVERY_MS = 80;          // cadence max de décodage JPEG par caméra (~12 Hz)
const TRAIL_MIN_STEP_M = 0.03;
const TRAIL_MAX_POINTS = 4000;
const ZOOM_MIN = 12, ZOOM_MAX = 220; // px/m

// ---------------------------------------------------------------------------
// IPM WebGL : un petit contexte par caméra projetée. Le fragment shader fait
// la projection inverse exacte : pour chaque point du sol (x latéral,
// y avant), il calcule le pixel image qui le voit (caméra sténopé à hauteur
// h, inclinée de pitch vers le bas) et l'échantillonne -- perspective
// correcte par construction, sur GPU.

const VERT_SRC = `
attribute vec2 aPos;
uniform float uHalfW, uNear, uFar;
varying vec2 vGround;
void main() {
	vGround = vec2(aPos.x * uHalfW, mix(uNear, uFar, (aPos.y + 1.0) * 0.5));
	gl_Position = vec4(aPos, 0.0, 1.0);
}`;

const FRAG_SRC = `
precision mediump float;
varying vec2 vGround;
uniform float uH, uPitch, uFx, uFy, uNear, uFar, uFlip;
uniform sampler2D uTex;
void main() {
	float s = sin(uPitch), c = cos(uPitch);
	// Point sol P=(x, y, 0) vu par la caméra C=(0, 0, h) : d = P - C.
	// Axes caméra (regard vers l'avant, incliné de pitch vers le bas) :
	//   droite=(1,0,0)  bas image=(0,-s,-c)  avant optique=(0,c,-s)
	float xc = vGround.x;
	float yc = -vGround.y * s + uH * c;
	float zc = vGround.y * c + uH * s;
	if (zc <= 0.01) discard;
	vec2 uv = vec2(uFx * xc / zc + 0.5, uFy * yc / zc + 0.5);
	if (uFlip > 0.5) uv = vec2(1.0) - uv;
	if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) discard;
	vec3 col = texture2D(uTex, uv).rgb;
	// Confiance décroissante vers la portée max (pixels très étirés) et
	// fondu d'attaque au pied de la caméra.
	float a = (1.0 - smoothstep(uFar * 0.72, uFar, vGround.y))
	        * smoothstep(uNear, uNear + 0.08, vGround.y);
	gl_FragColor = vec4(col, a);
}`;

function createIpmRenderer() {
	const canvas = document.createElement("canvas");
	canvas.width = GL_W;
	canvas.height = GL_H;
	// preserveDrawingBuffer : le canvas GL est relu par drawImage() hors du
	// tick de rendu (mosaïque + vue live). premultipliedAlpha:false : le
	// shader sort une alpha franche, pas prémultipliée.
	const gl = canvas.getContext("webgl", {
		alpha: true, premultipliedAlpha: false, preserveDrawingBuffer: true,
		antialias: false, depth: false, stencil: false,
	});
	if (!gl) return null;

	const compile = (type, src) => {
		const sh = gl.createShader(type);
		gl.shaderSource(sh, src);
		gl.compileShader(sh);
		if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
			console.error("[birdview] shader:", gl.getShaderInfoLog(sh));
			return null;
		}
		return sh;
	};
	const vs = compile(gl.VERTEX_SHADER, VERT_SRC);
	const fs = compile(gl.FRAGMENT_SHADER, FRAG_SRC);
	if (!vs || !fs) return null;
	const prog = gl.createProgram();
	gl.attachShader(prog, vs);
	gl.attachShader(prog, fs);
	gl.linkProgram(prog);
	if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
		console.error("[birdview] link:", gl.getProgramInfoLog(prog));
		return null;
	}
	gl.useProgram(prog);

	const buf = gl.createBuffer();
	gl.bindBuffer(gl.ARRAY_BUFFER, buf);
	gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
	const aPos = gl.getAttribLocation(prog, "aPos");
	gl.enableVertexAttribArray(aPos);
	gl.vertexAttribPointer(aPos, 2, gl.FLOAT, false, 0, 0);

	const tex = gl.createTexture();
	gl.bindTexture(gl.TEXTURE_2D, tex);
	// NPOT (frames JPEG de taille arbitraire) : clamp + linear, pas de mips.
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);

	const U = {};
	for (const name of ["uHalfW", "uNear", "uFar", "uH", "uPitch", "uFx", "uFy", "uFlip", "uTex"]) {
		U[name] = gl.getUniformLocation(prog, name);
	}
	gl.uniform1i(U.uTex, 0);
	gl.clearColor(0, 0, 0, 0);

	return {
		canvas,
		upload(bitmap) {
			gl.bindTexture(gl.TEXTURE_2D, tex);
			gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, bitmap);
		},
		render(g, flip) {
			gl.viewport(0, 0, GL_W, GL_H);
			gl.clear(gl.COLOR_BUFFER_BIT);
			gl.uniform1f(U.uHalfW, g.halfW);
			gl.uniform1f(U.uNear, g.near);
			gl.uniform1f(U.uFar, g.far);
			gl.uniform1f(U.uH, g.h);
			gl.uniform1f(U.uPitch, g.pitch);
			gl.uniform1f(U.uFx, g.fx);
			gl.uniform1f(U.uFy, g.fy);
			gl.uniform1f(U.uFlip, flip ? 1 : 0);
			gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
		},
	};
}

// Empreinte au sol visible par une caméra (hauteur h, inclinaison pitch vers
// le bas, FOV horizontal) pour une image de ratio texH/texW : bornes
// near/far le long de l'axe avant, demi-largeur latérale au niveau de far.
function ipmGeometry(camCfg, texW, texH) {
	const h = Math.max(0.05, camCfg.heightM);
	const pitch = Math.min(85, Math.max(5, camCfg.pitchDeg)) * DEG;
	const tanH = Math.tan((Math.min(120, Math.max(30, camCfg.hfovDeg)) * DEG) / 2);
	const tanV = tanH * (texH / texW);
	const vHalf = Math.atan(tanV);
	const downNear = pitch + vHalf; // dépression du bord bas de l'image
	const downFar = Math.max(pitch - vHalf, 4 * DEG); // bord haut, borné loin de l'horizon
	const near = downNear >= 89 * DEG ? 0.02 : Math.max(0.02, h / Math.tan(downNear));
	const far = Math.max(Math.min(FAR_CAP_M, h / Math.tan(downFar)), near + 0.15);
	const halfW = (far * Math.cos(pitch) + h * Math.sin(pitch)) * tanH;
	const nearHalfW = (near * Math.cos(pitch) + h * Math.sin(pitch)) * tanH;
	return { h, pitch, near, far, halfW, nearHalfW, fx: 0.5 / tanH, fy: 0.5 / tanV };
}

export function initBirdview({ mux, status, control }) {
	const $ = (id) => document.getElementById(id);
	const container = $("spatialView");
	const canvas = $("spatialCanvas");
	const hud = $("spatialHud");
	if (!container || !canvas) return;
	const ctx = canvas.getContext("2d");

	// ---- État -------------------------------------------------------------
	const pose = { x: 0, y: 0, yaw: 0 };      // mètres monde / rad
	const lastVel = { vx: 0, vy: 0, wz: 0 };  // normalisé, pour le vecteur du glyphe
	const trail = [{ x: 0, y: 0 }];
	const pan = { x: 0, y: 0 };               // décalage de vue (mode carte uniquement)
	let zoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, config.get("birdview.pxPerM") || 46));
	let lastTick = performance.now();
	let lastStamp = 0;
	let lastFade = performance.now();
	let lastHud = 0;

	// Mosaïque monde + tampon de recentrage (jamais de drawImage d'un canvas
	// sur lui-même).
	const map = document.createElement("canvas");
	map.width = map.height = MAP_SIZE;
	const mapCtx = map.getContext("2d");
	const mapSwap = document.createElement("canvas");
	mapSwap.width = mapSwap.height = MAP_SIZE;
	const mapSwapCtx = mapSwap.getContext("2d");
	let mapOrigin = { x: 0, y: 0 };            // coordonnées monde du centre de la mosaïque

	// ---- Flux caméra ------------------------------------------------------
	// front / rear : reprojetés au sol (IPM). gripper : médaillon.
	const makeFeed = () => ({
		id: null, jpeg: null, seq: 0, decodedSeq: -1, busy: false, lastDecode: 0,
		lastFrameAt: 0, bitmap: null, geom: null, ready: false,
	});
	const feeds = { front: makeFeed(), rear: makeFeed(), gripper: makeFeed() };
	feeds.front.ipm = createIpmRenderer();
	feeds.rear.ipm = createIpmRenderer();
	if (!feeds.front.ipm || !feeds.rear.ipm) {
		// Sans WebGL, la vue reste utile (pose, trace, glyphe, médaillon) --
		// seules les nappes sol manquent.
		console.warn("[birdview] WebGL indisponible : projections sol désactivées");
	}

	function resolveRoleId(path, autoResolve) {
		const cameras = mux.getCameras();
		const configured = config.get(path);
		if (configured === -2) return null;
		if (configured !== -1 && cameras.some((c) => c.id === configured)) return configured;
		return autoResolve(cameras);
	}
	function recomputeTargets() {
		feeds.front.id = resolveRoleId("birdview.frontId", (cams) => resolvePrimaryId(cams));
		feeds.rear.id = resolveRoleId("birdview.rearId",
			(cams) => resolveSecondaryId(cams, resolvePrimaryId(cams)));
		feeds.gripper.id = resolveRoleId("birdview.gripperId", (cams) => {
			const p = resolvePrimaryId(cams);
			return resolveTertiaryId(cams, p, resolveSecondaryId(cams, p));
		});
	}
	mux.onCameraList(recomputeTargets);
	config.subscribe((path) => {
		if (path.startsWith("birdview.") || path.startsWith("cameras.")) recomputeTargets();
	});

	const enabled = () => config.get("birdview.enabled");
	mux.onAnyFrame((camId, jpegBytes) => {
		if (!enabled()) return;
		for (const feed of Object.values(feeds)) {
			if (camId === feed.id) {
				feed.jpeg = jpegBytes;
				feed.seq++;
				feed.lastFrameAt = performance.now();
			}
		}
	});

	// Décodage "dernière frame seulement", throttlé (même philosophie que
	// camera.js : ne jamais accumuler de retard, quitte à sauter des frames).
	function pumpFeed(feed, onBitmap) {
		if (feed.busy || feed.seq === feed.decodedSeq || !feed.jpeg) return;
		const now = performance.now();
		if (now - feed.lastDecode < DECODE_EVERY_MS) return;
		feed.busy = true;
		feed.lastDecode = now;
		const seq = feed.seq;
		createImageBitmap(new Blob([feed.jpeg], { type: "image/jpeg" }))
			.then((bmp) => { feed.decodedSeq = seq; onBitmap(bmp); })
			.catch(() => { /* frame corrompue : la suivante arrive */ })
			.finally(() => { feed.busy = false; });
	}

	function pumpGroundFeed(feed, cfgPrefix, flipPath) {
		if (!feed.ipm) return;
		pumpFeed(feed, (bmp) => {
			feed.geom = ipmGeometry(config.get(cfgPrefix), bmp.width, bmp.height);
			feed.ipm.upload(bmp);
			feed.ipm.render(feed.geom, config.get(flipPath));
			feed.ready = true;
			bmp.close();
		});
	}

	// ---- Dead-reckoning ----------------------------------------------------
	function integratePose(dt) {
		const snap = status.getSnapshot() || {};
		const st = snap.state || {};
		let vx = 0, vy = 0, wz = 0;
		if (st.vel && snap.robot) {
			vx = st.vel.vx || 0; vy = st.vel.vy || 0; wz = st.vel.wz || 0;
		} else if (!st.vel) {
			// robot_agent.py sans "vel" : repli sur la commande de CE
			// navigateur (déjà gatée deadman + vitesse dans control.js).
			const cmd = control.getLastCmd();
			vx = cmd.vx; vy = cmd.vy; wz = cmd.wz;
		}
		lastVel.vx = vx; lastVel.vy = vy; lastVel.wz = wz;
		const vScale = config.get("birdview.speedFullMS");
		const wScale = config.get("birdview.rotFullRadS");
		// Intégration au point milieu du cap : réduit le biais en virage
		// par rapport à Euler avant, gratuit à calculer.
		const midYaw = pose.yaw + (wz * wScale * dt) / 2;
		const c = Math.cos(midYaw), s = Math.sin(midYaw);
		pose.x += (-s * vx + c * vy) * vScale * dt;
		pose.y += (c * vx + s * vy) * vScale * dt;
		pose.yaw += wz * wScale * dt;

		const last = trail[trail.length - 1];
		if (Math.hypot(pose.x - last.x, pose.y - last.y) > TRAIL_MIN_STEP_M) {
			trail.push({ x: pose.x, y: pose.y });
			if (trail.length > TRAIL_MAX_POINTS) trail.splice(0, trail.length - TRAIL_MAX_POINTS);
		}
	}

	// ---- Mosaïque monde ----------------------------------------------------
	// Transforme mapCtx en "mètres monde, y vers le haut", centré sur mapOrigin.
	function withMapTransform(fn) {
		mapCtx.save();
		mapCtx.setTransform(MAP_PPM, 0, 0, -MAP_PPM,
			MAP_SIZE / 2 - mapOrigin.x * MAP_PPM,
			MAP_SIZE / 2 + mapOrigin.y * MAP_PPM);
		fn(mapCtx);
		mapCtx.restore();
	}

	// Dessine la nappe sol d'une caméra dans un contexte déjà en "mètres
	// monde y-haut". dir = +1 caméra avant, -1 caméra arrière (montée dos à
	// la route : sa nappe s'étend derrière le robot).
	function drawGroundSheet(c2d, feed, dir) {
		if (!feed.ready || !feed.geom) return;
		const g = feed.geom;
		const off = config.get(dir > 0 ? "birdview.front.offsetM" : "birdview.rear.offsetM");
		c2d.save();
		c2d.translate(pose.x, pose.y);
		c2d.rotate(pose.yaw);              // local : +y = avant robot
		c2d.translate(0, dir * off);
		if (dir < 0) c2d.rotate(Math.PI);
		// Image IPM : rangée haute = far, rangée basse = near, x = latéral.
		c2d.translate(0, g.far);
		c2d.scale((2 * g.halfW) / GL_W, (g.near - g.far) / GL_H);
		c2d.drawImage(feed.ipm.canvas, -GL_W / 2, 0);
		c2d.restore();
	}

	function stampIntoMap(now) {
		if (now - lastStamp < STAMP_EVERY_MS) return;
		if (!feeds.front.ready && !feeds.rear.ready) return;
		lastStamp = now;
		// Recentrage préalable si le robot approche du bord de la mosaïque.
		if (Math.abs(pose.x - mapOrigin.x) > MAP_EXTENT_M / 2 - MAP_MARGIN_M
			|| Math.abs(pose.y - mapOrigin.y) > MAP_EXTENT_M / 2 - MAP_MARGIN_M) {
			const dxPx = Math.round((mapOrigin.x - pose.x) * MAP_PPM);
			const dyPx = Math.round((pose.y - mapOrigin.y) * MAP_PPM);
			mapSwapCtx.clearRect(0, 0, MAP_SIZE, MAP_SIZE);
			mapSwapCtx.drawImage(map, dxPx, dyPx);
			mapCtx.setTransform(1, 0, 0, 1, 0, 0);
			mapCtx.clearRect(0, 0, MAP_SIZE, MAP_SIZE);
			mapCtx.drawImage(mapSwap, 0, 0);
			mapOrigin = { x: pose.x, y: pose.y };
		}
		withMapTransform((c2d) => {
			c2d.globalAlpha = 0.9;
			drawGroundSheet(c2d, feeds.front, +1);
			drawGroundSheet(c2d, feeds.rear, -1);
		});
	}

	function fadeMap(now) {
		const halflife = config.get("birdview.fadeHalflifeS");
		const dt = (now - lastFade) / 1000;
		if (dt < 0.4) return;
		lastFade = now;
		if (!halflife) return;
		mapCtx.save();
		mapCtx.setTransform(1, 0, 0, 1, 0, 0);
		mapCtx.globalCompositeOperation = "destination-out";
		mapCtx.fillStyle = `rgba(0,0,0,${1 - Math.pow(0.5, dt / halflife)})`;
		mapCtx.fillRect(0, 0, MAP_SIZE, MAP_SIZE);
		mapCtx.restore();
	}

	function clearMap() {
		mapCtx.setTransform(1, 0, 0, 1, 0, 0);
		mapCtx.clearRect(0, 0, MAP_SIZE, MAP_SIZE);
		trail.length = 0;
		trail.push({ x: pose.x, y: pose.y });
	}

	// ---- Vue (canvas d'affichage) -------------------------------------------
	const view = () => ({
		headingUp: !config.get("birdview.northUp"),
		// En mode cap, le robot est placé sous le centre : plus de champ devant.
		anchorY: config.get("birdview.northUp") ? 0.5 : 0.62,
	});

	function viewCenterWorld(v) {
		return v.headingUp
			? { x: pose.x, y: pose.y }
			: { x: pose.x + pan.x, y: pose.y + pan.y };
	}

	// Applique monde (m, y-haut) -> pixels du canvas. À utiliser sous save().
	function applyViewTransform(v, w, h, ppm) {
		const c = viewCenterWorld(v);
		ctx.translate(w / 2, h * v.anchorY);
		ctx.scale(ppm, -ppm);
		if (v.headingUp) ctx.rotate(-pose.yaw);
		ctx.translate(-c.x, -c.y);
	}

	function worldToScreen(v, w, h, ppm, wx, wy) {
		const c = viewCenterWorld(v);
		let dx = wx - c.x, dy = wy - c.y;
		if (v.headingUp) {
			const co = Math.cos(-pose.yaw), si = Math.sin(-pose.yaw);
			[dx, dy] = [co * dx - si * dy, si * dx + co * dy];
		}
		return [w / 2 + dx * ppm, h * v.anchorY - dy * ppm];
	}

	function drawGrid(v, w, h, ppm) {
		const c = viewCenterWorld(v);
		const radius = Math.hypot(w, h) / 2 / ppm + 1;
		ctx.lineWidth = 1 / ppm;
		// Grille cartésienne 1 m alignée monde.
		ctx.strokeStyle = "rgba(255,255,255,0.055)";
		ctx.beginPath();
		for (let gx = Math.floor(c.x - radius); gx <= Math.ceil(c.x + radius); gx++) {
			ctx.moveTo(gx, c.y - radius);
			ctx.lineTo(gx, c.y + radius);
		}
		for (let gy = Math.floor(c.y - radius); gy <= Math.ceil(c.y + radius); gy++) {
			ctx.moveTo(c.x - radius, gy);
			ctx.lineTo(c.x + radius, gy);
		}
		ctx.stroke();
		// Anneaux de portée autour du robot.
		ctx.strokeStyle = "rgba(76,157,240,0.10)";
		ctx.beginPath();
		for (let r = 1; r <= radius + 1; r++) {
			ctx.moveTo(pose.x + r, pose.y);
			ctx.arc(pose.x, pose.y, r, 0, Math.PI * 2);
		}
		ctx.stroke();
	}

	function drawMosaic() {
		ctx.save();
		ctx.translate(mapOrigin.x, mapOrigin.y);
		ctx.scale(1 / MAP_PPM, -1 / MAP_PPM);
		ctx.drawImage(map, -MAP_SIZE / 2, -MAP_SIZE / 2);
		ctx.restore();
	}

	function drawTrail(ppm) {
		if (trail.length < 2) return;
		ctx.strokeStyle = "rgba(76,157,240,0.55)";
		ctx.lineWidth = 2 / ppm;
		ctx.lineJoin = "round";
		ctx.beginPath();
		ctx.moveTo(trail[0].x, trail[0].y);
		for (const p of trail) ctx.lineTo(p.x, p.y);
		ctx.lineTo(pose.x, pose.y);
		ctx.stroke();
	}

	// Contour du cône de vision d'une caméra (trapèze IPM), style "capteur".
	function drawSheetOutline(feed, dir, ppm, color) {
		if (!feed.ready || !feed.geom) return;
		const g = feed.geom;
		const off = config.get(dir > 0 ? "birdview.front.offsetM" : "birdview.rear.offsetM");
		ctx.save();
		ctx.translate(pose.x, pose.y);
		ctx.rotate(pose.yaw);
		ctx.translate(0, dir * off);
		if (dir < 0) ctx.rotate(Math.PI);
		ctx.strokeStyle = color;
		ctx.lineWidth = 1.2 / ppm;
		ctx.beginPath();
		ctx.moveTo(-g.nearHalfW, g.near);
		ctx.lineTo(-g.halfW, g.far);
		ctx.lineTo(g.halfW, g.far);
		ctx.lineTo(g.nearHalfW, g.near);
		ctx.closePath();
		ctx.stroke();
		ctx.restore();
	}

	function pathRoundRect(x, y, w, h, r) {
		if (ctx.roundRect) { ctx.roundRect(x, y, w, h, r); return; }
		ctx.rect(x, y, w, h); // repli très vieux navigateurs : coins droits
	}

	function drawRobot(ppm) {
		const L = config.get("birdview.robotLengthM");
		const W = config.get("birdview.robotWidthM");
		ctx.save();
		ctx.translate(pose.x, pose.y);
		ctx.rotate(pose.yaw); // local : x = droite, y = avant

		// Halo de présence.
		const halo = ctx.createRadialGradient(0, 0, 0, 0, 0, Math.max(L, W));
		halo.addColorStop(0, "rgba(76,157,240,0.20)");
		halo.addColorStop(1, "rgba(76,157,240,0)");
		ctx.fillStyle = halo;
		ctx.beginPath();
		ctx.arc(0, 0, Math.max(L, W), 0, Math.PI * 2);
		ctx.fill();

		// 4 roues mecanum aux coins.
		ctx.fillStyle = "rgba(255,255,255,0.28)";
		const ww = W * 0.16, wl = L * 0.28;
		for (const [sx, sy] of [[-1, 1], [1, 1], [-1, -1], [1, -1]]) {
			ctx.beginPath();
			pathRoundRect(sx * W / 2 - ww / 2, sy * L * 0.30 - wl / 2, ww, wl, ww * 0.4);
			ctx.fill();
		}

		// Châssis.
		ctx.beginPath();
		pathRoundRect(-W / 2 + W * 0.06, -L / 2, W - W * 0.12, L, W * 0.14);
		ctx.fillStyle = "rgba(20,26,34,0.92)";
		ctx.fill();
		ctx.lineWidth = 1.6 / ppm;
		ctx.strokeStyle = "rgba(76,157,240,0.9)";
		ctx.stroke();

		// Flèche de cap.
		ctx.beginPath();
		ctx.moveTo(0, L * 0.34);
		ctx.lineTo(-W * 0.16, L * 0.10);
		ctx.lineTo(W * 0.16, L * 0.10);
		ctx.closePath();
		ctx.fillStyle = "#4c9df0";
		ctx.fill();

		// Vecteur vitesse instantané (normalisé -> longueur max ~0.8 m) :
		// local x = droite = vy, local y = avant = vx.
		const vmag = Math.hypot(lastVel.vx, lastVel.vy);
		if (vmag > 0.03) {
			ctx.beginPath();
			ctx.moveTo(0, 0);
			ctx.lineTo(lastVel.vy * 0.8, lastVel.vx * 0.8);
			ctx.strokeStyle = "rgba(35,168,132,0.95)";
			ctx.lineWidth = 3 / ppm;
			ctx.lineCap = "round";
			ctx.stroke();
		}
		// Arc de rotation commandée.
		if (Math.abs(lastVel.wz) > 0.05) {
			ctx.beginPath();
			// wz CCW+ ; le repère du canvas est déjà y-haut, donc CCW visuel = CCW math.
			const a = lastVel.wz * 1.2;
			ctx.arc(0, 0, L * 0.62, Math.PI / 2, Math.PI / 2 + a, a < 0);
			ctx.strokeStyle = "rgba(154,140,240,0.9)";
			ctx.lineWidth = 2.5 / ppm;
			ctx.stroke();
		}
		ctx.restore();
	}

	// Médaillon caméra pince : accroché devant le glyphe, en pixels écran
	// (taille lisible quel que soit le zoom).
	function drawGripperInset(v, w, h, ppm, dpr, now) {
		const feed = feeds.gripper;
		if (!config.get("birdview.showGripperInset")) return;
		if (!feed.bitmap || now - feed.lastFrameAt > config.get("ui.staleMs") * 3) return;
		const L = config.get("birdview.robotLengthM");
		const d = L / 2 + 0.42; // ancré un peu devant le châssis
		const [sx, sy] = worldToScreen(v, w, h, ppm,
			pose.x - Math.sin(pose.yaw) * d, pose.y + Math.cos(pose.yaw) * d);
		const r = Math.min(Math.max(0.30 * ppm, 24 * dpr), Math.min(w, h) * 0.16);
		if (sx < -r || sy < -r || sx > w + r || sy > h + r) return;
		ctx.save();
		ctx.beginPath();
		ctx.arc(sx, sy, r, 0, Math.PI * 2);
		ctx.clip();
		// cover : crop carré centré de la frame.
		const b = feed.bitmap;
		const side = Math.min(b.width, b.height);
		ctx.drawImage(b, (b.width - side) / 2, (b.height - side) / 2, side, side,
			sx - r, sy - r, 2 * r, 2 * r);
		ctx.restore();
		ctx.beginPath();
		ctx.arc(sx, sy, r, 0, Math.PI * 2);
		ctx.strokeStyle = "rgba(35,168,132,0.9)";
		ctx.lineWidth = 2 * dpr;
		ctx.stroke();
		ctx.fillStyle = "rgba(35,168,132,0.9)";
		ctx.font = `600 ${10 * dpr}px system-ui, sans-serif`;
		ctx.textAlign = "center";
		ctx.fillText("PINCE", sx, sy + r + 12 * dpr);
	}

	function drawCompass(v, dpr) {
		// Rose des vents en haut à gauche : N tourne avec la carte en mode cap.
		const cx2 = 22 * dpr, cy2 = 22 * dpr, r = 13 * dpr;
		const a = v.headingUp ? pose.yaw : 0; // angle écran du nord
		ctx.save();
		ctx.translate(cx2, cy2);
		ctx.beginPath();
		ctx.arc(0, 0, r, 0, Math.PI * 2);
		ctx.fillStyle = "rgba(0,0,0,0.45)";
		ctx.fill();
		ctx.rotate(a);
		ctx.beginPath();
		ctx.moveTo(0, -r + 3 * dpr);
		ctx.lineTo(-3.5 * dpr, 2 * dpr);
		ctx.lineTo(3.5 * dpr, 2 * dpr);
		ctx.closePath();
		ctx.fillStyle = "#e5484d";
		ctx.fill();
		ctx.rotate(-a);
		ctx.fillStyle = "rgba(255,255,255,0.75)";
		ctx.font = `600 ${8 * dpr}px system-ui, sans-serif`;
		ctx.textAlign = "center";
		ctx.fillText("N", 0, -r - 3 * dpr);
		ctx.restore();
	}

	function drawScaleBar(w, h, ppm, dpr) {
		// Barre d'échelle 1 m (ou 0.5 m si trop large), bas-centre.
		let meters = 1, px = ppm;
		if (px > w * 0.4) { meters = 0.5; px = ppm / 2; }
		const y = h - 10 * dpr, x0 = w / 2 - px / 2;
		ctx.strokeStyle = "rgba(255,255,255,0.55)";
		ctx.lineWidth = 1.5 * dpr;
		ctx.beginPath();
		ctx.moveTo(x0, y - 3 * dpr); ctx.lineTo(x0, y);
		ctx.lineTo(x0 + px, y); ctx.lineTo(x0 + px, y - 3 * dpr);
		ctx.stroke();
		ctx.fillStyle = "rgba(255,255,255,0.6)";
		ctx.font = `${9 * dpr}px system-ui, sans-serif`;
		ctx.textAlign = "center";
		ctx.fillText(meters === 1 ? "1 m" : "0,5 m", w / 2, y - 5 * dpr);
	}

	function updateHud(now) {
		if (now - lastHud < 250 || !hud) return;
		lastHud = now;
		const capDeg = ((-pose.yaw / DEG) % 360 + 360) % 360; // cap boussole (horaire depuis N)
		const snap = status.getSnapshot() || {};
		const mastMm = snap.mast && snap.mast.position_mm;
		const velFromRobot = !!(snap.state && snap.state.vel);
		hud.textContent =
			`x ${pose.x >= 0 ? "+" : ""}${pose.x.toFixed(1)} m · ` +
			`y ${pose.y >= 0 ? "+" : ""}${pose.y.toFixed(1)} m · ` +
			`cap ${capDeg.toFixed(0)}°` +
			(mastMm != null ? ` · mât ${mastMm.toFixed(0)} mm` : "") +
			(velFromRobot ? "" : " · odom. navigateur");
	}

	// ---- Boucle principale ---------------------------------------------------
	function frame() {
		requestAnimationFrame(frame);
		const now = performance.now();
		const dt = Math.min((now - lastTick) / 1000, 0.1);
		lastTick = now;
		if (!enabled()) return;

		integratePose(dt);
		pumpGroundFeed(feeds.front, "birdview.front", "birdview.frontRotate180");
		pumpGroundFeed(feeds.rear, "birdview.rear", "birdview.rearRotate180");
		if (config.get("birdview.showGripperInset")) {
			pumpFeed(feeds.gripper, (bmp) => {
				if (feeds.gripper.bitmap) feeds.gripper.bitmap.close();
				feeds.gripper.bitmap = bmp;
			});
		}
		fadeMap(now);
		stampIntoMap(now);

		// Taille du canvas = taille CSS x dpr (l'overlay peut être agrandi).
		const dpr = window.devicePixelRatio || 1;
		const w = Math.max(1, Math.round(canvas.clientWidth * dpr));
		const h = Math.max(1, Math.round(canvas.clientHeight * dpr));
		if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
		const ppm = zoom * dpr;
		const v = view();

		ctx.setTransform(1, 0, 0, 1, 0, 0);
		ctx.clearRect(0, 0, w, h);
		ctx.fillStyle = "rgba(5,7,10,0.85)";
		ctx.fillRect(0, 0, w, h);

		ctx.save();
		applyViewTransform(v, w, h, ppm);
		drawGrid(v, w, h, ppm);
		drawMosaic();
		// Nappes live par-dessus la mosaïque : pleine confiance sur l'instant.
		drawGroundSheet(ctx, feeds.front, +1);
		drawGroundSheet(ctx, feeds.rear, -1);
		drawSheetOutline(feeds.front, +1, ppm, "rgba(76,157,240,0.35)");
		drawSheetOutline(feeds.rear, -1, ppm, "rgba(154,140,240,0.30)");
		drawTrail(ppm);
		drawRobot(ppm);
		ctx.restore();

		drawGripperInset(v, w, h, ppm, dpr, now);
		drawCompass(v, dpr);
		drawScaleBar(w, h, ppm, dpr);
		updateHud(now);
	}
	requestAnimationFrame(frame);

	// ---- Interactions ----------------------------------------------------------
	// Molette = zoom (persisté), glisser = panoramique (mode carte seulement).
	container.addEventListener("wheel", (e) => {
		e.preventDefault();
		zoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, zoom * Math.pow(1.0015, -e.deltaY)));
		clearTimeout(container._zoomSave);
		container._zoomSave = setTimeout(() => config.set("birdview.pxPerM", Math.round(zoom)), 400);
	}, { passive: false });

	let drag = null;
	canvas.addEventListener("pointerdown", (e) => {
		if (view().headingUp) return; // en mode cap le robot est ancré au centre
		drag = { x: e.clientX, y: e.clientY };
		canvas.setPointerCapture(e.pointerId);
	});
	canvas.addEventListener("pointermove", (e) => {
		if (!drag) return;
		pan.x -= (e.clientX - drag.x) / zoom;
		pan.y += (e.clientY - drag.y) / zoom;
		drag = { x: e.clientX, y: e.clientY };
	});
	const endDrag = () => { drag = null; };
	canvas.addEventListener("pointerup", endDrag);
	canvas.addEventListener("pointercancel", endDrag);

	const btnMode = $("spatialMode");
	const btnCenter = $("spatialCenter");
	const btnClear = $("spatialClear");
	const btnExpand = $("spatialExpand");
	const syncModeBtn = () => {
		const northUp = config.get("birdview.northUp");
		btnMode.classList.toggle("active", northUp);
		btnMode.title = northUp
			? "Mode carte (nord en haut) — cliquer pour repasser en mode cap"
			: "Mode cap (robot fixe, avant en haut) — cliquer pour passer en mode carte";
	};
	btnMode.addEventListener("click", () => {
		config.set("birdview.northUp", !config.get("birdview.northUp"));
		pan.x = pan.y = 0;
		syncModeBtn();
	});
	syncModeBtn();
	btnCenter.addEventListener("click", () => {
		pan.x = pan.y = 0;
		toast("Vue recentrée sur le robot");
	});
	btnClear.addEventListener("click", () => {
		pose.x = pose.y = pose.yaw = 0;
		pan.x = pan.y = 0;
		mapOrigin = { x: 0, y: 0 };
		clearMap(); // après la remise à zéro : la trace repart de la nouvelle origine
		toast("Mémoire spatiale réinitialisée — le robot repart de l'origine");
	});
	btnExpand.addEventListener("click", () => {
		container.classList.toggle("big");
		btnExpand.classList.toggle("active", container.classList.contains("big"));
	});
	canvas.addEventListener("dblclick", () => btnExpand.click());

	// ---- Visibilité --------------------------------------------------------------
	const applyEnabled = () => { container.hidden = !enabled(); };
	applyEnabled();
	config.subscribe((path) => { if (path === "birdview.enabled") applyEnabled(); });
}
