// Third-person 3D viewer — pure 2D-canvas line art, no WebGL.
//
// Orbit camera around the missile. Renders a *shaded* elevation-mesh terrain
// patch from the DEM (painter's-algorithm quads + wireframe), a detailed
// line-art Tomahawk Block V oriented by the flight state, the planned/flown
// trajectories, target/impact markers, and an AGL drop-line. The solid rocket
// booster is drawn attached during BOOST and separates with a code-triggered
// animation at the boost→cruise handoff. Redraws only on demand.

import { fitCanvas, cssVar, clamp } from "./util.js";

const DEG = Math.PI / 180;
const G = 9.80665;

// --- tiny vec3 helpers ------------------------------------------------------
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

// --- Tomahawk Block V line-art model (body frame: x=nose fwd, y=right, z=up) -
const R = 0.24;                          // fuselage radius (model units)
function ring(x, r, n = 14) {
  const p = [];
  for (let i = 0; i <= n; i++) { const a = (i / n) * 2 * Math.PI; p.push([x, r * Math.cos(a), r * Math.sin(a)]); }
  return p;
}
function buildTomahawk() {
  const parts = [];
  const noseTip = 3.4, bodyEnd = -2.6;
  // fuselage: cross-section rings + 4 longitudinals + nose cone
  parts.push(ring(2.6, R), ring(1.2, R), ring(-0.4, R), ring(-2.0, R), ring(bodyEnd, R * 0.85));
  for (const ang of [0, 90, 180, 270]) {
    const c = Math.cos(ang * DEG), s = Math.sin(ang * DEG);
    parts.push([[noseTip, 0, 0], [2.6, R * c, R * s], [bodyEnd, R * 0.85 * c, R * 0.85 * s]]);
  }
  // rounded nose (a few taper lines)
  for (const ang of [45, 135, 225, 315]) {
    const c = Math.cos(ang * DEG), s = Math.sin(ang * DEG);
    parts.push([[noseTip, 0, 0], [2.6, R * c, R * s]]);
  }
  // mid-body straight wings (slight sweep), mounted low-mid
  const wz = -0.04;
  parts.push([[0.55, R * 0.7, wz], [-0.05, 2.05, wz], [-0.55, 2.05, wz], [-0.55, R * 0.7, wz]]); // right
  parts.push([[0.55, -R * 0.7, wz], [-0.05, -2.05, wz], [-0.55, -2.05, wz], [-0.55, -R * 0.7, wz]]); // left
  // cruciform tail fins at the rear
  const fin = (dir) => { // dir is unit [y,z] direction
    const rootF = [-1.9, dir[0] * R, dir[1] * R], rootA = [-2.55, dir[0] * R, dir[1] * R];
    const tip = [-2.6, dir[0] * (R + 0.72), dir[1] * (R + 0.72)];
    parts.push([rootF, tip, rootA]);
  };
  fin([0, 1]); fin([0, -1]); fin([1, 0]); fin([-1, 0]);
  // belly air inlet scoop (rear underside)
  const iy = 0.17, iz = -R - 0.02, iz2 = -R - 0.26;
  parts.push([[-0.6, iy, iz], [-0.9, iy, iz2], [-1.9, iy, iz2], [-2.0, iy, iz]]);
  parts.push([[-0.6, -iy, iz], [-0.9, -iy, iz2], [-1.9, -iy, iz2], [-2.0, -iy, iz]]);
  parts.push([[-0.9, iy, iz2], [-0.9, -iy, iz2]], [[-1.9, iy, iz2], [-1.9, -iy, iz2]]);
  return parts;
}
function buildBooster() {
  const Rb = 0.3, x0 = -2.6, x1 = -4.3, parts = [];
  parts.push(ring(x0, Rb * 0.85), ring(-3.4, Rb), ring(x1, Rb));
  for (const ang of [0, 90, 180, 270]) {
    const c = Math.cos(ang * DEG), s = Math.sin(ang * DEG);
    parts.push([[x0, Rb * 0.85 * c, Rb * 0.85 * s], [x1, Rb * c, Rb * s]]);
  }
  // nozzle
  parts.push(ring(x1 - 0.25, Rb * 0.55));
  for (const ang of [0, 90, 180, 270]) {
    const c = Math.cos(ang * DEG), s = Math.sin(ang * DEG);
    parts.push([[x1, Rb * c, Rb * s], [x1 - 0.25, Rb * 0.55 * c, Rb * 0.55 * s]]);
  }
  // booster tail fins
  for (const d of [[0, 1], [0, -1], [1, 0], [-1, 0]]) {
    parts.push([[-3.6, d[0] * Rb, d[1] * Rb], [x1 - 0.1, d[0] * (Rb + 0.5), d[1] * (Rb + 0.5)], [x1, d[0] * Rb, d[1] * Rb]]);
  }
  return parts;
}
const TOMAHAWK = buildTomahawk();
const BOOSTER = buildBooster();

export class Viewer3D {
  constructor(canvas) {
    this.canvas = canvas;
    this.grid = null;
    this.origin = null;
    this.mPerLat = 111320;
    this.mPerLon = 111320;
    this.plan = null;
    this.path = [];
    this.target = null;
    this.start = null;
    this.frame = null;

    this.cam = { az: -0.7, el: 0.42, dist: 2600, target: [0, 0, 0] };
    this.vExag = 3;
    this.follow = true;
    this.showTerrain = true;
    this.detach = null;          // booster separation: { t, pos, f, r, u }
    this._dirty = true;
    this._raf = null;

    this._wire();
    this._loop();
    // Redraw at the new resolution on any container resize (fullscreen / resize).
    this._ro = new ResizeObserver(() => { this._dirty = true; });
    this._ro.observe(this.canvas);
  }

  // --- data -----------------------------------------------------------------
  setDEM(grid) {
    this.grid = grid;
    if (!this.origin && grid) { const b = grid.bounds; this._setOrigin((b.north + b.south) / 2, (b.west + b.east) / 2); }
    this._dirty = true;
  }
  _setOrigin(lat0, lon0) { this.origin = [lat0, lon0]; this.mPerLon = 111320 * Math.cos(lat0 * DEG); }
  _world(lat, lon, z) {
    if (!this.origin) this._setOrigin(lat, lon);
    return [(lon - this.origin[1]) * this.mPerLon, (lat - this.origin[0]) * this.mPerLat, z * this.vExag];
  }
  setStart(gps) { if (gps) { this._setOrigin(gps[0], gps[1]); this.start = this._world(gps[0], gps[1], gps[2]); this._dirty = true; } }
  setTarget(gps) { if (gps) { this.target = this._world(gps[0], gps[1], gps[2]); this._dirty = true; } }
  setPlan(traj) { this.plan = traj && traj.length ? traj.map((p) => this._world(p[0], p[1], p[2])) : null; this._dirty = true; }
  resetPath() { this.path = []; this.detach = null; this._dirty = true; }

  pushFrame(frame) {
    const prev = this.frame;
    this.frame = frame;
    const t = frame.true;
    const w = this._world(t.lat, t.lon, t.alt);
    const ground = t.ground_alt != null ? this._world(t.lat, t.lon, t.ground_alt) : [w[0], w[1], 0];
    this.path.push({ p: w, ground, stage: frame.stage });
    if (prev && prev.stage === "BOOST" && frame.stage !== "BOOST") this._markDetach(frame);
    if (this.follow) this.cam.target = w;
    this._dirty = true;
  }
  setPathFrames(frames) {
    this.path = frames.map((f) => {
      const t = f.true;
      return { p: this._world(t.lat, t.lon, t.alt), ground: t.ground_alt != null ? this._world(t.lat, t.lon, t.ground_alt) : null, stage: f.stage };
    });
    // find booster separation deterministically for scrubbing
    this.detach = null;
    for (let i = 1; i < frames.length; i++) {
      if (frames[i - 1].stage === "BOOST" && frames[i].stage !== "BOOST") { this.frame = frames[i]; this._markDetach(frames[i]); break; }
    }
    if (frames.length) { this.frame = frames[frames.length - 1]; if (this.follow) this.cam.target = this.path[this.path.length - 1].p; }
    this._dirty = true;
  }
  _markDetach(frame) {
    const t = frame.true, pos = this._world(t.lat, t.lon, t.alt);
    const { f, r, u } = this._attitudeBasis(frame);
    this.detach = { t: frame.t, pos, f, r, u };
  }

  resetView() { this.cam.az = -0.7; this.cam.el = 0.42; this.cam.dist = 2600; this._dirty = true; }
  frameCamera() {
    const pts = [];
    if (this.plan) pts.push(...this.plan);
    if (this.start) pts.push(this.start);
    if (this.target) pts.push(this.target);
    if (!pts.length) { this.resetView(); return; }
    let max = 800; const c = this.cam.target;
    for (const p of pts) max = Math.max(max, len(sub(p, c)));
    this.follow = false;
    if (this.start && this.target) this.cam.target = scale(add(this.start, this.target), 0.5);
    this.cam.dist = clamp(max * 2.2, 1200, 250000);
    this._dirty = true;
  }

  // --- interaction ----------------------------------------------------------
  _wire() {
    const c = this.canvas;
    let dragging = false, lx = 0, ly = 0;
    c.style.touchAction = "none";
    c.addEventListener("pointerdown", (e) => { dragging = true; lx = e.clientX; ly = e.clientY; c.setPointerCapture(e.pointerId); });
    c.addEventListener("pointermove", (e) => {
      if (!dragging) return;
      this.cam.az -= (e.clientX - lx) * 0.006;
      this.cam.el = clamp(this.cam.el + (e.clientY - ly) * 0.006, -1.45, 1.45);
      lx = e.clientX; ly = e.clientY; this._dirty = true;
    });
    c.addEventListener("pointerup", (e) => { dragging = false; c.releasePointerCapture?.(e.pointerId); });
    c.addEventListener("wheel", (e) => { e.preventDefault(); this.cam.dist = clamp(this.cam.dist * (1 + Math.sign(e.deltaY) * 0.12), 300, 120000); this._dirty = true; }, { passive: false });
    window.addEventListener("themechange", () => { this._dirty = true; });
  }

  _basis() {
    const { az, el } = this.cam;
    const dir = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
    const eye = add(this.cam.target, scale(dir, this.cam.dist));
    const forward = norm(sub(this.cam.target, eye));
    const right = norm(cross(forward, [0, 0, 1]));
    const up = cross(right, forward);
    return { eye, forward, right, up };
  }
  _project(p, view, W, H, focal) {
    const rel = sub(p, view.eye);
    const cz = dot(rel, view.forward);
    if (cz <= 1) return null;
    return [W / 2 + (dot(rel, view.right) / cz) * focal, H / 2 - (dot(rel, view.up) / cz) * focal, cz];
  }

  _loop() { const tick = () => { const anim = this.detach && this.frame && (this.frame.t - this.detach.t) < 4; if (this._dirty || anim) { this._dirty = false; this._render(); } this._raf = requestAnimationFrame(tick); }; this._raf = requestAnimationFrame(tick); }
  invalidate() { this._dirty = true; }
  destroy() { cancelAnimationFrame(this._raf); this._ro?.disconnect(); }

  _render() {
    const [W, H, , ctx] = fitCanvas(this.canvas);
    const g = ctx.createLinearGradient(0, 0, 0, H);
    g.addColorStop(0, cssVar("--instr-bg-2") || "#0d1420");
    g.addColorStop(1, cssVar("--instr-bg") || "#0a0f1a");
    ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);

    const view = this._basis();
    const focal = H * 0.9;
    const P = (p) => this._project(p, view, W, H, focal);

    if (this.showTerrain) this._drawTerrain(ctx, view, P);
    this._drawPlan(ctx, P);
    this._drawPath(ctx, P);
    this._drawMarkers(ctx, P);
    this._drawMissile(ctx, P);

    if (!this.grid) {
      ctx.fillStyle = cssVar("--instr-ink-dim") || "#7f8ba6"; ctx.font = "12px " + (cssVar("--font-mono") || "monospace");
      ctx.textAlign = "center"; ctx.fillText("No DEM loaded", W / 2, H / 2); ctx.textAlign = "left";
    }
  }

  // --- shaded elevation terrain --------------------------------------------
  _drawTerrain(ctx, view, P) {
    if (!this.grid || !this.frame) return;
    const grid = this.grid, b = grid.bounds, rows = grid.rows, cols = grid.cols, elev = grid.elev;
    const win = clamp(this.cam.dist * 0.85, 1400, 16000);
    const lat0 = this.frame.true.lat, lon0 = this.frame.true.lon;
    const dLat = win / this.mPerLat, dLon = win / this.mPerLon;
    const latToRow = (lat) => (b.north - lat) / (b.north - b.south) * (rows - 1);
    const lonToCol = (lon) => (lon - b.west) / (b.east - b.west) * (cols - 1);
    let r0 = clamp(Math.floor(latToRow(lat0 + dLat)), 0, rows - 1), r1 = clamp(Math.ceil(latToRow(lat0 - dLat)), 0, rows - 1);
    let c0 = clamp(Math.floor(lonToCol(lon0 - dLon)), 0, cols - 1), c1 = clamp(Math.ceil(lonToCol(lon0 + dLon)), 0, cols - 1);
    if (r1 - r0 < 2 || c1 - c0 < 2) return;

    const N = 30;
    const rStep = Math.max(1, Math.floor((r1 - r0) / N)), cStep = Math.max(1, Math.floor((c1 - c0) / N));
    const zRange = Math.max(1, grid.z_max - grid.z_min);
    const light = norm([-0.5, -0.35, 0.72]);

    const nodeAt = (ri, ci) => {
      const lat = b.north - (ri / (rows - 1)) * (b.north - b.south);
      const lon = b.west + (ci / (cols - 1)) * (b.east - b.west);
      const z = elev[ri * cols + ci];
      return { w: this._world(lat, lon, z), h: (z - grid.z_min) / zRange };
    };

    // build quads
    const quads = [];
    for (let ri = r0; ri < r1; ri += rStep) {
      const rn = Math.min(ri + rStep, r1);
      for (let ci = c0; ci < c1; ci += cStep) {
        const cn = Math.min(ci + cStep, c1);
        const a = nodeAt(ri, ci), bb = nodeAt(ri, cn), cc = nodeAt(rn, cn), dd = nodeAt(rn, ci);
        const pa = P(a.w), pb = P(bb.w), pc = P(cc.w), pd = P(dd.w);
        if (!pa || !pb || !pc || !pd) continue;
        const nrm = norm(cross(sub(bb.w, a.w), sub(dd.w, a.w)));
        const shade = clamp(Math.abs(dot(nrm, light)), 0, 1);
        const depth = pa[2] + pc[2];
        const h = (a.h + cc.h) / 2;
        quads.push({ pts: [pa, pb, pc, pd], shade, h, depth });
      }
    }
    quads.sort((q1, q2) => q2.depth - q1.depth); // back to front

    for (const q of quads) {
      const s = 0.32 + q.shade * 0.75;
      // subtle blue-grey terrain, lighter with elevation + light
      const r = clamp((26 + q.h * 74) * s, 6, 210);
      const gg = clamp((36 + q.h * 78) * s, 8, 215);
      const bl = clamp((52 + q.h * 70) * s, 12, 225);
      ctx.beginPath();
      ctx.moveTo(q.pts[0][0], q.pts[0][1]);
      for (let i = 1; i < 4; i++) ctx.lineTo(q.pts[i][0], q.pts[i][1]);
      ctx.closePath();
      ctx.fillStyle = `rgb(${r | 0},${gg | 0},${bl | 0})`;
      ctx.fill();
      ctx.strokeStyle = "rgba(150,180,220,0.10)"; ctx.lineWidth = 0.5; ctx.stroke();
    }
  }

  _poly(ctx, worldPts, color, width, P, glow = false) {
    if (!worldPts || worldPts.length < 2) return;
    ctx.save();
    if (glow) { ctx.shadowColor = color; ctx.shadowBlur = 6; }
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = "round";
    ctx.beginPath(); let started = false;
    for (const w of worldPts) { const p = P(w); if (!p) { started = false; continue; } if (!started) { ctx.moveTo(p[0], p[1]); started = true; } else ctx.lineTo(p[0], p[1]); }
    ctx.stroke(); ctx.restore();
  }

  _drawPlan(ctx, P) { if (this.plan) this._poly(ctx, this.plan, cssVar("--c-planned") || "#4d8dff", 1.6, P, true); }

  _drawPath(ctx, P) {
    if (this.path.length < 2) return;
    const colFor = (s) => s === "BOOST" ? cssVar("--c-boost") : (s === "TERMINAL" || s === "IMPACT") ? cssVar("--c-terminal") : cssVar("--c-actual");
    const last = this.path[this.path.length - 1];
    if (last.ground) this._poly(ctx, [last.p, last.ground], cssVar("--instr-ink-dim") || "#7f8ba6", 0.8, P);
    let seg = [this.path[0].p]; let cur = this.path[0].stage;
    for (let i = 1; i < this.path.length; i++) {
      seg.push(this.path[i].p);
      if (this.path[i].stage !== cur || i === this.path.length - 1) { this._poly(ctx, seg, colFor(cur), 2, P, true); seg = [this.path[i].p]; cur = this.path[i].stage; }
    }
  }

  _drawMarkers(ctx, P) {
    const mark = (w, color, label) => {
      if (!w) return; const p = P(w); if (!p) return;
      ctx.strokeStyle = color; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.arc(p[0], p[1], 6, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(p[0] - 9, p[1]); ctx.lineTo(p[0] + 9, p[1]); ctx.moveTo(p[0], p[1] - 9); ctx.lineTo(p[0], p[1] + 9); ctx.stroke();
      if (label) { ctx.fillStyle = color; ctx.font = "10px " + (cssVar("--font-mono") || "monospace"); ctx.fillText(label, p[0] + 10, p[1] - 8); }
    };
    mark(this.start, cssVar("--c-start") || "#32d74b", "LAUNCH");
    mark(this.target, cssVar("--c-target") || "#ff453a", "TARGET");
  }

  // --- attitude basis (forward/right/up) from a frame ----------------------
  _attitudeBasis(frame) {
    const yaw = (frame.att.yaw || 0) * DEG, pitch = (frame.att.fpa || 0) * DEG;
    const f = norm([Math.cos(pitch) * Math.sin(yaw), Math.cos(pitch) * Math.cos(yaw), Math.sin(pitch)]);
    const r = norm(cross(f, [0, 0, 1]));
    const u = cross(r, f);
    return { f, r, u };
  }

  _drawModel(ctx, parts, pos, basis, s, color, P, tumble = 0) {
    let { f, r, u } = basis;
    if (tumble) { // rotate about right axis for the tumbling booster
      const ca = Math.cos(tumble), sa = Math.sin(tumble);
      const f2 = add(scale(f, ca), scale(u, sa)), u2 = add(scale(u, ca), scale(f, -sa));
      f = f2; u = u2;
    }
    const toWorld = (bp) => add(pos, add(add(scale(f, bp[0] * s), scale(r, bp[1] * s)), scale(u, bp[2] * s)));
    ctx.save(); ctx.shadowColor = color; ctx.shadowBlur = 4;
    ctx.strokeStyle = color; ctx.lineWidth = 1.4; ctx.lineJoin = "round";
    for (const seg of parts) {
      ctx.beginPath(); let started = false;
      for (const bp of seg) { const p = P(toWorld(bp)); if (!p) { started = false; continue; } if (!started) { ctx.moveTo(p[0], p[1]); started = true; } else ctx.lineTo(p[0], p[1]); }
      ctx.stroke();
    }
    ctx.restore();
  }

  _drawMissile(ctx, P) {
    if (!this.frame) return;
    const t = this.frame.true;
    const pos = this._world(t.lat, t.lon, t.alt);
    const basis = this._attitudeBasis(this.frame);
    const s = clamp(this.cam.dist * 0.035, 10, 500);
    const stage = this.frame.stage;
    const color = stage === "TERMINAL" || stage === "IMPACT" ? (cssVar("--c-terminal") || "#ff5a52") : (cssVar("--instr-ink") || "#cdd7ea");

    // missile body
    this._drawModel(ctx, TOMAHAWK, pos, basis, s, color, P);

    // booster: attached during boost, separating for a few seconds after
    if (stage === "BOOST") {
      this._drawModel(ctx, BOOSTER, pos, basis, s, cssVar("--c-boost") || "#b48cff", P);
    } else if (this.detach) {
      const dt = this.frame.t - this.detach.t;
      if (dt >= 0 && dt < 4) {
        // fall behind + down from the separation point, tumbling
        const drop = 0.5 * G * dt * dt * this.vExag;
        const bpos = add(this.detach.pos, add(scale(this.detach.f, -18 * dt), [0, 0, -drop]));
        const bColor = cssVar("--c-boost") || "#b48cff";
        ctx.globalAlpha = clamp(1 - dt / 4, 0.15, 1);
        this._drawModel(ctx, BOOSTER, bpos, this.detach, s, bColor, P, dt * 2.4);
        ctx.globalAlpha = 1;
      }
    }

    // id label
    const p = P(pos);
    if (p && this.label) { ctx.fillStyle = color; ctx.font = "11px " + (cssVar("--font-mono") || "monospace"); ctx.fillText(this.label, p[0] + s * 0.5 + 6, p[1] - s * 0.5); }
  }
}
