// Third-person 3D viewer — self-contained WebGL2 renderer (no libraries).
//
// Orbit camera around the missile. Renders the whole DEM tile as a lit,
// hypsometrically-tinted elevation mesh under an atmospheric sky with sun and
// distance fog; a solid-shaded Tomahawk Block V with real proportions (ogive
// nose, ventral inlet, x-cruciform tail fins, wings that swing out after boost);
// the Mk-111 booster with a flickering rocket plume and smoke trail during
// BOOST, a tumbling ballistic separation at the boost→cruise handoff, and a
// fireball on IMPACT. Planned/flown trajectories, marker beacons, labels and
// the AGL drop-line draw on a 2D overlay. Renders on demand; animates only
// while flame/smoke/explosion effects are live.
//
// Public API is unchanged from the old canvas viewer: setDEM/setStart/setTarget/
// setPlan/pushFrame/setPathFrames/resetPath/resetView/frameCamera/invalidate.

import { cssVar, clamp } from "./util.js";

const DEG = Math.PI / 180;
const G = 9.80665;

// --- vec3 helpers -------------------------------------------------------------
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

// --- mat4 (column-major) helpers -----------------------------------------------
function m4mul(a, b) {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
  }
  return o;
}
// Model matrix from position + body basis (f=x fwd, r=y right, u=z up) + scales.
function m4basis(pos, f, r, u, sx, sy = sx, sz = sx) {
  return new Float32Array([
    f[0] * sx, f[1] * sx, f[2] * sx, 0,
    r[0] * sy, r[1] * sy, r[2] * sy, 0,
    u[0] * sz, u[1] * sz, u[2] * sz, 0,
    pos[0], pos[1], pos[2], 1,
  ]);
}
function m4persp(fovY, aspect, near, far) {
  const t = 1 / Math.tan(fovY / 2);
  return new Float32Array([
    t / aspect, 0, 0, 0,
    0, t, 0, 0,
    0, 0, (far + near) / (near - far), -1,
    0, 0, (2 * far * near) / (near - far), 0,
  ]);
}
function m4view(eye, f, r, u) {
  return new Float32Array([
    r[0], u[0], -f[0], 0,
    r[1], u[1], -f[1], 0,
    r[2], u[2], -f[2], 0,
    -dot(r, eye), -dot(u, eye), dot(f, eye), 1,
  ]);
}
// Rotation about local z through a pivot (used for the wing swing-out).
function m4pivotRotZ(pivot, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  const [px, py, pz] = pivot;
  return new Float32Array([
    c, s, 0, 0,
    -s, c, 0, 0,
    0, 0, 1, 0,
    px - c * px + s * py, py - s * px - c * py, pz - pz, 1,
  ]);
}

// --- deterministic rng (explosion / flicker seeds) ------------------------------
function mulberry(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// =================================================================================
// Shaders
// =================================================================================
const MESH_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec3 aCol;
uniform mat4 uVP, uModel;
out vec3 vNrm; out vec3 vCol; out vec3 vWorld;
void main() {
  vec4 w = uModel * vec4(aPos, 1.0);
  vWorld = w.xyz;
  vNrm = mat3(uModel) * aNrm;
  vCol = aCol;
  gl_Position = uVP * w;
}`;
const MESH_FS = `#version 300 es
precision highp float;
uniform vec3 uSun, uCamPos, uFogColor;
uniform float uFogDensity, uEmissive, uAlpha, uAmbient, uSpec;
in vec3 vNrm; in vec3 vCol; in vec3 vWorld;
out vec4 frag;
void main() {
  vec3 n = normalize(vNrm);
  if (!gl_FrontFacing) n = -n;
  float diff = max(dot(n, uSun), 0.0);
  vec3 v = normalize(uCamPos - vWorld);
  float spec = pow(max(dot(n, normalize(v + uSun)), 0.0), 42.0) * uSpec;
  vec3 lit = vCol * (uAmbient + diff * (1.15 - uAmbient)) + vec3(spec);
  vec3 col = mix(lit, vCol, uEmissive);
  float d = length(uCamPos - vWorld);
  float fog = 1.0 - exp(-pow(d * uFogDensity, 1.5));
  col = mix(col, uFogColor, clamp(fog, 0.0, 1.0) * (1.0 - uEmissive));
  frag = vec4(col, uAlpha);
}`;

const SKY_VS = `#version 300 es
layout(location=0) in vec2 aXY;
uniform vec3 uCamF, uCamR, uCamU;
uniform float uTanF, uAspect;
out vec3 vDir;
void main() {
  vDir = uCamF + uCamR * (aXY.x * uTanF * uAspect) + uCamU * (aXY.y * uTanF);
  gl_Position = vec4(aXY, 0.99999, 1.0);
}`;
const SKY_FS = `#version 300 es
precision highp float;
uniform vec3 uSun;
in vec3 vDir;
out vec4 frag;
void main() {
  vec3 d = normalize(vDir);
  vec3 zen = vec3(0.030, 0.062, 0.150);
  vec3 hor = vec3(0.245, 0.335, 0.475);
  vec3 below = vec3(0.016, 0.024, 0.042);
  vec3 col = d.z >= 0.0
    ? mix(hor, zen, pow(d.z, 0.42))
    : mix(hor, below, clamp(-d.z * 7.0, 0.0, 1.0));
  float s = max(dot(d, uSun), 0.0);
  col += vec3(1.0, 0.72, 0.45) * pow(s, 6.0) * 0.16;   // warm haze around the sun
  col += vec3(1.0, 0.92, 0.78) * pow(s, 900.0) * 2.2;  // sun disc
  frag = vec4(col, 1.0);
}`;

const SPRITE_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aUV;
layout(location=2) in vec4 aCol;
uniform mat4 uVP;
out vec2 vUV; out vec4 vCol;
void main() { vUV = aUV; vCol = aCol; gl_Position = uVP * vec4(aPos, 1.0); }`;
const SPRITE_FS = `#version 300 es
precision highp float;
uniform int uMode;   // 0 = soft radial puff/glow, 1 = vertical beam
in vec2 vUV; in vec4 vCol;
out vec4 frag;
void main() {
  float a;
  if (uMode == 1) {
    float dx = abs(vUV.x * 2.0 - 1.0);
    a = pow(1.0 - dx, 2.2) * (1.0 - vUV.y);
  } else {
    float d = length(vUV * 2.0 - 1.0);
    if (d > 1.0) discard;
    a = exp(-d * d * 3.4) * (1.0 - d * d * 0.55);
  }
  frag = vec4(vCol.rgb, vCol.a * a);
}`;

function compile(gl, vsSrc, fsSrc) {
  const mk = (type, src) => {
    const sh = gl.createShader(type);
    gl.shaderSource(sh, src); gl.compileShader(sh);
    if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(sh) || "shader compile failed");
    return sh;
  };
  const p = gl.createProgram();
  gl.attachShader(p, mk(gl.VERTEX_SHADER, vsSrc));
  gl.attachShader(p, mk(gl.FRAGMENT_SHADER, fsSrc));
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p) || "program link failed");
  return p;
}

// =================================================================================
// Mesh building (interleaved pos3 / nrm3 / col3)
// =================================================================================
class MeshBuilder {
  constructor() { this.v = []; }
  vert(p, n, c) { this.v.push(p[0], p[1], p[2], n[0], n[1], n[2], c[0], c[1], c[2]); }
  tri(a, b, c, col, n = null) {
    const nn = n || norm(cross(sub(b, a), sub(c, a)));
    this.vert(a, nn, col); this.vert(b, nn, col); this.vert(c, nn, col);
  }
  quad(a, b, c, d, col, n = null) { this.tri(a, b, c, col, n); this.tri(a, c, d, col, n); }
  // Surface of revolution around the x-axis. profile = [[x, r, col?], ...].
  // Smooth per-vertex normals from the profile slope.
  revolve(profile, seg, defCol) {
    const P = profile;
    const slope = (i) => {
      const i0 = Math.max(0, i - 1), i1 = Math.min(P.length - 1, i + 1);
      const dx = P[i1][0] - P[i0][0] || 1e-6;
      return (P[i1][1] - P[i0][1]) / dx;
    };
    for (let i = 0; i < P.length - 1; i++) {
      const [x0, r0] = P[i], [x1, r1] = P[i + 1];
      const c0 = P[i][2] || defCol, c1 = P[i + 1][2] || defCol;
      const m0 = slope(i), m1 = slope(i + 1);
      for (let j = 0; j < seg; j++) {
        const a0 = (j / seg) * 2 * Math.PI, a1 = ((j + 1) / seg) * 2 * Math.PI;
        const ca0 = Math.cos(a0), sa0 = Math.sin(a0), ca1 = Math.cos(a1), sa1 = Math.sin(a1);
        const p00 = [x0, r0 * ca0, r0 * sa0], p01 = [x0, r0 * ca1, r0 * sa1];
        const p10 = [x1, r1 * ca0, r1 * sa0], p11 = [x1, r1 * ca1, r1 * sa1];
        const n00 = norm([-m0, ca0, sa0]), n01 = norm([-m0, ca1, sa1]);
        const n10 = norm([-m1, ca0, sa0]), n11 = norm([-m1, ca1, sa1]);
        this.vert(p00, n00, c0); this.vert(p10, n10, c1); this.vert(p11, n11, c1);
        this.vert(p00, n00, c0); this.vert(p11, n11, c1); this.vert(p01, n01, c0);
      }
    }
  }
  // Thin plate from a planar polygon (fan-triangulated) extruded along `dir`.
  plate(poly, dir, th, col) {
    const off = scale(norm(dir), th / 2);
    const a = poly.map((p) => add(p, off)), b = poly.map((p) => sub(p, off));
    for (let i = 1; i < poly.length - 1; i++) {
      this.tri(a[0], a[i], a[i + 1], col);
      this.tri(b[0], b[i + 1], b[i], col);
    }
    for (let i = 0; i < poly.length; i++) {
      const j = (i + 1) % poly.length;
      this.quad(a[i], a[j], b[j], b[i], col);
    }
  }
  upload(gl) {
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const vbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.v), gl.STATIC_DRAW);
    for (const [loc, size, off] of [[0, 3, 0], [1, 3, 12], [2, 3, 24]]) {
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 36, off);
    }
    gl.bindVertexArray(null);
    return { vao, count: this.v.length / 9 };
  }
}

// --- Tomahawk Block V (body frame, metres: x=nose fwd, y=right, z=up) -----------
const R = 0.26;                              // fuselage radius (0.52 m diameter)
const NOSE = 2.90, TAIL = -2.66;             // 5.56 m airframe
const BOOST_TAIL = -3.74, NOZZLE_X = -3.96;  // Mk-111 booster aft of the airframe
const BOOST_CENTER = (TAIL + NOZZLE_X) / 2;  // booster mid-point in body coords
const DRAG_TAU = 2.2;                        // spent-casing horizontal drag time constant (s)
const BODY_COL = [0.78, 0.80, 0.84];
const DARK_COL = [0.30, 0.31, 0.34];

function buildAirframe() {
  const mb = new MeshBuilder();
  const radome = [0.60, 0.62, 0.66];
  // ogive nose → constant fuselage → boattail; nozzle base ring is dark.
  mb.revolve([
    [NOSE, 0.004, radome], [NOSE - 0.05, 0.085, radome], [NOSE - 0.16, 0.155, radome],
    [NOSE - 0.38, 0.212, radome], [NOSE - 0.75, 0.246, BODY_COL], [NOSE - 1.35, R, BODY_COL],
    [TAIL + 0.32, R, BODY_COL], [TAIL, 0.225, BODY_COL], [TAIL, 0.13, DARK_COL],
  ], 26);
  // x-cruciform tail fins (clipped delta) at 45/135/225/315.
  for (const ang of [45, 135, 225, 315]) {
    const c = Math.cos(ang * DEG), s = Math.sin(ang * DEG);
    const at = (x, sp) => [x, c * (R * 0.55 + sp), s * (R * 0.55 + sp)];
    mb.plate([at(-2.02, 0), at(-2.44, 0.56), at(-2.62, 0.56), at(-2.62, 0)], [0, -s, c], 0.035, BODY_COL);
  }
  // ventral turbofan inlet scoop (deployed) under the rear belly.
  mb.plate([[-1.30, 0, -R + 0.02], [-1.62, 0, -0.46], [-2.14, 0, -0.46], [-2.24, 0, -R + 0.02]], [0, 1, 0], 0.24, DARK_COL);
  return mb;
}
function buildWing(side) { // side = +1 right, -1 left; hinge at the root
  const mb = new MeshBuilder();
  const wz = -0.03, half = 1.335; // 2.67 m span
  mb.plate([
    [0.58, side * 0.10, wz], [0.30, side * half, wz],
    [-0.14, side * half, wz], [-0.10, side * 0.10, wz],
  ], [0, 0, 1], 0.035, BODY_COL);
  return mb;
}
function buildBooster() {
  const mb = new MeshBuilder();
  const col = [0.40, 0.41, 0.44];
  mb.revolve([
    [TAIL, 0.23, col], [TAIL - 0.10, R, col], [BOOST_TAIL + 0.18, R, col],
    [BOOST_TAIL, 0.20, DARK_COL], [NOZZLE_X, 0.155, DARK_COL],
  ], 22);
  return mb;
}
// Unit flame cone: apex at x=0, extends to x=-1, max radius 1 (scaled per draw).
function buildFlame(col) {
  const mb = new MeshBuilder();
  const tip = [col[0], col[1] * 0.7, col[2] * 0.5];
  mb.revolve([[0, 0.42, col], [-0.16, 1.0, col], [-0.55, 0.72, col], [-1.0, 0.03, tip]], 18);
  return mb;
}

// Wing pivot in body coords (root leading edge) and stowed swing angle.
const WING_PIVOT = [0.58, 0.10, -0.03];
const WING_STOWED = 82 * DEG;

// =================================================================================
// Viewer
// =================================================================================
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
    // Fixed world-space model exaggeration (drawn length = 5.56 m × this). Static,
    // so the model behaves like a real object: bigger when you zoom in, smaller
    // when you zoom out — it no longer rescales itself with camera distance.
    this.modelScale = 45;
    this.follow = true;
    this.showTerrain = true;
    this.detach = null;    // booster separation: { t, pos, f, r, u }
    this.boom = null;      // impact: { t, pos, parts }
    this.smoke = [];       // booster trail particles: { p, t0, size0, seed }
    this._dirty = true;
    this._raf = null;
    this._terr = null;     // uploaded terrain mesh { vao, count, origin }

    this._initGL();
    this._initOverlay();
    this._wire();
    this._loop();
    this._ro = new ResizeObserver(() => { this._dirty = true; });
    this._ro.observe(this.canvas);
  }

  _initGL() {
    const gl = this.canvas.getContext("webgl2", { antialias: true, alpha: false, powerPreference: "high-performance" });
    this.gl = gl;
    if (!gl) return; // overlay will show a notice
    this.progMesh = compile(gl, MESH_VS, MESH_FS);
    this.progSky = compile(gl, SKY_VS, SKY_FS);
    this.progSprite = compile(gl, SPRITE_VS, SPRITE_FS);
    this.uni = {
      mesh: Object.fromEntries(["uVP", "uModel", "uSun", "uCamPos", "uFogColor", "uFogDensity", "uEmissive", "uAlpha", "uAmbient", "uSpec"].map((n) => [n, gl.getUniformLocation(this.progMesh, n)])),
      sky: Object.fromEntries(["uCamF", "uCamR", "uCamU", "uTanF", "uAspect", "uSun"].map((n) => [n, gl.getUniformLocation(this.progSky, n)])),
      sprite: Object.fromEntries(["uVP", "uMode"].map((n) => [n, gl.getUniformLocation(this.progSprite, n)])),
    };
    // fullscreen triangle for the sky
    this.skyVAO = gl.createVertexArray();
    gl.bindVertexArray(this.skyVAO);
    const b = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, b);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 3, -1, -1, 3]), gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    // dynamic sprite buffer (flame glow / smoke / beams / shadows / explosion)
    this.spriteVAO = gl.createVertexArray();
    gl.bindVertexArray(this.spriteVAO);
    this.spriteVBO = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.spriteVBO);
    for (const [loc, size, off] of [[0, 3, 0], [1, 2, 12], [2, 4, 20]]) {
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 36, off);
    }
    gl.bindVertexArray(null);

    this.meshAirframe = buildAirframe().upload(gl);
    this.meshWingR = buildWing(1).upload(gl);
    this.meshWingL = buildWing(-1).upload(gl);
    this.meshBooster = buildBooster().upload(gl);
    this.meshFlameOuter = buildFlame([1.0, 0.52, 0.16]).upload(gl);
    this.meshFlameInner = buildFlame([1.0, 0.95, 0.72]).upload(gl);

    this.sun = norm([-0.45, -0.28, 0.85]);
    this.fogColor = [0.225, 0.31, 0.44];

    this.canvas.addEventListener("webglcontextlost", (e) => e.preventDefault());
  }

  // Text/lines overlay stacked on the GL canvas (parent .instr is relative).
  _initOverlay() {
    this.overlay = document.createElement("canvas");
    Object.assign(this.overlay.style, { position: "absolute", inset: "0", width: "100%", height: "100%", pointerEvents: "none", zIndex: "300" });
    (this.canvas.parentElement || this.canvas.closest(".instr"))?.appendChild(this.overlay);
  }

  // --- data (same contract as the old viewer) --------------------------------
  setDEM(grid) {
    this.grid = grid;
    if (!this.origin && grid) { const b = grid.bounds; this._setOrigin((b.north + b.south) / 2, (b.west + b.east) / 2); }
    this._terr = null;
    this._dirty = true;
    this._fetchFineGrid(grid);
  }
  // The shared map grid is capped at 180px; quietly swap in a finer mesh for 3D.
  async _fetchFineGrid(grid) {
    if (!grid?.name || Math.max(grid.rows || 0, grid.cols || 0) >= 320) return;
    const want = (this._finePending = grid.name);
    try {
      const res = await fetch(`/api/dems/${encodeURIComponent(grid.name)}/grid?max_size=512`);
      if (!res.ok) return;
      const fine = await res.json();
      if (this._finePending === want && this.grid?.name === grid.name) { this.grid = fine; this._terr = null; this._dirty = true; }
    } catch { /* keep the coarse grid */ }
  }
  _setOrigin(lat0, lon0) { this.origin = [lat0, lon0]; this.mPerLon = 111320 * Math.cos(lat0 * DEG); }
  _world(lat, lon, z) {
    if (!this.origin) this._setOrigin(lat, lon);
    return [(lon - this.origin[1]) * this.mPerLon, (lat - this.origin[0]) * this.mPerLat, z * this.vExag];
  }
  setStart(gps) { if (gps) { this._setOrigin(gps[0], gps[1]); this.start = this._world(gps[0], gps[1], gps[2]); this._dirty = true; } }
  setTarget(gps) { if (gps) { this.target = this._world(gps[0], gps[1], gps[2]); this._dirty = true; } }
  setPlan(traj) { this.plan = traj && traj.length ? traj.map((p) => this._world(p[0], p[1], p[2])) : null; this._dirty = true; }
  resetPath() { this.path = []; this.detach = null; this.boom = null; this.smoke = []; this._dirty = true; }

  // Keep the flown trail on the missile's true trajectory. The oversized model
  // is lifted separately to avoid terrain clipping; baking that visual offset
  // into the trail makes low-altitude attitude changes look like real motion.
  _pathEntry(frame) {
    const t = frame.true;
    const p = this._world(t.lat, t.lon, t.alt);
    const ground = t.ground_alt != null ? this._world(t.lat, t.lon, t.ground_alt) : [p[0], p[1], 0];
    return { p, ground, stage: frame.stage };
  }
  pushFrame(frame) {
    const prev = this.frame;
    this.frame = frame;
    const entry = this._pathEntry(frame);
    this.path.push(entry);
    if (frame.stage === "BOOST") this._emitSmoke(prev, frame);
    if (prev && prev.stage === "BOOST" && frame.stage !== "BOOST") this._markDetach(frame);
    if (prev && prev.stage !== "IMPACT" && frame.stage === "IMPACT") this._markBoom(frame);
    if (this.follow) this.cam.target = entry.p;
    this._dirty = true;
  }
  setPathFrames(frames) {
    this.path = [];
    this.detach = null; this.boom = null; this.smoke = [];
    let prev = null;
    for (const f of frames) {
      this.frame = f;
      this.path.push(this._pathEntry(f));
      if (f.stage === "BOOST") this._emitSmoke(prev, f);
      if (prev && prev.stage === "BOOST" && f.stage !== "BOOST") this._markDetach(f);
      if (prev && prev.stage !== "IMPACT" && f.stage === "IMPACT") this._markBoom(f);
      prev = f;
    }
    if (frames.length && this.follow) this.cam.target = this.path[this.path.length - 1].p;
    this._dirty = true;
  }
  _markDetach(frame) {
    const t = frame.true;
    const { f, r, u } = this._attitudeBasis(frame);
    const k = this._modelScale();
    // The casing separates from the booster's own centre (aft of the airframe),
    // carrying the missile's velocity at that instant.
    const cg = this._drawnPos(this._world(t.lat, t.lon, t.alt), f, k, -NOZZLE_X);
    const pos = add(cg, scale(f, BOOST_CENTER * k));
    const vel = [frame.vel.east || 0, frame.vel.north || 0, frame.vel.up || 0];
    this.detach = { t: frame.t, pos, f, r, u, vel };
  }
  _markBoom(frame) {
    const t = frame.true;
    const pos = this._world(t.lat, t.lon, t.ground_alt != null ? Math.min(t.alt, t.ground_alt) : t.alt);
    const rng = mulberry(Math.round(frame.t * 1000) + 7);
    const parts = [];
    for (let i = 0; i < 46; i++) {
      const az = rng() * 2 * Math.PI, el = rng() * 0.9 + 0.12; // biased upward
      parts.push({
        dir: [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)],
        spd: 0.5 + rng() * 1.3, size: 0.5 + rng() * 0.8, hot: i < 22, seed: rng(),
      });
    }
    this.boom = { t: frame.t, pos, parts };
  }
  _emitSmoke(prev, frame) {
    const k = this._modelScale();
    const t = frame.true;
    const { f } = this._attitudeBasis(frame);
    const w = this._drawnPos(this._world(t.lat, t.lon, t.alt), f, k, -NOZZLE_X);
    const nozzle = add(w, scale(f, NOZZLE_X * k));
    const pw = prev ? this._drawnPos(this._world(prev.true.lat, prev.true.lon, prev.true.alt), f, k, -NOZZLE_X) : nozzle;
    const pNozzle = prev ? add(pw, scale(f, NOZZLE_X * k)) : nozzle;
    const dt = prev ? Math.max(0.001, frame.t - prev.t) : 0.05;
    const n = clamp(Math.round(dt * 26), 1, 24);
    for (let i = 0; i < n; i++) {
      const a = (i + 1) / n;
      const p = add(scale(pNozzle, 1 - a), scale(nozzle, a));
      const j = () => (Math.random() - 0.5) * 0.24 * k;
      this.smoke.push({ p: [p[0] + j(), p[1] + j(), p[2] + j()], t0: frame.t - dt * (1 - a), size0: (0.30 + Math.random() * 0.22) * k, seed: Math.random() });
    }
    if (this.smoke.length > 2200) this.smoke.splice(0, this.smoke.length - 2200);
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

  // --- interaction ------------------------------------------------------------
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
    // Never let the eye sink under the terrain mesh (no void-side views).
    const gz = this._groundZAt(eye[0], eye[1]);
    if (gz != null) eye[2] = Math.max(eye[2], gz + Math.max(8, this.cam.dist * 0.03));
    const forward = norm(sub(this.cam.target, eye));
    const right = norm(cross(forward, [0, 0, 1]));
    const up = cross(right, forward);
    return { eye, forward, right, up };
  }

  // Bilinear drawn-terrain height (already vExag-scaled) at world x/y, or null.
  _groundZAt(x, y) {
    const g = this.grid;
    if (!g || !this.origin) return null;
    const b = g.bounds;
    const lat = this.origin[0] + y / this.mPerLat;
    const lon = this.origin[1] + x / this.mPerLon;
    const rf = ((b.north - lat) / (b.north - b.south)) * (g.rows - 1);
    const cf = ((lon - b.west) / (b.east - b.west)) * (g.cols - 1);
    if (!(rf >= 0 && cf >= 0 && rf <= g.rows - 1 && cf <= g.cols - 1)) return null;
    const r0 = Math.floor(rf), c0 = Math.floor(cf);
    const r1 = Math.min(r0 + 1, g.rows - 1), c1 = Math.min(c0 + 1, g.cols - 1);
    const fr = rf - r0, fc = cf - c0;
    const e = g.elev, C = g.cols;
    const z = e[r0 * C + c0] * (1 - fr) * (1 - fc) + e[r0 * C + c1] * (1 - fr) * fc
            + e[r1 * C + c0] * fr * (1 - fc) + e[r1 * C + c1] * fr * fc;
    return z * this.vExag;
  }

  // How far (in model units) the hull can extend below the CG for an attitude:
  // tail-down when climbing, nose-down when diving, wings/fins when level.
  _reach(f, aft) {
    return Math.max(f[2], 0) * aft + Math.max(-f[2], 0) * NOSE + (1 - Math.abs(f[2])) * 0.95 + 0.05;
  }
  // Where the (deliberately oversized) model is drawn: the true position, lifted
  // just enough that it never pokes through the terrain mesh. On the pad this
  // parks the booster nozzle exactly on the ground.
  _drawnPos(pos, f, k, aft) {
    const gz = this._groundZAt(pos[0], pos[1]);
    if (gz == null) return pos;
    const minZ = gz + k * this._reach(f, aft);
    return pos[2] >= minZ ? pos : [pos[0], pos[1], minZ];
  }

  _attitudeBasis(frame) {
    // Body attitude controls model orientation. Flight-path angle describes the
    // velocity vector and can differ sharply from pitch during vertical launch.
    const yaw = (frame.att.yaw || 0) * DEG, pitch = (frame.att.pitch || 0) * DEG, roll = (frame.att.roll || 0) * DEG;
    const f = norm([Math.cos(pitch) * Math.sin(yaw), Math.cos(pitch) * Math.cos(yaw), Math.sin(pitch)]);
    let r = norm(cross(f, [0, 0, 1]));
    let u = cross(r, f);
    if (roll) { // bank about the body axis
      const c = Math.cos(roll), s = Math.sin(roll);
      const r2 = add(scale(r, c), scale(u, s)), u2 = sub(scale(u, c), scale(r, s));
      r = r2; u = u2;
    }
    return { f, r, u };
  }

  _modelScale() { return this.modelScale; }

  // Continuous redraws only while something is visibly animating.
  _animating() {
    const f = this.frame;
    if (!f) return false;
    if (f.stage === "BOOST") return true;
    if (this.detach && f.t - this.detach.t < 10.5) return true;
    if (this.boom && f.t - this.boom.t < 4) return true;
    if (this.smoke.length && f.t - this.smoke[this.smoke.length - 1].t0 < SMOKE_LIFE + 1) return true;
    return false;
  }
  _loop() {
    const tick = () => {
      if (this._dirty || this._animating()) { this._dirty = false; this._render(); }
      this._raf = requestAnimationFrame(tick);
    };
    this._raf = requestAnimationFrame(tick);
  }
  invalidate() { this._dirty = true; }
  destroy() { cancelAnimationFrame(this._raf); this._ro?.disconnect(); this.overlay?.remove(); }

  // --- render ------------------------------------------------------------------
  _fit() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(rect.width * dpr)), h = Math.max(1, Math.round(rect.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    if (this.overlay.width !== w || this.overlay.height !== h) { this.overlay.width = w; this.overlay.height = h; }
    return [rect.width, rect.height, dpr];
  }

  _render() {
    const [W, H, dpr] = this._fit();
    if (W < 4 || H < 4) return;
    const octx = this.overlay.getContext("2d");
    octx.setTransform(dpr, 0, 0, dpr, 0, 0);
    octx.clearRect(0, 0, W, H);

    const gl = this.gl;
    if (!gl) { this._overlayNotice(octx, W, H, "WebGL2 unavailable"); return; }

    const view = this._basis();
    const fovY = 50 * DEG;
    const near = clamp(this.cam.dist * 0.004, 0.5, 400);
    const far = 2.6e6;
    const proj = m4persp(fovY, W / H, near, far);
    const vp = m4mul(proj, m4view(view.eye, view.forward, view.right, view.up));
    this._vp = vp; this._view = view; this._W = W; this._H = H;
    const fogDensity = 1 / clamp(this.cam.dist * 22, 26000, 900000);

    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.disable(gl.CULL_FACE);
    gl.clearColor(0.02, 0.03, 0.05, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // sky
    gl.disable(gl.DEPTH_TEST);
    gl.useProgram(this.progSky);
    gl.uniform3fv(this.uni.sky.uCamF, view.forward);
    gl.uniform3fv(this.uni.sky.uCamR, view.right);
    gl.uniform3fv(this.uni.sky.uCamU, view.up);
    gl.uniform1f(this.uni.sky.uTanF, Math.tan(fovY / 2));
    gl.uniform1f(this.uni.sky.uAspect, W / H);
    gl.uniform3fv(this.uni.sky.uSun, this.sun);
    gl.bindVertexArray(this.skyVAO);
    gl.drawArrays(gl.TRIANGLES, 0, 3);

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);

    // mesh program shared uniforms
    gl.useProgram(this.progMesh);
    const mu = this.uni.mesh;
    gl.uniformMatrix4fv(mu.uVP, false, vp);
    gl.uniform3fv(mu.uSun, this.sun);
    gl.uniform3fv(mu.uCamPos, view.eye);
    gl.uniform3fv(mu.uFogColor, this.fogColor);
    gl.uniform1f(mu.uFogDensity, fogDensity);

    if (this.showTerrain) this._drawTerrain(gl, mu);
    this._drawVehicle(gl, mu);

    this._drawSprites(gl);
    this._drawOverlay(octx, W, H);
  }

  _setMesh(gl, mu, model, { emissive = 0, alpha = 1, ambient = 0.45, spec = 0.35 } = {}) {
    gl.uniformMatrix4fv(mu.uModel, false, model);
    gl.uniform1f(mu.uEmissive, emissive);
    gl.uniform1f(mu.uAlpha, alpha);
    gl.uniform1f(mu.uAmbient, ambient);
    gl.uniform1f(mu.uSpec, spec);
  }
  static IDENT = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);

  // --- terrain -------------------------------------------------------------------
  _buildTerrain() {
    const gl = this.gl, grid = this.grid;
    const { rows, cols, bounds: b } = grid;
    const elev = grid.elev;
    const zSpan = Math.max(1, grid.z_max - grid.z_min);
    const latStep = (b.north - b.south) / (rows - 1);
    const lonStep = (b.east - b.west) / (cols - 1);
    const dyM = latStep * this.mPerLat, dxM = lonStep * this.mPerLon;

    const verts = new Float32Array(rows * cols * 9);
    let vi = 0;
    const zAt = (r, c) => elev[clamp(r, 0, rows - 1) * cols + clamp(c, 0, cols - 1)];
    for (let r = 0; r < rows; r++) {
      const lat = b.north - r * latStep;
      const y = (lat - this.origin[0]) * this.mPerLat;
      for (let c = 0; c < cols; c++) {
        const lon = b.west + c * lonStep;
        const x = (lon - this.origin[1]) * this.mPerLon;
        const z = zAt(r, c);
        const dzdx = ((zAt(r, c + 1) - zAt(r, c - 1)) * this.vExag) / (2 * dxM);
        const dzdy = ((zAt(r - 1, c) - zAt(r + 1, c)) * this.vExag) / (2 * dyM);
        const nl = Math.hypot(dzdx, dzdy, 1);
        const h = (z - grid.z_min) / zSpan;
        const slope = 1 - 1 / nl; // 0 flat … →1 steep
        const [cr, cg, cb] = terrainColor(h, slope, r * cols + c);
        verts[vi++] = x; verts[vi++] = y; verts[vi++] = z * this.vExag;
        verts[vi++] = -dzdx / nl; verts[vi++] = -dzdy / nl; verts[vi++] = 1 / nl;
        verts[vi++] = cr; verts[vi++] = cg; verts[vi++] = cb;
      }
    }
    const idx = new Uint32Array((rows - 1) * (cols - 1) * 6);
    let ii = 0;
    for (let r = 0; r < rows - 1; r++) for (let c = 0; c < cols - 1; c++) {
      const a = r * cols + c;
      idx[ii++] = a; idx[ii++] = a + cols; idx[ii++] = a + 1;
      idx[ii++] = a + 1; idx[ii++] = a + cols; idx[ii++] = a + cols + 1;
    }

    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    const vbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);
    for (const [loc, size, off] of [[0, 3, 0], [1, 3, 12], [2, 3, 24]]) {
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 36, off);
    }
    const ibo = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ibo);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, idx, gl.STATIC_DRAW);
    gl.bindVertexArray(null);
    this._terr = { vao, count: idx.length, origin: [...this.origin] };
  }

  _drawTerrain(gl, mu) {
    if (!this.grid || !this.origin) return;
    if (!this._terr || this._terr.origin[0] !== this.origin[0] || this._terr.origin[1] !== this.origin[1]) this._buildTerrain();
    gl.enable(gl.POLYGON_OFFSET_FILL);
    gl.polygonOffset(1, 1);
    this._setMesh(gl, mu, Viewer3D.IDENT, { ambient: 0.30, spec: 0.04 });
    gl.bindVertexArray(this._terr.vao);
    gl.drawElements(gl.TRIANGLES, this._terr.count, gl.UNSIGNED_INT, 0);
    gl.disable(gl.POLYGON_OFFSET_FILL);
  }

  // --- vehicle ---------------------------------------------------------------------
  _drawVehicle(gl, mu) {
    const frame = this.frame;
    if (!frame) return;
    const t = frame.true;
    const { f, r, u } = this._attitudeBasis(frame);
    const k = this._modelScale();
    const stage = frame.stage;
    const aft = stage === "BOOST" ? -NOZZLE_X : -TAIL;
    const pos = this._drawnPos(this._world(t.lat, t.lon, t.alt), f, k, aft);
    const draw = (mesh) => { gl.bindVertexArray(mesh.vao); gl.drawArrays(gl.TRIANGLES, 0, mesh.count); };

    const destroyed = this.boom && frame.t - this.boom.t > 0.1 && stage === "IMPACT";
    if (!destroyed) {
      const M = m4basis(pos, f, r, u, k);
      this._setMesh(gl, mu, M, { spec: 0.5 });
      draw(this.meshAirframe);

      // wings: stowed through boost, swing out shortly after separation
      let deploy = stage === "BOOST" || stage === "PRE_LAUNCHED" ? 0 : 1;
      if (this.detach) deploy = clamp((frame.t - this.detach.t - 0.3) / 1.6, 0, 1);
      const ease = deploy * deploy * (3 - 2 * deploy);
      const ang = (1 - ease) * WING_STOWED;
      const pivR = WING_PIVOT, pivL = [WING_PIVOT[0], -WING_PIVOT[1], WING_PIVOT[2]];
      this._setMesh(gl, mu, m4mul(M, m4pivotRotZ(pivR, ang)), { spec: 0.5 });
      draw(this.meshWingR);
      this._setMesh(gl, mu, m4mul(M, m4pivotRotZ(pivL, -ang)), { spec: 0.5 });
      draw(this.meshWingL);

      if (stage === "BOOST") {
        this._setMesh(gl, mu, M, { spec: 0.4 });
        draw(this.meshBooster);
      }
    }

    // Separated booster: a spent casing on a ballistic arc. It keeps the
    // missile's velocity at separation, bleeds horizontal speed to drag
    // (v ∝ e^{-t/τ}), falls under gravity, pitches over about its own centre,
    // and stays where it lands.
    if (this.detach && stage !== "BOOST") {
      const dt = frame.t - this.detach.t;
      if (dt >= 0 && dt < 10) {
        const d = this.detach;
        const [ve, vn, vu] = d.vel;
        // time until the casing reaches the ground (real metres; world z is ×vExag)
        const gzB = this._groundZAt(d.pos[0], d.pos[1]);
        let tLand = Infinity;
        if (gzB != null) {
          const h = Math.max(0, d.pos[2] - (gzB + 0.45 * k)) / this.vExag;
          tLand = (vu + Math.sqrt(vu * vu + 2 * G * h)) / G;
        }
        const te = Math.min(dt, tLand);
        const hf = DRAG_TAU * (1 - Math.exp(-te / DRAG_TAU)); // ∫₀ᵗ e^{-s/τ} ds
        const bpos = [
          d.pos[0] + ve * hf,
          d.pos[1] + vn * hf,
          d.pos[2] + (vu * te - 0.5 * G * te * te) * this.vExag,
        ];
        // pitch-over about the casing's own centre (mesh is modelled aft of the
        // body origin, so rotate about BOOST_CENTER, not the distant origin)
        const a = 1.5 * te, ca = Math.cos(a), sa = Math.sin(a);
        const f2 = add(scale(d.f, ca), scale(d.u, sa));
        const u2 = sub(scale(d.u, ca), scale(d.f, sa));
        const recenter = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -BOOST_CENTER, 0, 0, 1]);
        const M = m4mul(m4basis(bpos, f2, d.r, u2, k), recenter);
        const alpha = dt < 6 ? 1 : clamp(1 - (dt - 6) / 4, 0, 1);
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        this._setMesh(gl, mu, M, { alpha, spec: 0.4 });
        draw(this.meshBooster);
        gl.disable(gl.BLEND);
      }
    }

    // booster plume: flickering nested cones (additive, emissive)
    if (stage === "BOOST" && !destroyed) {
      const now = performance.now() / 1000;
      const flick = 0.85 + 0.18 * Math.sin(now * 47) + 0.10 * Math.sin(now * 89 + 1.7) + 0.05 * Math.sin(now * 233);
      const fpos = add(pos, scale(f, NOZZLE_X * k));
      // when climbing off the pad, the plume can only reach down to the ground
      let avail = Infinity;
      const gzN = this._groundZAt(fpos[0], fpos[1]);
      if (gzN != null && f[2] > 0.02) avail = Math.max(0, fpos[2] - gzN) / f[2];
      const outerS = clamp(avail / (2.6 * k), 0.12, 1), innerS = clamp(avail / (1.5 * k), 0.12, 1);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE);
      gl.depthMask(false);
      this._setMesh(gl, mu, m4basis(fpos, scale(f, 2.6 * flick * outerS), scale(r, 0.34), scale(u, 0.34), k), { emissive: 1, alpha: 0.35 });
      draw(this.meshFlameOuter);
      this._setMesh(gl, mu, m4basis(fpos, scale(f, 1.5 * flick * innerS), scale(r, 0.20), scale(u, 0.20), k), { emissive: 1, alpha: 0.8 });
      draw(this.meshFlameInner);
      gl.depthMask(true);
      gl.disable(gl.BLEND);
    }
  }

  // --- sprites: glow / smoke / beams / shadow / explosion ----------------------------
  _drawSprites(gl) {
    const view = this._view, frame = this.frame;
    const addQ = []; // additive
    const nrmQ = []; // normal alpha blend
    const k = this._modelScale();
    const now = performance.now() / 1000;
    const camR = view.right, camU = view.up;
    const quad = (list, c, ex, ey, rgba) => {
      const p00 = add(add(c, scale(ex, -1)), scale(ey, -1)), p10 = add(add(c, ex), scale(ey, -1));
      const p11 = add(add(c, ex), ey), p01 = add(add(c, scale(ex, -1)), ey);
      const push = (p, uu, vv) => list.push(p[0], p[1], p[2], uu, vv, rgba[0], rgba[1], rgba[2], rgba[3]);
      push(p00, 0, 1); push(p10, 1, 1); push(p11, 1, 0);
      push(p00, 0, 1); push(p11, 1, 0); push(p01, 0, 0);
    };
    const puff = (list, c, rad, rgba) => quad(list, c, scale(camR, rad), scale(camU, rad), rgba);

    // marker beacons: two crossed vertical light planes each
    const beam = (base, rgb) => {
      if (!base) return;
      const hgt = clamp(this.cam.dist * 0.45, 600, 16000);
      const wid = clamp(this.cam.dist * 0.012, 14, 320);
      const pulse = 0.30 + 0.10 * Math.sin(now * 2.2);
      for (const ex of [[wid, 0, 0], [0, wid, 0]]) {
        const c = add(base, [0, 0, hgt / 2]);
        quad(addQ, c, ex, [0, 0, hgt / 2], [rgb[0], rgb[1], rgb[2], pulse], true);
      }
    };
    beam(this.start, [0.2, 0.95, 0.4]);
    beam(this.target, [1.0, 0.28, 0.22]);

    if (frame) {
      const t = frame.true;
      const { f } = this._attitudeBasis(frame);
      const aft = frame.stage === "BOOST" ? -NOZZLE_X : -TAIL;
      const pos = this._drawnPos(this._world(t.lat, t.lon, t.alt), f, k, aft);
      const destroyed = this.boom && frame.t - this.boom.t > 0.1 && frame.stage === "IMPACT";

      // soft ground shadow under the missile, glued to the drawn terrain mesh
      const gzMesh = this._groundZAt(pos[0], pos[1]);
      const groundZ = Math.max(gzMesh ?? -Infinity, t.ground_alt != null ? t.ground_alt * this.vExag : -Infinity);
      if (!destroyed && Number.isFinite(groundZ)) {
        const aglW = Math.max(0, (pos[2] - groundZ) / this.vExag);
        const rad = k * 1.5 * (1 + aglW / 4000);
        const a = clamp(0.42 - aglW / 6000, 0.05, 0.42);
        quad(nrmQ, [pos[0], pos[1], groundZ + Math.max(2, k * 0.06)], [rad, 0, 0], [0, rad * 0.55, 0], [0, 0, 0, a]);
      }

      // engine glow at the booster nozzle
      if (frame.stage === "BOOST" && !destroyed) {
        const flick = 0.8 + 0.2 * Math.sin(now * 61 + 2.2);
        const fpos = add(pos, scale(f, NOZZLE_X * k));
        puff(addQ, fpos, k * 1.05 * flick, [1.0, 0.62, 0.25, 0.55]);
        puff(addQ, fpos, k * 0.45, [1.0, 0.92, 0.7, 0.8]);
      }

      // booster smoke trail (sim-time driven so scrubbing replays it)
      const T = frame.t;
      let alive = 0;
      for (const s of this.smoke) {
        const age = T - s.t0;
        if (age < 0 || age > SMOKE_LIFE) continue;
        if (++alive > 1400) break;
        const g = age / SMOKE_LIFE;
        const rad = s.size0 * (1 + g * 5.5);
        const rise = 6 * age * this.vExag * (0.4 + s.seed * 0.5);
        const drift = 3 * age;
        const a = 0.30 * (1 - g) * (1 - g);
        const grey = 0.62 + 0.2 * s.seed;
        puff(nrmQ, [s.p[0] + drift * (s.seed - 0.5), s.p[1] + drift * (0.5 - s.seed), s.p[2] + rise], rad, [grey, grey, grey, a]);
      }
      // hot fresh section of the trail glows faintly
      if (frame.stage === "BOOST") {
        for (const s of this.smoke.slice(-24)) {
          const age = T - s.t0;
          if (age < 0 || age > 0.7) continue;
          puff(addQ, s.p, s.size0 * 1.4, [1.0, 0.55, 0.2, 0.25 * (1 - age / 0.7)]);
        }
      }

      // impact fireball + smoke + flash (clamped up to the drawn terrain mesh)
      if (this.boom) {
        const dt = T - this.boom.t;
        const gzE = this._groundZAt(this.boom.pos[0], this.boom.pos[1]);
        const bpos = [this.boom.pos[0], this.boom.pos[1], Math.max(this.boom.pos[2], gzE ?? -Infinity)];
        if (dt >= 0 && dt < 4) {
          const b = this.boom;
          if (dt < 0.45) puff(addQ, add(bpos, [0, 0, k * 0.6]), k * (2 + 14 * dt), [1.0, 0.9, 0.7, 0.9 * (1 - dt / 0.45)]);
          for (const p of b.parts) {
            const reach = 1 - Math.exp(-dt * 1.9);
            const c = add(bpos, scale(p.dir, p.spd * k * 2.2 * reach));
            if (p.hot && dt < 1.7) {
              const g = dt / 1.7;
              puff(addQ, c, k * p.size * (0.5 + 1.4 * g), [1.0, 0.45 + 0.4 * (1 - g), 0.12, 0.55 * (1 - g)]);
            } else if (!p.hot) {
              const g = clamp((dt - 0.15) / 3.6, 0, 1);
              if (g < 1) {
                const dark = 0.16 + 0.25 * p.seed;
                puff(nrmQ, add(c, [0, 0, k * dt * 0.8]), k * p.size * (0.7 + 2.4 * g), [dark, dark, dark, 0.5 * (1 - g)]);
              }
            }
          }
          // scorch on the ground
          quad(nrmQ, add(bpos, [0, 0, Math.max(2, k * 0.06)]), [k * 2.4, 0, 0], [0, k * 2.4, 0], [0, 0, 0, 0.5 * clamp(dt * 3, 0, 1)]);
        } else if (dt >= 4) {
          quad(nrmQ, add(bpos, [0, 0, Math.max(2, k * 0.06)]), [k * 2.4, 0, 0], [0, k * 2.4, 0], [0.05, 0.04, 0.04, 0.5]);
        }
      }
    }

    const drawList = (list, additive, mode) => {
      if (!list.length) return;
      gl.useProgram(this.progSprite);
      gl.uniformMatrix4fv(this.uni.sprite.uVP, false, this._vp);
      gl.uniform1i(this.uni.sprite.uMode, mode);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, additive ? gl.ONE : gl.ONE_MINUS_SRC_ALPHA);
      gl.depthMask(false);
      gl.bindVertexArray(this.spriteVAO);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.spriteVBO);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(list), gl.DYNAMIC_DRAW);
      gl.drawArrays(gl.TRIANGLES, 0, list.length / 9);
      gl.depthMask(true);
      gl.disable(gl.BLEND);
    };
    // beams need the beam falloff; split them out of the additive list
    const beamsPer = 2 * 6 * 9;
    const nBeams = ((this.start ? 1 : 0) + (this.target ? 1 : 0)) * beamsPer;
    drawList(addQ.slice(0, nBeams), true, 1);
    drawList(addQ.slice(nBeams), true, 0);
    drawList(nrmQ, false, 0);
  }

  // --- 2D overlay: trajectories, markers, labels --------------------------------
  _projectPt(w) {
    const vp = this._vp;
    const x = w[0], y = w[1], z = w[2];
    const cw = vp[3] * x + vp[7] * y + vp[11] * z + vp[15];
    if (cw <= 0.0001) return null;
    const cx = vp[0] * x + vp[4] * y + vp[8] * z + vp[12];
    const cy = vp[1] * x + vp[5] * y + vp[9] * z + vp[13];
    return [(cx / cw * 0.5 + 0.5) * this._W, (0.5 - cy / cw * 0.5) * this._H];
  }
  _poly(ctx, pts, color, width, glow = false, dash = null) {
    if (!pts || pts.length < 2) return;
    ctx.save();
    if (glow) { ctx.shadowColor = color; ctx.shadowBlur = 7; }
    if (dash) ctx.setLineDash(dash);
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.beginPath();
    let started = false;
    for (const w of pts) {
      const p = this._projectPt(w);
      if (!p) { started = false; continue; }
      if (!started) { ctx.moveTo(p[0], p[1]); started = true; } else ctx.lineTo(p[0], p[1]);
    }
    ctx.stroke(); ctx.restore();
  }
  _drawOverlay(ctx, W, H) {
    // Planned route = dashed reference (the A* + spline solution the missile
    // should track). Flown path = solid, segmented by flight stage so boost
    // (purple) and terminal (red) read distinctly against cruise.
    if (this.plan) this._poly(ctx, this.plan, cssVar("--c-planned") || "#4d8dff", 1.4, false, [7, 6]);

    if (this.path.length >= 2) {
      const colFor = (s) => s === "BOOST" ? (cssVar("--c-boost") || "#b48cff") : (s === "TERMINAL" || s === "IMPACT") ? (cssVar("--c-terminal") || "#ff5a52") : (cssVar("--c-actual") || "#39d98a");
      const at = (e) => e.p;
      const last = this.path[this.path.length - 1];
      if (last.ground) this._poly(ctx, [at(last), last.ground], cssVar("--instr-ink-dim") || "#7f8ba6", 0.8);
      // Draw segment-by-segment, carrying the boundary point into the next stage's
      // segment so the coloured path has no gaps at stage transitions.
      let seg = [at(this.path[0])], cur = this.path[0].stage;
      for (let i = 1; i < this.path.length; i++) {
        seg.push(at(this.path[i]));
        if (this.path[i].stage !== cur || i === this.path.length - 1) {
          this._poly(ctx, seg, colFor(cur), 2.4, true);
          seg = [at(this.path[i])]; cur = this.path[i].stage;
        }
      }
    }

    const mark = (w, color, label) => {
      if (!w) return;
      const p = this._projectPt(w);
      if (!p || p[0] < -40 || p[0] > W + 40 || p[1] < -40 || p[1] > H + 40) return;
      ctx.strokeStyle = color; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.arc(p[0], p[1], 6, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(p[0] - 9, p[1]); ctx.lineTo(p[0] + 9, p[1]); ctx.moveTo(p[0], p[1] - 9); ctx.lineTo(p[0], p[1] + 9); ctx.stroke();
      if (label) { ctx.fillStyle = color; ctx.font = "10px " + (cssVar("--font-mono") || "monospace"); ctx.fillText(label, p[0] + 10, p[1] - 8); }
    };
    mark(this.start, cssVar("--c-start") || "#32d74b", "LAUNCH");
    mark(this.target, cssVar("--c-target") || "#ff453a", "TARGET");

    if (this.frame && this.label) {
      const t = this.frame.true;
      const { f } = this._attitudeBasis(this.frame);
      const k = this._modelScale();
      const aft = this.frame.stage === "BOOST" ? -NOZZLE_X : -TAIL;
      const p = this._projectPt(this._drawnPos(this._world(t.lat, t.lon, t.alt), f, k, aft));
      if (p) {
        const stage = this.frame.stage;
        const color = stage === "TERMINAL" || stage === "IMPACT" ? (cssVar("--c-terminal") || "#ff5a52") : (cssVar("--instr-ink") || "#cdd7ea");
        ctx.fillStyle = color; ctx.font = "11px " + (cssVar("--font-mono") || "monospace");
        ctx.fillText(this.label, p[0] + this._modelScale() * 0.04 + 16, p[1] - 14);
      }
    }

    if (!this.grid) this._overlayNotice(ctx, W, H, "No DEM loaded");
  }
  _overlayNotice(ctx, W, H, text) {
    ctx.fillStyle = cssVar("--instr-ink-dim") || "#7f8ba6";
    ctx.font = "12px " + (cssVar("--font-mono") || "monospace");
    ctx.textAlign = "center"; ctx.fillText(text, W / 2, H / 2); ctx.textAlign = "left";
  }
}

const SMOKE_LIFE = 11; // seconds of sim time a trail puff lives

// Hypsometric tint: relative elevation ramp, steep slopes pull toward bare rock,
// a per-vertex hash breaks up banding.
const TERRAIN_STOPS = [
  [0.00, 0.135, 0.215, 0.135],
  [0.16, 0.235, 0.305, 0.160],
  [0.36, 0.415, 0.385, 0.225],
  [0.56, 0.465, 0.375, 0.260],
  [0.74, 0.430, 0.415, 0.410],
  [0.88, 0.600, 0.615, 0.645],
  [1.00, 0.920, 0.940, 0.970],
];
function terrainColor(h, slope, seed) {
  let lo = TERRAIN_STOPS[0], hi = TERRAIN_STOPS[TERRAIN_STOPS.length - 1];
  for (let i = 0; i < TERRAIN_STOPS.length - 1; i++) {
    if (h >= TERRAIN_STOPS[i][0] && h <= TERRAIN_STOPS[i + 1][0]) { lo = TERRAIN_STOPS[i]; hi = TERRAIN_STOPS[i + 1]; break; }
  }
  const f = clamp((h - lo[0]) / Math.max(1e-6, hi[0] - lo[0]), 0, 1);
  let r = lo[1] + (hi[1] - lo[1]) * f;
  let g = lo[2] + (hi[2] - lo[2]) * f;
  let b = lo[3] + (hi[3] - lo[3]) * f;
  const rock = Math.pow(clamp(slope * 2.4, 0, 1), 1.5) * 0.7;
  r = r + (0.40 - r) * rock; g = g + (0.38 - g) * rock; b = b + (0.36 - b) * rock;
  const n = (((seed * 2654435761) >>> 0) % 1000) / 1000 * 0.08 - 0.04;
  return [clamp(r + n, 0, 1), clamp(g + n, 0, 1), clamp(b + n, 0, 1)];
}
