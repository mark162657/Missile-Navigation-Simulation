// Self-contained tactical map — canvas hillshade of the DEM plus vector overlays
// (DEM footprint, launch/target pins, planned + flown trajectories, live missile
// marker with id label). No external tile provider, so it works offline and is
// unaffected by the light/dark theme, exactly as the brief asks. Pan by drag,
// zoom by wheel; click reports a lat/lon back to the host screen.

import { fitCanvas, cssVar, clamp } from "./util.js";

const DEG = Math.PI / 180;

export class Map2D {
  constructor(canvas) {
    this.canvas = canvas;
    this.grid = null;
    this.shade = null;         // offscreen hillshade canvas
    this.view = null;          // { cLat, cLon, spanLat }
    this.plan = null;          // [[lat,lon,elev]]
    this.path = [];            // [{lat,lon,stage}]
    this.start = null; this.target = null;
    this.missile = null;       // {lat,lon,yaw,label,stage}
    this.onClick = null;
    this.layers = { planned: true, flown: true, markers: true };
    this.elevColor = false;
    this._dirty = true;
    this._autoFit = true;      // re-fit on resize until the user pans/zooms
    this._fitBounds = null;    // last geographic bounds passed to fit()
    this._wire();
    this._loop();
    // Re-render at the new resolution whenever the canvas box changes size
    // (widget resize / fullscreen / window resize), else the bitmap is stretched.
    this._ro = new ResizeObserver(() => {
      // Re-frame to the current aspect if the view is still the auto fit; else
      // just repaint the existing view at the new resolution.
      if (this._autoFit && this._fitBounds) this.fit(this._fitBounds);
      this._dirty = true;
    });
    this._ro.observe(this.canvas);
  }

  setDEM(grid) {
    this.grid = grid;
    this.shade = grid ? buildHillshade(grid, this.elevColor) : null;
    // Always recentre on the new DEM so switching tiles jumps to the new area.
    if (grid) this.fit(grid.bounds);
    this._dirty = true;
  }
  setElevColor(on) {
    this.elevColor = on;
    if (this.grid) this.shade = buildHillshade(this.grid, on);
    this._dirty = true;
  }
  setLayer(name, on) { this.layers[name] = on; this._dirty = true; }
  fit(b) {
    const cLat = (b.north + b.south) / 2, cLon = (b.west + b.east) / 2;
    // Aspect-aware "contain": pick the latitude span so the whole box fits the
    // current canvas both ways — otherwise a wide (fullscreen) canvas crops it.
    const rect = this.canvas.getBoundingClientRect();
    const aspect = rect.width / Math.max(1, rect.height);
    const latH = b.north - b.south;
    const lonW = (b.east - b.west) * Math.cos(cLat * DEG);
    const spanLat = Math.max(latH, lonW / Math.max(aspect, 1e-6)) * 1.08;
    this.view = { cLat, cLon, spanLat };
    this._fitBounds = b;
    this._autoFit = true;
    this._dirty = true;
  }
  // Fit the view to a set of [lat, lon] points (route or flown path).
  fitPoints(pts, padFrac = 0.2) {
    if (!pts || !pts.length) return;
    let s = 90, n = -90, w = 180, e = -180;
    for (const p of pts) { s = Math.min(s, p[0]); n = Math.max(n, p[0]); w = Math.min(w, p[1]); e = Math.max(e, p[1]); }
    const dLat = Math.max(n - s, 0.02), dLon = Math.max(e - w, 0.02);
    this.fit({ south: s - dLat * padFrac, north: n + dLat * padFrac, west: w - dLon * padFrac, east: e + dLon * padFrac });
  }
  setPlan(traj) { this.plan = traj; this._dirty = true; }
  resetPath() { this.path = []; this._dirty = true; }
  pushPath(lat, lon, stage) { this.path.push({ lat, lon, stage }); this._dirty = true; }
  setPathPoints(pts) { this.path = pts; this._dirty = true; }
  setStart(gps) { this.start = gps; this._dirty = true; }
  setTarget(gps) { this.target = gps; this._dirty = true; }
  setMissile(m) { this.missile = m; this._dirty = true; }
  invalidate() { this._dirty = true; }

  // --- projection -----------------------------------------------------------
  _dims() {
    const rect = this.canvas.getBoundingClientRect();
    const aspect = rect.width / Math.max(1, rect.height);
    const spanLat = this.view.spanLat;
    const spanLon = (spanLat * aspect) / Math.cos(this.view.cLat * DEG);
    return { W: rect.width, H: rect.height, spanLat, spanLon,
             west: this.view.cLon - spanLon / 2, north: this.view.cLat + spanLat / 2, spanLonDeg: spanLon };
  }
  _project(lat, lon, d, W, H) {
    return [((lon - d.west) / d.spanLonDeg) * W, ((d.north - lat) / d.spanLat) * H];
  }
  _unproject(x, y, d, W, H) {
    return [d.north - (y / H) * d.spanLat, d.west + (x / W) * d.spanLonDeg];
  }

  // --- interaction ----------------------------------------------------------
  _wire() {
    const c = this.canvas;
    let dragging = false, moved = false, lx = 0, ly = 0;
    c.style.touchAction = "none";
    c.addEventListener("pointerdown", (e) => { dragging = true; moved = false; lx = e.clientX; ly = e.clientY; c.setPointerCapture(e.pointerId); });
    c.addEventListener("pointermove", (e) => {
      if (!dragging || !this.view) return;
      if (Math.hypot(e.clientX - lx, e.clientY - ly) > 3) moved = true;
      const d = this._dims();
      this.view.cLon -= ((e.clientX - lx) / d.W) * d.spanLonDeg;
      this.view.cLat += ((e.clientY - ly) / d.H) * d.spanLat;
      lx = e.clientX; ly = e.clientY; this._autoFit = false; this._dirty = true;
    });
    c.addEventListener("pointerup", (e) => {
      dragging = false; c.releasePointerCapture?.(e.pointerId);
      if (!moved && this.onClick && this.view) {
        const rect = c.getBoundingClientRect();
        const d = this._dims();
        const [lat, lon] = this._unproject(e.clientX - rect.left, e.clientY - rect.top, d, d.W, d.H);
        this.onClick(lat, lon);
      }
    });
    c.addEventListener("wheel", (e) => {
      if (!this.view) return;
      e.preventDefault();
      this.view.spanLat = clamp(this.view.spanLat * (1 + Math.sign(e.deltaY) * 0.14), 0.002, 40);
      this._autoFit = false; this._dirty = true;
    }, { passive: false });
    window.addEventListener("themechange", () => (this._dirty = true));
  }

  _loop() { const tick = () => { if (this._dirty) { this._dirty = false; this._render(); } this._raf = requestAnimationFrame(tick); }; this._raf = requestAnimationFrame(tick); }
  destroy() { cancelAnimationFrame(this._raf); this._ro?.disconnect(); }

  _render() {
    const [W, H, , ctx] = fitCanvas(this.canvas);
    ctx.fillStyle = cssVar("--instr-bg") || "#0a0f1a"; ctx.fillRect(0, 0, W, H);
    if (!this.view) return;
    const d = { ...this._dims(), W, H };

    // Hillshade, clipped to the DEM footprint.
    if (this.shade && this.grid) {
      const b = this.grid.bounds;
      const [x0, y0] = this._project(b.north, b.west, d, W, H);
      const [x1, y1] = this._project(b.south, b.east, d, W, H);
      ctx.imageSmoothingEnabled = true;
      ctx.globalAlpha = 0.92;
      ctx.drawImage(this.shade, x0, y0, x1 - x0, y1 - y0);
      ctx.globalAlpha = 1;
      // footprint outline
      ctx.strokeStyle = "rgba(140,170,220,.35)"; ctx.lineWidth = 1;
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    }

    this._graticule(ctx, d, W, H);

    const P = (lat, lon) => this._project(lat, lon, d, W, H);
    if (this.layers.planned && this.plan && this.plan.length > 1) this._line(ctx, this.plan.map((p) => P(p[0], p[1])), cssVar("--c-planned"), 2, true, [6, 4]);
    if (this.layers.flown) this._drawPath(ctx, P);
    if (this.layers.markers && this.start) this._pin(ctx, P(this.start[0], this.start[1]), cssVar("--c-start"), "LAUNCH");
    if (this.layers.markers && this.target) this._pin(ctx, P(this.target[0], this.target[1]), cssVar("--c-target"), "TARGET");
    if (this.missile) this._missile(ctx, P(this.missile.lat, this.missile.lon));
  }

  _graticule(ctx, d, W, H) {
    ctx.strokeStyle = "rgba(120,150,200,.08)"; ctx.lineWidth = 1;
    ctx.fillStyle = cssVar("--instr-ink-dim") || "#7f8ba6"; ctx.font = "9px " + (cssVar("--font-mono") || "monospace");
    const step = niceStep(d.spanLat);
    const north = d.north, west = d.west;
    for (let lat = Math.ceil((north - d.spanLat) / step) * step; lat <= north; lat += step) {
      const y = ((north - lat) / d.spanLat) * H;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
      ctx.fillText(`${lat.toFixed(2)}°`, 4, y - 2);
    }
    for (let lon = Math.ceil(west / step) * step; lon <= west + d.spanLonDeg; lon += step) {
      const x = ((lon - west) / d.spanLonDeg) * W;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
      ctx.fillText(`${lon.toFixed(2)}°`, x + 2, H - 4);
    }
  }

  _line(ctx, pts, color, w, glow, dash) {
    ctx.save();
    if (glow) { ctx.shadowColor = color; ctx.shadowBlur = 6; }
    if (dash) ctx.setLineDash(dash);
    ctx.strokeStyle = color; ctx.lineWidth = w; ctx.lineJoin = "round";
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
    ctx.stroke(); ctx.restore();
  }

  _drawPath(ctx, P) {
    if (this.path.length < 2) return;
    const colFor = (s) => s === "BOOST" ? cssVar("--c-boost")
      : (s === "TERMINAL" || s === "IMPACT") ? cssVar("--c-terminal") : cssVar("--c-actual");
    let seg = [P(this.path[0].lat, this.path[0].lon)]; let cur = this.path[0].stage;
    for (let i = 1; i < this.path.length; i++) {
      seg.push(P(this.path[i].lat, this.path[i].lon));
      if (this.path[i].stage !== cur || i === this.path.length - 1) {
        this._line(ctx, seg, colFor(cur), 2.2, true);
        seg = [P(this.path[i].lat, this.path[i].lon)]; cur = this.path[i].stage;
      }
    }
  }

  _pin(ctx, [x, y], color, label) {
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.6;
    ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.arc(x, y, 1.6, 0, Math.PI * 2); ctx.fill();
    ctx.beginPath(); ctx.moveTo(x, y - 9); ctx.lineTo(x, y + 9); ctx.moveTo(x - 9, y); ctx.lineTo(x + 9, y); ctx.stroke();
    ctx.font = "9px " + (cssVar("--font-mono") || "monospace");
    ctx.fillText(label, x + 9, y - 7);
  }

  _missile(ctx, [x, y]) {
    const yaw = (this.missile.yaw || 0) * DEG;
    const stage = this.missile.stage;
    const color = stage === "TERMINAL" || stage === "IMPACT" ? cssVar("--c-terminal") : cssVar("--c-actual");
    ctx.save(); ctx.translate(x, y); ctx.rotate(yaw);
    ctx.shadowColor = color; ctx.shadowBlur = 8;
    ctx.fillStyle = color; ctx.strokeStyle = color; ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.moveTo(0, -8); ctx.lineTo(5, 6); ctx.lineTo(0, 3); ctx.lineTo(-5, 6); ctx.closePath(); ctx.fill();
    ctx.restore();
    if (this.missile.label) {
      ctx.save(); ctx.shadowColor = "rgba(0,0,0,.6)"; ctx.shadowBlur = 4;
      ctx.fillStyle = cssVar("--instr-ink") || "#cdd7ea"; ctx.font = "10px " + (cssVar("--font-mono") || "monospace");
      ctx.fillText(this.missile.label, x + 10, y + 3);
      ctx.restore();
    }
  }
}

// --- hillshade --------------------------------------------------------------
// Hypsometric ramp (low -> high): deep green, green, tan, brown, grey, snow.
const HYPSO = [
  [0.00, [40, 78, 58]], [0.20, [74, 108, 56]], [0.42, [140, 140, 78]],
  [0.60, [140, 104, 62]], [0.78, [110, 92, 84]], [0.92, [150, 150, 156]], [1.00, [222, 224, 220]],
];
function hypso(h) {
  for (let i = 1; i < HYPSO.length; i++) {
    if (h <= HYPSO[i][0]) {
      const [t0, c0] = HYPSO[i - 1], [t1, c1] = HYPSO[i];
      const f = (h - t0) / (t1 - t0 || 1);
      return [c0[0] + (c1[0] - c0[0]) * f, c0[1] + (c1[1] - c0[1]) * f, c0[2] + (c1[2] - c0[2]) * f];
    }
  }
  return HYPSO[HYPSO.length - 1][1];
}

function buildHillshade(grid, colorize = false) {
  const { rows, cols, elev, z_min, z_max } = grid;
  const off = document.createElement("canvas");
  off.width = cols; off.height = rows;
  const ctx = off.getContext("2d");
  const img = ctx.createImageData(cols, rows);
  const range = Math.max(1, z_max - z_min);
  const lx = -0.6, ly = -0.6, lz = 0.53;
  const zScale = 0.9;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const i = r * cols + c;
      const zl = elev[r * cols + Math.max(0, c - 1)];
      const zr = elev[r * cols + Math.min(cols - 1, c + 1)];
      const zu = elev[Math.max(0, r - 1) * cols + c];
      const zd = elev[Math.min(rows - 1, r + 1) * cols + c];
      const nx = (zl - zr) * zScale, ny = (zu - zd) * zScale;
      const nl = Math.hypot(nx, ny, 30);
      const shade = clamp((nx * lx + ny * ly + 30 * lz) / (nl * Math.hypot(lx, ly, lz)), 0, 1);
      const hyp = (elev[i] - z_min) / range;
      const o = i * 4;
      if (colorize) {
        const [cr, cg, cb] = hypso(hyp);
        const k = 0.5 + shade * 0.75;
        img.data[o] = clamp(cr * k, 0, 255); img.data[o + 1] = clamp(cg * k, 0, 255); img.data[o + 2] = clamp(cb * k, 0, 255);
      } else {
        const v = clamp((26 + hyp * 60) * (0.45 + shade * 0.85), 8, 210);
        img.data[o] = v * 0.82; img.data[o + 1] = v * 0.92; img.data[o + 2] = v * 1.08;
      }
      img.data[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

function niceStep(span) {
  const raw = span / 6;
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / pow;
  return (n >= 5 ? 5 : n >= 2 ? 2 : 1) * pow;
}
