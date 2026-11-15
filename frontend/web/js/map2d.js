// Self-contained tactical map — canvas hillshade of the DEM plus vector overlays
// (DEM footprint, launch/target pins, planned + flown trajectories, live missile
// marker with id label). No external tile provider, so it works offline and is
// unaffected by the light/dark theme, exactly as the brief asks. Pan by drag,
// zoom by wheel; click reports a lat/lon back to the host screen.

import { fitCanvas, cssVar, clamp } from "./util.js";
import { MAP_PAINT, strokeRoute, paintMapLabel } from "./map_presentation.js";

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
    this.pullup = null;        // [lat,lon] terminal-guidance engage point
    this.onClick = null;
    this.layers = { planned: true, flown: true, markers: true, pullup: true };
    this.elevColor = true;
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
  setPullup(gps) { this.pullup = gps; this._dirty = true; }
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
    ctx.fillStyle = MAP_PAINT.background; ctx.fillRect(0, 0, W, H);
    if (!this.view) { this._empty(ctx, W, H); return; }
    const d = { ...this._dims(), W, H };

    // Hillshade, clipped to the DEM footprint.
    if (this.shade && this.grid) {
      const b = this.grid.bounds;
      const [x0, y0] = this._project(b.north, b.west, d, W, H);
      const [x1, y1] = this._project(b.south, b.east, d, W, H);
      ctx.imageSmoothingEnabled = true;
      ctx.globalAlpha = 1;
      ctx.drawImage(this.shade, x0, y0, x1 - x0, y1 - y0);
      ctx.globalAlpha = 1;
      // footprint outline
      ctx.strokeStyle = MAP_PAINT.edge; ctx.lineWidth = 1;
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    }

    this._graticule(ctx, d, W, H);

    const P = (lat, lon) => this._project(lat, lon, d, W, H);
    if (this.layers.planned && this.plan && this.plan.length > 1) this._line(ctx, this.plan.map((p) => P(p[0], p[1])), cssVar("--c-planned"), 2, false, [9, 7]);
    if (this.layers.flown) this._drawPath(ctx, P);
    if (this.layers.pullup && this.pullup) this._pullup(ctx, P(this.pullup[0], this.pullup[1]));
    if (this.layers.markers && this.start) this._pin(ctx, P(this.start[0], this.start[1]), cssVar("--c-start"), "Launch");
    if (this.layers.markers && this.target) this._pin(ctx, P(this.target[0], this.target[1]), cssVar("--c-target"), "Target");
    if (this.missile) this._missile(ctx, P(this.missile.lat, this.missile.lon));
  }

  _empty(ctx, W, H) {
    ctx.save(); ctx.translate(W / 2, H / 2 - 18);
    ctx.strokeStyle = '#8fa493'; ctx.lineWidth = 1.5;
    for (const [x,y] of [[-26,-13],[-8,-20],[10,-13]]) {
      ctx.beginPath(); ctx.roundRect(x, y, 16, 29, 3); ctx.stroke();
    }
    ctx.textAlign = 'center'; ctx.fillStyle = MAP_PAINT.ink;
    ctx.font = '500 13px system-ui, sans-serif'; ctx.fillText('No terrain loaded', 0, 42);
    ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = MAP_PAINT.muted;
    ctx.fillText('Select a DEM to view its relief.', 0, 63); ctx.restore();
  }

  _graticule(ctx, d, W, H) {
    // Keep the same coordinate spacing, with marginal ticks instead of a full
    // black grid through the terrain. Labels remain available at the edges.
    ctx.save(); ctx.strokeStyle = MAP_PAINT.grid; ctx.lineWidth = 1;
    ctx.fillStyle = MAP_PAINT.muted; ctx.font = '10px system-ui, sans-serif';
    const step = niceStep(d.spanLat), north = d.north, west = d.west;
    for (let lat = Math.ceil((north - d.spanLat) / step) * step; lat <= north; lat += step) {
      const y = ((north - lat) / d.spanLat) * H;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(10, y); ctx.moveTo(W - 10, y); ctx.lineTo(W, y); ctx.stroke();
      if (y > 78 && y < H - 45) ctx.fillText(`${lat.toFixed(2)}°`, 15, y + 3);
    }
    for (let lon = Math.ceil(west / step) * step; lon <= west + d.spanLonDeg; lon += step) {
      const x = ((lon - west) / d.spanLonDeg) * W;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, 10); ctx.moveTo(x, H - 10); ctx.lineTo(x, H); ctx.stroke();
      if (x > 55 && x < W - 55) ctx.fillText(`${lon.toFixed(2)}°`, x - 13, H - 14);
    }
    ctx.restore();
  }

  _line(ctx, pts, color, width, _glow, dash) {
    strokeRoute(ctx, pts, color, width, dash);
  }

  _drawPath(ctx, P) {
    if (this.path.length < 2) return;
    const colFor = (s) => s === "BOOST" ? cssVar("--c-boost")
      : (s === "TERMINAL" || s === "IMPACT") ? cssVar("--c-terminal") : cssVar("--c-actual");
    let seg = [P(this.path[0].lat, this.path[0].lon)]; let cur = this.path[0].stage;
    for (let i = 1; i < this.path.length; i++) {
      seg.push(P(this.path[i].lat, this.path[i].lon));
      if (this.path[i].stage !== cur || i === this.path.length - 1) {
        this._line(ctx, seg, colFor(cur), 2.8, false);
        seg = [P(this.path[i].lat, this.path[i].lon)]; cur = this.path[i].stage;
      }
    }
  }

  _pin(ctx, [x, y], color, label) {
    ctx.save(); ctx.fillStyle = color; ctx.strokeStyle = MAP_PAINT.paper; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.restore(); paintMapLabel(ctx, label, x + 11, y < 100 ? y + 13 : y - 31, color);
  }

  _pullup(ctx, [x, y]) {
    const color = cssVar('--c-terminal');
    ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = MAP_PAINT.paper; ctx.lineWidth = 1.6;
    ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.restore(); paintMapLabel(ctx, 'Pull-up', x + 11, y + 10, color);
  }

  _missile(ctx, [x, y]) {
    const yaw = (this.missile.yaw || 0) * DEG, stage = this.missile.stage;
    const color = stage === 'TERMINAL' || stage === 'IMPACT' ? cssVar('--c-terminal') : cssVar('--c-actual');
    ctx.save(); ctx.translate(x, y);
    ctx.fillStyle = MAP_PAINT.paper; ctx.strokeStyle = 'rgba(52,72,63,.28)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(0, 0, 12, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.rotate(yaw); ctx.fillStyle = color;
    ctx.beginPath(); ctx.moveTo(0, -8); ctx.lineTo(5, 6); ctx.lineTo(0, 3); ctx.lineTo(-5, 6); ctx.closePath(); ctx.fill();
    ctx.restore();
    paintMapLabel(ctx, this.missile.label, x + 17, y - 12, color);
  }

}

// --- hillshade --------------------------------------------------------------
// Muted elevation ramp (low -> high); these colors do not represent land cover.
const HYPSO = [
  [0.00, [112, 147, 120]], [0.20, [143, 166, 133]], [0.42, [175, 181, 145]],
  [0.60, [191, 181, 153]], [0.78, [181, 172, 154]], [0.92, [193, 192, 183]], [1.00, [218, 219, 211]],
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
        const k = 0.72 + shade * 0.40;
        img.data[o] = clamp(cr * k, 0, 255); img.data[o + 1] = clamp(cg * k, 0, 255); img.data[o + 2] = clamp(cb * k, 0, 255);
      } else {
        const v = clamp((158 + hyp * 48) * (0.76 + shade * 0.30), 95, 235);
        img.data[o] = v * 0.97; img.data[o + 1] = v; img.data[o + 2] = v * 0.96;
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
