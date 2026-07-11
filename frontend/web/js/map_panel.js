// MapPanel — a map widget with two modes and shared overlays.
//
//   DEM  : the self-contained canvas hillshade (Map2D) with layer toggles
//          (planned / flown / markers) and an elevation-colour switch.
//   Map  : an external slippy map (Leaflet, no-label tiles) that draws the DEM
//          footprint rectangle from its GPS corners plus the same overlays.
//
// Screens talk to one object; MapPanel forwards data to whichever renderer is
// active and keeps the other in sync. Leaflet is lazy-loaded on first switch so
// the DEM mode needs no network at all.

import { el, cssVar, fmtLat, fmtLon, toast } from "./util.js";
import { Map2D } from "./map2d.js";

let _leafletPromise = null;
function loadLeaflet() {
  if (_leafletPromise) return _leafletPromise;
  _leafletPromise = new Promise((resolve, reject) => {
    if (window.L) return resolve(window.L);
    const css = document.createElement("link");
    css.rel = "stylesheet"; css.href = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css";
    document.head.appendChild(css);
    const s = document.createElement("script");
    s.src = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js";
    s.onload = () => resolve(window.L);
    s.onerror = () => reject(new Error("Leaflet failed to load (offline?)"));
    document.head.appendChild(s);
  });
  return _leafletPromise;
}

export class MapPanel {
  constructor(host, { onClick = null, legend = "mission", extraControls = [] } = {}) {
    this.onClick = onClick;
    this.mode = "dem";
    this.data = { grid: null, plan: null, path: [], start: null, target: null, missile: null };

    // DOM
    this.canvas = el("canvas");
    // `isolation:isolate` keeps Leaflet's internal z-indexed panes from painting
    // over our sibling controls / HUD / legend (which otherwise vanish in Map mode).
    this.leafletEl = el("div", { style: { position: "absolute", inset: "0", display: "none", background: "var(--instr-bg)", isolation: "isolate" } });
    this.infoHud = el("div", { class: "instr__hud instr__hud--tl mono" });
    this.coordHud = el("div", { class: "instr__hud instr__hud--br mono" });
    this.legendEl = el("div", { class: "instr__legend" });

    this.modeSeg = el("div", { class: "seg", style: { padding: "2px" } }, [
      segBtn("DEM", true, () => this.setMode("dem")),
      segBtn("Map", false, () => this.setMode("map")),
    ]);
    this.controls = el("div", { class: "instr__controls" }, [
      this.modeSeg,
      this._layersBtn(),
      iconBtn("my_location", "Fit", () => this.fit()),
      ...extraControls,
    ]);

    host.append(el("div", { class: "instr" }, [this.canvas, this.leafletEl, this.controls, this.infoHud, this.coordHud, this.legendEl]));

    this.dem = new Map2D(this.canvas);
    this.dem.onClick = (lat, lon) => this._emitClick(lat, lon);
    this.canvas.addEventListener("pointermove", (e) => this._coordFromCanvas(e));
    this._legendKind = legend;
    this._renderLegend();

    // Leaflet needs an explicit size recompute (+refit) when its container box
    // changes — e.g. the widget going fullscreen — or it keeps the old viewport.
    this._ro = new ResizeObserver(() => {
      if (this.mode === "map" && this.map) requestAnimationFrame(() => this.map.invalidateSize());
    });
    this._ro.observe(this.leafletEl);
  }

  // --- mode -----------------------------------------------------------------
  async setMode(mode) {
    if (mode === this.mode) return;
    this.mode = mode;
    Array.from(this.modeSeg.children).forEach((b, i) => b.setAttribute("aria-selected", String((mode === "dem") === (i === 0))));
    this._layerMenuItems && this._layerMenuItems.elev && (this._layerMenuItems.elev.style.display = mode === "dem" ? "" : "none");
    if (mode === "map") {
      this.canvas.style.display = "none"; this.leafletEl.style.display = "block";
      try { await this._ensureLeaflet(); this._syncLeaflet(); }
      catch (e) { this.infoHud.innerHTML = `<b>Map unavailable</b> — ${e.message}`; }
    } else {
      this.leafletEl.style.display = "none"; this.canvas.style.display = "block";
      this.dem.invalidate();
    }
    this._renderLegend();
  }

  // --- data forwarding ------------------------------------------------------
  setDEM(grid) {
    this.data.grid = grid; this.dem.setDEM(grid);
    if (this.map) { this._drawFootprint(); this._leafletFit(); }
  }

  // Route every click through a bounds check so endpoints can't be placed
  // outside the loaded DEM footprint (in either DEM or Map mode).
  _emitClick(lat, lon) {
    if (!this.onClick) return;
    const g = this.data.grid;
    if (g) {
      const b = g.bounds;
      if (lat < b.south || lat > b.north || lon < b.west || lon > b.east) {
        this.setInfo("<b>Outside DEM bounds</b> — pick a point inside the footprint");
        toast("Point is outside the DEM footprint", "warn");
        return;
      }
    }
    this.onClick(lat, lon);
  }
  setPlan(traj) { this.data.plan = traj; this.dem.setPlan(traj); this._syncLeafletVectors(); }
  resetPath() { this.data.path = []; this.dem.resetPath(); this._syncLeafletVectors(); }
  pushPath(lat, lon, stage) { this.data.path.push({ lat, lon, stage }); this.dem.pushPath(lat, lon, stage); this._syncLeafletVectors(); }
  setPathPoints(pts) { this.data.path = pts; this.dem.setPathPoints(pts); this._syncLeafletVectors(); }
  setStart(gps) { this.data.start = gps; this.dem.setStart(gps); this._syncLeafletMarkers(); }
  setTarget(gps) { this.data.target = gps; this.dem.setTarget(gps); this._syncLeafletMarkers(); }
  setMissile(m) { this.data.missile = m; this.dem.setMissile(m); this._syncLeafletMissile(); }
  invalidate() { this.dem.invalidate(); }
  setInfo(html) { this.infoHud.innerHTML = html; }
  get grid() { return this.data.grid; }

  fit() {
    if (this.mode === "map" && this.map) return this._leafletFit();
    const g = this.data.grid; if (g) this.dem.fit(g.bounds);
  }
  fitPoints(pts) {
    if (this.mode === "map" && this.map && pts?.length) {
      const b = bounds(pts); this.map.fitBounds([[b.s, b.w], [b.n, b.e]], { padding: [30, 30] }); return;
    }
    this.dem.fitPoints(pts);
    this._pendingFit = pts;
  }

  // --- layers ---------------------------------------------------------------
  _layersBtn() {
    const btn = iconBtn("layers", "Layers", (e) => { e.stopPropagation(); this._toggleMenu(); });
    const menu = el("div", { class: "map-menu", style: { display: "none" } });
    this._layerMenuItems = {
      planned: layerToggle("Planned route", true, (on) => { this.dem.setLayer("planned", on); this._syncLeafletVectors(); this._renderLegend(); }),
      flown: layerToggle("Flown path", true, (on) => { this.dem.setLayer("flown", on); this._syncLeafletVectors(); this._renderLegend(); }),
      markers: layerToggle("Markers", true, (on) => { this.dem.setLayer("markers", on); this._syncLeafletMarkers(); this._renderLegend(); }),
      elev: layerToggle("Elevation colour", false, (on) => this.dem.setElevColor(on)),
    };
    menu.append(this._layerMenuItems.planned, this._layerMenuItems.flown, this._layerMenuItems.markers, this._layerMenuItems.elev);
    document.body.append(menu);
    this._menu = menu; this._menuBtn = btn;
    document.addEventListener("click", () => (menu.style.display = "none"));
    return btn;
  }
  _toggleMenu() {
    const m = this._menu, open = m.style.display === "flex";
    if (open) { m.style.display = "none"; return; }
    const r = this._menuBtn.getBoundingClientRect();
    Object.assign(m.style, {
      display: "flex", flexDirection: "column", gap: "2px", position: "fixed",
      top: `${r.bottom + 6}px`, left: `${Math.min(r.left, innerWidth - 190)}px`, zIndex: "var(--z-popover)",
      background: "var(--surface-2)", border: "1px solid var(--line-2)", borderRadius: "var(--r-sm)", padding: "6px", minWidth: "180px", boxShadow: "var(--shadow-float)",
    });
  }

  _renderLegend() {
    const items = [];
    const L = this.dem.layers;
    if (L.markers) { items.push(leg("--c-start", "Launch"), leg("--c-target", "Target")); }
    if (L.planned) items.push(leg("--c-planned", "Planned"));
    if (L.flown) items.push(leg("--c-actual", "Flown"));
    if (this.mode === "map") items.push(leg("--c-planned", "DEM area", true));
    this.legendEl.replaceChildren(...items);
  }

  _coordFromCanvas(e) {
    if (this.mode !== "dem" || !this.dem.view) return;
    const r = this.canvas.getBoundingClientRect();
    const d = this.dem._dims();
    const [la, lo] = this.dem._unproject(e.clientX - r.left, e.clientY - r.top, { ...d, W: r.width, H: r.height }, r.width, r.height);
    this.coordHud.innerHTML = `${fmtLat(la)} ${fmtLon(lo)}`;
  }

  // --- Leaflet --------------------------------------------------------------
  async _ensureLeaflet() {
    if (this.map) return;
    this.L = await loadLeaflet();
    const L = this.L;
    this.map = L.map(this.leafletEl, { zoomControl: true, attributionControl: true, preferCanvas: true });
    this._setTiles();
    this.map.on("click", (e) => this._emitClick(e.latlng.lat, e.latlng.lng));
    this.map.on("mousemove", (e) => (this.coordHud.innerHTML = `${fmtLat(e.latlng.lat)} ${fmtLon(e.latlng.lng)}`));
    this._layers = {};
    this._onTheme = () => this._setTiles();
    window.addEventListener("themechange", this._onTheme);
  }
  _setTiles() {
    if (!this.map) return;
    const L = this.L;
    const light = document.documentElement.dataset.theme === "light";
    const url = light
      ? "https://{s}.basemaps.cartocdn.com/light_nolabels/{z}/{x}/{y}{r}.png"
      : "https://{s}.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}{r}.png";
    if (this._tileLayer) this.map.removeLayer(this._tileLayer);
    this._tileLayer = L.tileLayer(url, { subdomains: "abcd", maxZoom: 19, attribution: "&copy; OpenStreetMap &copy; CARTO" }).addTo(this.map);
    this._tileLayer.bringToBack?.();
  }
  _syncLeaflet() {
    this._drawFootprint(); this._syncLeafletVectors(); this._syncLeafletMarkers(); this._syncLeafletMissile();
    // Recompute the container size FIRST (it was display:none until now), then fit
    // — otherwise fitBounds runs against a zero-size map and the view/clicks drift.
    requestAnimationFrame(() => { this.map.invalidateSize(); this._leafletFit(); });
  }
  _leafletFit() {
    if (!this.map) return;
    const pts = this.data.path.length ? this.data.path.map((p) => [p.lat, p.lon]) : (this.data.plan?.map((p) => [p[0], p[1]]));
    if (pts?.length) { const b = bounds(pts); this.map.fitBounds([[b.s, b.w], [b.n, b.e]], { padding: [30, 30] }); }
    else if (this.data.grid) { const g = this.data.grid.bounds; this.map.fitBounds([[g.south, g.west], [g.north, g.east]]); }
  }
  _drawFootprint() {
    if (!this.map || !this.data.grid) return;
    const g = this.data.grid.bounds, L = this.L;
    this._layers.footprint && this.map.removeLayer(this._layers.footprint);
    this._layers.footprint = L.rectangle([[g.south, g.west], [g.north, g.east]], {
      color: cssVar("--c-planned") || "#4d8dff", weight: 1.5, fill: true, fillOpacity: 0.05, dashArray: "6 4",
      interactive: false,
    }).addTo(this.map);
  }
  _syncLeafletVectors() {
    if (!this.map) return;
    const L = this.L, l = this._layers;
    ["planned", "flown"].forEach((k) => { if (l[k]) { l[k].forEach ? l[k].forEach((x) => this.map.removeLayer(x)) : this.map.removeLayer(l[k]); l[k] = null; } });
    if (this.dem.layers.planned && this.data.plan?.length > 1) {
      l.planned = L.polyline(this.data.plan.map((p) => [p[0], p[1]]), { color: cssVar("--c-planned"), weight: 2, dashArray: "6 5", opacity: 0.9 }).addTo(this.map);
    }
    if (this.dem.layers.flown && this.data.path.length > 1) {
      l.flown = [];
      let seg = [[this.data.path[0].lat, this.data.path[0].lon]]; let cur = this.data.path[0].stage;
      const push = (st) => { if (seg.length > 1) l.flown.push(L.polyline(seg, { color: stageColor(cur), weight: 3, opacity: 0.95 }).addTo(this.map)); };
      for (let i = 1; i < this.data.path.length; i++) {
        seg.push([this.data.path[i].lat, this.data.path[i].lon]);
        if (this.data.path[i].stage !== cur || i === this.data.path.length - 1) { push(); seg = [[this.data.path[i].lat, this.data.path[i].lon]]; cur = this.data.path[i].stage; }
      }
    }
  }
  _syncLeafletMarkers() {
    if (!this.map) return;
    const L = this.L, l = this._layers, show = this.dem.layers.markers;
    ["start", "target"].forEach((k) => { if (l[k]) { this.map.removeLayer(l[k]); l[k] = null; } });
    if (show && this.data.start) l.start = pin(L, this.data.start, cssVar("--c-start"), "LAUNCH").addTo(this.map);
    if (show && this.data.target) l.target = pin(L, this.data.target, cssVar("--c-target"), "TARGET").addTo(this.map);
  }
  _syncLeafletMissile() {
    if (!this.map) return;
    const L = this.L, l = this._layers, m = this.data.missile;
    if (l.missile) { this.map.removeLayer(l.missile); l.missile = null; }
    if (!m) return;
    const color = m.stage === "TERMINAL" || m.stage === "IMPACT" ? cssVar("--c-terminal") : cssVar("--c-actual");
    const icon = L.divIcon({ className: "", html: `<div style="transform:rotate(${m.yaw || 0}deg);color:${color};font-size:16px;line-height:1;filter:drop-shadow(0 0 4px ${color})">▲</div><div style="font:600 10px var(--font-mono);color:var(--instr-ink);white-space:nowrap;margin-top:2px">${m.label || ""}</div>`, iconSize: [16, 16], iconAnchor: [8, 8] });
    l.missile = L.marker([m.lat, m.lon], { icon }).addTo(this.map);
  }

  destroy() { this._ro?.disconnect(); this._menu?.remove(); if (this._onTheme) window.removeEventListener("themechange", this._onTheme); this.dem.destroy(); this.map?.remove(); }
}

// --- helpers ----------------------------------------------------------------
function bounds(pts) {
  let s = 90, n = -90, w = 180, e = -180;
  for (const p of pts) { s = Math.min(s, p[0]); n = Math.max(n, p[0]); w = Math.min(w, p[1]); e = Math.max(e, p[1]); }
  const dLat = Math.max(n - s, 0.02) * 0.15, dLon = Math.max(e - w, 0.02) * 0.15;
  return { s: s - dLat, n: n + dLat, w: w - dLon, e: e + dLon };
}
function stageColor(s) { return s === "BOOST" ? cssVar("--c-boost") : (s === "TERMINAL" || s === "IMPACT") ? cssVar("--c-terminal") : cssVar("--c-actual"); }
function pin(L, gps, color, label) {
  return L.circleMarker([gps[0], gps[1]], { radius: 6, color, weight: 2, fill: true, fillColor: color, fillOpacity: 0.35 }).bindTooltip(label, { permanent: true, direction: "right", className: "map-pin-label" });
}
function segBtn(label, selected, onClick) { const b = el("button", { class: "seg__btn", "aria-selected": String(selected), text: label }); b.addEventListener("click", (e) => { e.stopPropagation(); onClick(); }); return b; }
function iconBtn(icon, title, onClick) { return el("button", { class: "instr__btn", title, "aria-label": title, onClick }, [el("span", { class: "mi", text: icon })]); }
function leg(colorVar, label, dashed) { return el("span", {}, [el("i", { style: { color: `var(${colorVar})`, borderTopStyle: dashed ? "dashed" : "solid" } }), el("span", { text: label })]); }
function layerToggle(label, on, cb) {
  const cb_ = el("input", { type: "checkbox", checked: on ? "checked" : null });
  cb_.checked = on;
  cb_.addEventListener("change", (e) => { e.stopPropagation(); cb(cb_.checked); });
  const row = el("label", { class: "map-menu__row", onClick: (e) => e.stopPropagation() }, [cb_, el("span", { text: label })]);
  return row;
}
