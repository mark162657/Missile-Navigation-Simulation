// Planning screen: choose a DEM + missile, set launch/target, tune the pathfinder,
// run A*, and watch the route appear on the map. A successful plan is stored on
// the app and carried to Mission Control.

import { el, $, clear, toast, fmtDist, nf, fmtLat, fmtLon } from "./util.js";
import { api } from "./api.js";
import { Workspace } from "./widgets.js";
import { MapPanel } from "./map_panel.js";

const MAX_FLIGHT_TIME_S = 7200; // 2 hr hard ceiling (matches SimulationConfig)
const DEFAULTS = {
  impact_angle_deg: -30, detonation_radius_m: 25,
  wind_speed_ref_ms: 8, wind_from_deg: null,
  heuristic_weight: 2.0,
  max_flight_time_s: MAX_FLIGHT_TIME_S, // default to the maximum
};

export class PlanningScreen {
  constructor(root, app) {
    this.root = root; this.app = app;
    this.built = false;
    this.pinMode = "start";
    this.state = {
      dem: null, profile: null,
      start: null, target: null,
      plan: null,
      config: { ...DEFAULTS },
    };
  }

  activate() {
    if (!this.built) this._build();
    this._syncCatalog();
    this._refreshActions();
    this.map?.invalidate();
  }
  deactivate() {}

  _build() {
    this.ws = new Workspace(this.root);
    this._buildMap();
    this._buildMissionConfig();
    this._buildPathfinder();
    this._buildMissileConfig();
    this._buildDemConfig();
    this._buildTerminal();
    this.built = true;
  }

  _refreshActions() {
    const launch = el("button", { class: "btn btn--primary btn--sm", disabled: !this.state.plan, onClick: () => this._launch() }, [
      el("span", { text: "Proceed to Mission" }), el("span", { class: "mi", text: "arrow_forward" }),
    ]);
    this.launchBtn = launch;
    this.app.setActions([this.app.hiddenWidgetsControl(this.ws), launch]);
  }

  // --- Widgets --------------------------------------------------------------
  _buildMap() {
    this.mapWidget = this.ws.add({ id: "plan-map", title: "DEM · Tactical Map", cols: 8, rows: 9, instrument: true, flush: true,
      build: (body) => {
        this.map = new MapPanel(body, { legend: "planning", onClick: (lat, lon) => this._placePin(lat, lon) });
        this.map.setInfo("<b>Click</b> map to place launch / target");
      } });
  }

  _buildMissionConfig() {
    this.ws.add({ id: "plan-config", title: "Mission Configuration", cols: 4, rows: 9,
      build: (body) => {
        this.demSelect = select([], (v) => this._selectDem(v));
        this.profileSelect = select([], (v) => this._selectProfile(v));

        const pinToggle = el("div", { class: "seg", role: "tablist", style: { marginTop: "2px" } }, [
          segBtn("Launch", true, () => this._setPinMode("start")),
          segBtn("Target", false, () => this._setPinMode("target")),
        ]);
        this.pinToggle = pinToggle;

        this.startFields = gpsFields();
        this.targetFields = gpsFields();
        this.startFields.wire(() => this._readFields("start"));
        this.targetFields.wire(() => this._readFields("target"));

        this.cfgImpact = numInput(DEFAULTS.impact_angle_deg, (v) => (this.state.config.impact_angle_deg = v));
        this.cfgDeton = numInput(DEFAULTS.detonation_radius_m, (v) => (this.state.config.detonation_radius_m = v));
        this.cfgWind = numInput(DEFAULTS.wind_speed_ref_ms, (v) => (this.state.config.wind_speed_ref_ms = v));
        this.cfgWindDir = textInput("auto", (v) => (this.state.config.wind_from_deg = v === "" ? null : Number(v)));
        this.cfgMaxDur = numInput(DEFAULTS.max_flight_time_s, (v) => (this.state.config.max_flight_time_s = v), { min: 1, max: MAX_FLIGHT_TIME_S, step: 1 });

        body.append(
          field("DEM Terrain", this.demSelect),
          field("Missile Profile", this.profileSelect),
          subsection("place", "Endpoints"),
          el("div", { class: "field" }, [el("span", { class: "label", text: "Set on map" }), pinToggle]),
          labeledGps("Launch", this.startFields.node),
          labeledGps("Target", this.targetFields.node),
          el("p", { class: "hint", style: { marginTop: "2px" }, text: "Enter absolute values; N/S and E/W set the hemisphere (positive = N/E)." }),
          subsection("tune", "Mission Parameters"),
          formGrid([
            field("Impact angle °", this.cfgImpact),
            field("Detonation r (m)", this.cfgDeton),
            field("Wind (m/s)", this.cfgWind),
            field("Wind from °", this.cfgWindDir),
            field("Max duration (s)", this.cfgMaxDur),
          ]),
          el("p", { class: "hint", style: { marginTop: "2px" }, text: `Flight is aborted after this many seconds (max ${MAX_FLIGHT_TIME_S}). Defaults to the maximum.` }),
        );
      } });
  }

  _buildPathfinder() {
    this.ws.add({ id: "plan-pathfinder", title: "Pathfinding Algorithm", cols: 4, rows: 5,
      build: (body, widget) => {
        widget.setMeta(el("span", { class: "chip", text: "A* · B-spline" }));
        this.hwVal = el("span", { class: "kv__v mono", text: DEFAULTS.heuristic_weight.toFixed(1) });
        const slider = el("input", { class: "range", type: "range", min: "1", max: "4", step: "0.1", value: String(DEFAULTS.heuristic_weight) });
        slider.addEventListener("input", () => { this.state.config.heuristic_weight = Number(slider.value); this.hwVal.textContent = Number(slider.value).toFixed(1); });

        this.planBtn = el("button", { class: "btn btn--primary btn--block", onClick: () => this._runPlan() }, [
          el("span", { class: "mi", text: "route" }), el("span", { text: "Run Pathfinding" }),
        ]);

        body.append(
          el("p", { class: "hint", text: "Search over DEM pixels. Higher weight biases the heuristic for a faster, more direct route; 1.0 is the guaranteed shortest path." }),
          el("div", { class: "kv", style: { marginTop: "8px" } }, [el("span", { class: "kv__k", text: "Heuristic weight" }), this.hwVal]),
          slider,
          el("div", { style: { height: "10px" } }),
          this.planBtn,
          el("div", { class: "kv", style: { marginTop: "10px" } }, [el("span", { class: "kv__k", text: "Waypoints" }), (this.planWpts = el("span", { class: "kv__v", text: "—" }))]),
          el("div", { class: "kv" }, [el("span", { class: "kv__k", text: "Route length" }), (this.planLen = el("span", { class: "kv__v", text: "—" }))]),
        );
      } });
  }

  _buildMissileConfig() {
    this.ws.add({ id: "plan-missile", title: "Missile Configuration", cols: 4, rows: 5,
      build: (body, widget) => {
        this.missileWidget = widget;
        widget.setMeta(el("button", { class: "btn btn--sm btn--ghost", title: "New profile from this one", onClick: () => this._openProfileForm() }, [el("span", { class: "mi", text: "add" }), el("span", { text: "New" })]));
        this.missileBody = body; this._renderMissile();
      } });
  }

  _buildDemConfig() {
    this.ws.add({ id: "plan-dem", title: "DEM Configuration", cols: 4, rows: 4,
      build: (body) => { this.demBody = body; this._renderDem(); } });
  }

  _buildTerminal() {
    this.ws.add({ id: "plan-terminal", title: "Pathfinding Console", cols: 8, rows: 4, instrument: false, flush: true,
      build: (body) => {
        this.term = el("div", { class: "terminal" }, [
          el("div", { class: "terminal__line", html: '<span class="t-tag">[gnc]</span> planning console ready. select endpoints and run pathfinding.' }),
        ]);
        body.append(this.term);
      } });
  }

  // --- catalog / selection --------------------------------------------------
  _syncCatalog() {
    const dems = this.app.store.dems, profiles = this.app.store.profiles;
    setOptions(this.demSelect, dems.map((d) => ({ value: d.name, label: prettyDem(d.name) })));
    setOptions(this.profileSelect, profiles.map((p) => ({ value: p.name, label: p.name })));
    if (dems.length && !this.state.dem) this._selectDem(dems[0].name);
    if (profiles.length && !this.state.profile) this._selectProfile(profiles[0].name);
  }

  async _selectDem(name) {
    const meta = this.app.store.dems.find((d) => d.name === name);
    if (!meta) return;
    this.demSelect.value = name;
    this.state.dem = { meta, grid: null };
    this._renderDem();
    try {
      const grid = await api.demGrid(name);
      this.state.dem.grid = grid;
      this.map.setDEM(grid);
      this._renderDem();
    } catch (e) { toast(`DEM load failed: ${e.message}`, "err"); }
  }

  _selectProfile(name) {
    this.state.profile = this.app.store.profiles.find((p) => p.name === name) || null;
    this.profileSelect.value = name;
    this._renderMissile();
  }

  // --- endpoints ------------------------------------------------------------
  _setPinMode(mode) {
    this.pinMode = mode;
    Array.from(this.pinToggle.children).forEach((b, i) =>
      b.setAttribute("aria-selected", String((mode === "start") === (i === 0))));
  }

  async _placePin(lat, lon) {
    const which = this.pinMode;
    let alt = 0;
    if (this.state.dem) {
      try { const r = await api.demElevation(this.state.dem.meta.name, lat, lon); alt = r.elevation_m ?? 0; } catch { /* ignore */ }
    }
    this.state[which] = [lat, lon, alt];
    (which === "start" ? this.startFields : this.targetFields).set(lat, lon, alt);
    if (which === "start") this.map.setStart(this.state.start); else this.map.setTarget(this.state.target);
    this._refreshActions();
  }

  _readFields(which) {
    const f = which === "start" ? this.startFields : this.targetFields;
    const v = f.get();
    if (v) { this.state[which] = v; if (which === "start") this.map.setStart(v); else this.map.setTarget(v); }
  }

  // --- rendering panels -----------------------------------------------------
  _renderMissile() {
    if (!this.missileBody) return;
    clear(this.missileBody);
    const p = this.state.profile;
    if (!p) { this.missileBody.append(emptyState("deployed_code", "No profile", "Select a missile profile.")); return; }
    const b = p.basic, d = p.detailed || {}, w = p.warhead || {};
    this.missileBody.append(el("div", { class: "profile-card" }, [
      el("div", { class: "profile-card__name", text: p.name }),
      el("div", { class: "profile-card__spec" }, [
        kv("Cruise", `${nf(b.cruise_speed)} km/h`), kv("Max spd", `${nf(b.max_speed)} km/h`),
        kv("Max g", `${nf(b.max_g_force, 1)}`), kv("Range", `${nf(b.max_range)} km`),
        kv("Cruise AGL", `${nf(b.cruise_agl_min)}–${nf(b.cruise_agl_max)} m`), kv("Turn rate", `${nf(b.sustained_turn_rate, 1)}°/s`),
        kv("Mass", `${nf(d.mass_kg)} kg`), kv("Fuel", `${nf(d.fuel_capacity_kg)} kg`),
        kv("IMU", d.imu_grade || "—"), kv("Warhead", w.warhead_name || "—"),
        kv("Blast r", `${nf(w.blast_radius_m)} m`), kv("GPS", `${nf(d.gps_update_rate_hz)} Hz`),
      ]),
    ]));
  }

  // Compact profile editor: create a new missile from the selected one.
  _openProfileForm() {
    const base = this.state.profile;
    const b = base?.basic || {};
    clear(this.missileBody);
    const fields = {
      name: textPrefilled(base ? `${base.name} Copy` : "New Missile"),
      cruise_speed: numPrefilled(b.cruise_speed ?? 880),
      max_speed: numPrefilled(b.max_speed ?? 920),
      min_speed: numPrefilled(b.min_speed ?? 400),
      max_g_force: numPrefilled(b.max_g_force ?? 4),
      sustained_turn_rate: numPrefilled(b.sustained_turn_rate ?? 5),
      max_range: numPrefilled(b.max_range ?? 1600),
      cruise_agl_min: numPrefilled(b.cruise_agl_min ?? 20),
      cruise_agl_max: numPrefilled(b.cruise_agl_max ?? 70),
      min_altitude: numPrefilled(b.min_altitude ?? 30),
      max_altitude: numPrefilled(b.max_altitude ?? 1500),
    };
    const save = el("button", { class: "btn btn--primary btn--sm", onClick: () => this._saveProfile(fields) }, [el("span", { class: "mi", text: "save" }), el("span", { text: "Save profile" })]);
    const cancel = el("button", { class: "btn btn--sm btn--ghost", text: "Cancel", onClick: () => this._renderMissile() });
    this.missileBody.append(
      field("Name", fields.name),
      el("div", { class: "form-grid" }, [
        field("Cruise km/h", fields.cruise_speed), field("Max km/h", fields.max_speed),
        field("Min km/h", fields.min_speed), field("Max g", fields.max_g_force),
        field("Turn °/s", fields.sustained_turn_rate), field("Range km", fields.max_range),
        field("Cruise AGL min", fields.cruise_agl_min), field("Cruise AGL max", fields.cruise_agl_max),
        field("Min alt m", fields.min_altitude), field("Max alt m", fields.max_altitude),
      ]),
      el("div", { style: { display: "flex", gap: "8px", marginTop: "12px" } }, [save, cancel]),
    );
  }

  async _saveProfile(fields) {
    const name = fields.name.value.trim();
    if (!name) return toast("Profile needs a name", "err");
    const base = this.state.profile;
    const basic = {
      cruise_speed: num(fields.cruise_speed), max_speed: num(fields.max_speed), min_speed: num(fields.min_speed),
      max_acceleration: base?.basic?.max_acceleration ?? 39.24,
      min_altitude: num(fields.min_altitude), max_altitude: num(fields.max_altitude),
      max_g_force: num(fields.max_g_force),
      max_longitudinal_g_boost: base?.basic?.max_longitudinal_g_boost ?? 20,
      sustained_turn_rate: num(fields.sustained_turn_rate),
      sustained_g_force: base?.basic?.sustained_g_force ?? 1.8,
      evasive_turn_rate: base?.basic?.evasive_turn_rate ?? 9,
      max_range: num(fields.max_range), cruise_agl_min: num(fields.cruise_agl_min), cruise_agl_max: num(fields.cruise_agl_max),
    };
    try {
      const profiles = await api.saveProfile({ name, basic, detailed: base?.detailed || {}, warhead: base?.warhead || {} });
      this.app.store.profiles = profiles;
      setOptions(this.profileSelect, profiles.map((p) => ({ value: p.name, label: p.name })));
      this._selectProfile(name);
      toast(`Saved profile "${name}"`, "ok");
    } catch (e) { toast(`Save failed: ${e.message}`, "err"); }
  }

  _renderDem() {
    if (!this.demBody) return;
    clear(this.demBody);
    const dem = this.state.dem;
    if (!dem) { this.demBody.append(emptyState("terrain", "No DEM", "Select a DEM terrain tile.")); return; }
    const m = dem.meta, g = dem.grid;
    const rows = [
      kvRow("File", prettyDem(m.name)),
      kvRow("Grid", `${nf(m.width)} × ${nf(m.height)} px`),
      kvRow("Resolution", `${(m.resolution_deg * 3600).toFixed(1)}″ · ${nf(m.resolution_deg * 111320)} m`),
      kvRow("Latitude", `${m.bounds.south.toFixed(2)}° – ${m.bounds.north.toFixed(2)}°`),
      kvRow("Longitude", `${m.bounds.west.toFixed(2)}° – ${m.bounds.east.toFixed(2)}°`),
    ];
    if (g) rows.push(kvRow("Elevation", `${nf(g.z_min)} – ${nf(g.z_max)} m`));
    this.demBody.append(el("div", {}, rows));
  }

  // --- run ------------------------------------------------------------------
  async _runPlan() {
    if (this._planning) { this._abortPlan(); return; }
    if (!this.state.dem) return toast("Select a DEM first", "err");
    if (!this.state.start || !this.state.target) return toast("Place a launch and target point", "err");
    this._planning = true;
    this._planCtrl = new AbortController();
    this._setPlanBtn(true);
    this._log(`[a*] planning ${prettyDem(this.state.dem.meta.name)} · w=${this.state.config.heuristic_weight.toFixed(1)}`, "");
    this._log(`[a*] launch ${fmtLat(this.state.start[0])} ${fmtLon(this.state.start[1])} → target ${fmtLat(this.state.target[0])} ${fmtLon(this.state.target[1])}`, "");
    const cursor = this._log("[a*] searching… (click Abort to cancel)", "run");

    try {
      const res = await api.plan({
        dem_name: this.state.dem.meta.name,
        start_gps: this.state.start, target_gps: this.state.target,
        heuristic_weight: this.state.config.heuristic_weight,
      }, this._planCtrl.signal);
      cursor.remove();
      res.log.forEach((line) => this._log(line, line.includes("done") ? "ok" : ""));
      this.state.plan = res;
      this.map.setPlan(res.trajectory);
      this.map.fitPoints(res.trajectory);
      this.planWpts.textContent = nf(res.waypoints);
      this.planLen.textContent = fmtDist(routeLength(res.trajectory));
      this._refreshActions();
      toast("Route planned", "ok");
    } catch (e) {
      cursor.remove();
      if (e.name === "AbortError") {
        this._log("[abort] pathfinding cancelled by operator", "warn");
        toast("Pathfinding aborted", "warn");
      } else {
        this._log(`[error] ${e.message}`, "err");
        toast(`Pathfinding failed: ${e.message}`, "err");
      }
    } finally {
      this._planning = false;
      this._planCtrl = null;
      this._setPlanBtn(false);
    }
  }

  _abortPlan() {
    this._planCtrl?.abort();
  }

  _setPlanBtn(running) {
    clear(this.planBtn);
    if (running) {
      this.planBtn.classList.add("btn--danger");
      this.planBtn.append(el("span", { class: "mi", text: "stop" }), el("span", { text: "Abort Pathfinding" }));
    } else {
      this.planBtn.classList.remove("btn--danger");
      this.planBtn.append(el("span", { class: "mi", text: "route" }), el("span", { text: "Run Pathfinding" }));
    }
  }

  _log(text, kind = "") {
    const tag = text.match(/^\[[^\]]+\]/)?.[0] || "";
    const rest = text.slice(tag.length);
    const line = el("div", { class: `terminal__line ${kind === "ok" ? "is-ok" : kind === "err" ? "is-err" : kind === "warn" ? "is-warn" : ""}` });
    if (kind === "run") line.append(document.createTextNode(text + " "), el("span", { class: "terminal__cursor" }));
    else line.innerHTML = `<span class="t-tag">${tag}</span>${escapeHtml(rest)}`;
    this.term.append(line);
    this.term.scrollTop = this.term.scrollHeight;
    return line;
  }

  _launch() {
    if (!this.state.plan) return;
    this.app.store.plan = {
      dem_name: this.state.dem.meta.name,
      dem_grid: this.state.dem.grid,
      start_gps: this.state.start,
      target_gps: this.state.target,
      trajectory: this.state.plan.trajectory,
      profile_name: this.state.profile?.name,
      profile: this.state.profile,
      config: { ...this.state.config },
    };
    toast("Plan armed — opening Mission Control", "ok");
    this.app.navigateTo("mission");
  }
}

// --- small builders ---------------------------------------------------------
function routeLength(traj) {
  let m = 0;
  for (let i = 1; i < traj.length; i++) {
    const a = traj[i - 1], b = traj[i];
    const mLat = 111320, mLon = 111320 * Math.cos(a[0] * Math.PI / 180);
    m += Math.hypot((b[0] - a[0]) * mLat, (b[1] - a[1]) * mLon);
  }
  return m;
}
function prettyDem(name) {
  return name.replace(/^merged_dem_/, "").replace(/\.tif$/, "").replace(/_/g, " ");
}
function field(label, control) { return el("div", { class: "field" }, [el("span", { class: "label", text: label }), control]); }
function formGrid(fields) { return el("div", { class: "form-grid" }, fields); }
function labeledGps(label, node) { return el("div", { class: "field" }, [el("span", { class: "label", text: label }), node]); }
function subsection(icon, title) {
  return el("div", { class: "subsection__head" }, [el("span", { class: "mi", text: icon }), el("span", { class: "subsection__title", text: title }), el("span", { class: "subsection__rule" })]);
}
function select(options, onChange) {
  const s = el("select", { class: "select" });
  setOptions(s, options);
  s.addEventListener("change", () => onChange(s.value));
  return s;
}
function setOptions(sel, options) {
  const cur = sel.value;
  clear(sel);
  options.forEach((o) => sel.append(el("option", { value: o.value, text: o.label })));
  if (options.some((o) => o.value === cur)) sel.value = cur;
}
function segBtn(label, selected, onClick) {
  const b = el("button", { class: "seg__btn", role: "tab", "aria-selected": String(selected), text: label });
  b.addEventListener("click", onClick);
  return b;
}
function numInput(value, onChange, opts = {}) {
  const attrs = { class: "input", type: "number", step: String(opts.step ?? "any"), value: String(value) };
  if (opts.min != null) attrs.min = String(opts.min);
  if (opts.max != null) attrs.max = String(opts.max);
  const i = el("input", attrs);
  const emit = (clampVal) => {
    if (i.value === "") return onChange(null);
    let n = Number(i.value);
    if (Number.isNaN(n)) return;
    if (clampVal) {
      if (opts.min != null && n < opts.min) n = opts.min;
      if (opts.max != null && n > opts.max) n = opts.max;
      if (String(n) !== i.value) i.value = String(n);
    }
    onChange(n);
  };
  i.addEventListener("input", () => emit(false));
  i.addEventListener("change", () => emit(true)); // clamp to [min,max] on commit
  return i;
}
function textInput(placeholder, onChange) {
  const i = el("input", { class: "input", type: "text", placeholder });
  i.addEventListener("input", () => onChange(i.value.trim()));
  return i;
}
function textPrefilled(value) { return el("input", { class: "input", type: "text", value: String(value) }); }
function numPrefilled(value) { return el("input", { class: "input", type: "number", step: "any", value: String(value) }); }
function num(input) { return Number(input.value || 0); }
function gpsFields() {
  const lat = el("input", { class: "input", type: "number", step: "any", min: "0", placeholder: "latitude" });
  const lon = el("input", { class: "input", type: "number", step: "any", min: "0", placeholder: "longitude" });
  const alt = el("input", { class: "input", type: "number", step: "any", placeholder: "altitude m" });
  const latH = hemiSelect(["N", "S"]);
  const lonH = hemiSelect(["E", "W"]);
  const row = (a, b) => el("div", { style: { display: "grid", gridTemplateColumns: "1fr 58px", gap: "6px" } }, [a, b]);
  const node = el("div", { style: { display: "grid", gap: "6px" } }, [row(lat, latH), row(lon, lonH), alt]);
  return {
    node,
    set(la, lo, al) {
      lat.value = Math.abs(la).toFixed(5); latH.value = la >= 0 ? "N" : "S";
      lon.value = Math.abs(lo).toFixed(5); lonH.value = lo >= 0 ? "E" : "W";
      alt.value = Math.round(al);
    },
    get() {
      if (lat.value === "" || lon.value === "") return null;
      const la = Math.abs(Number(lat.value)) * (latH.value === "S" ? -1 : 1);
      const lo = Math.abs(Number(lon.value)) * (lonH.value === "W" ? -1 : 1);
      return [la, lo, Number(alt.value || 0)];
    },
    wire(cb) { [lat, latH, lon, lonH, alt].forEach((i) => i.addEventListener("change", cb)); },
  };
}
function hemiSelect(opts) {
  const s = el("select", { class: "select", style: { paddingRight: "22px" } });
  opts.forEach((o) => s.append(el("option", { value: o, text: o })));
  return s;
}
function kv(k, v) { return el("div", { class: "kv" }, [el("span", { class: "kv__k", text: k }), el("span", { class: "kv__v", text: v })]); }
function kvRow(k, v) { return kv(k, v); }
function emptyState(icon, title, msg) {
  return el("div", { class: "state" }, [el("span", { class: "mi", text: icon }), el("span", { class: "state__title", text: title }), el("span", { class: "state__msg", text: msg })]);
}
function legendItem(colorVar, label) { return el("span", {}, [el("i", { style: { color: `var(${colorVar})` } }), el("span", { text: label })]); }
function iconBtn(icon, title, onClick) { return el("button", { class: "instr__btn", title, "aria-label": title, onClick }, [el("span", { class: "mi", text: icon })]); }
function escapeHtml(s) { return s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c])); }
