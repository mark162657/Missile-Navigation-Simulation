// Planning screen: choose a DEM + missile, set launch/target, pick Rust (CFA*)
// or C++ (A*) pathfinding, tune parameters, and watch the route appear on the
// map. A successful plan is stored on the app and carried to Mission Control.

import { el, $, clear, toast, fmtDist, nf, fmtLat, fmtLon } from "./util.js";
import { api } from "./api.js";
import { Workspace } from "./widgets.js";
import { MapPanel } from "./map_panel.js";
import { terminalPullupGps, MAX_IMPACT_DEG, MIN_IMPACT_DEG, RECOMMENDED_IMPACT_DEG } from "./guidance.js";

const MAX_FLIGHT_TIME_S = 7200; // 2 hr hard ceiling (matches SimulationConfig)
const K_PRESETS = [2, 4, 8, 16];
const MIN_K = 1;
const MAX_K = 256;

const DEFAULTS = {
  engine: "rust",          // "rust" (CFA*) | "cpp" (A*)
  downsample_k: 4,         // CFA* coarse-grid downsampling factor k
  heuristic_weight: 2.0,
  impact_angle_deg: -30,
  detonation_radius_m: 25,
  wind_speed_ref_ms: 8,
  wind_from_deg: null,
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
    this._syncEngineUI();
    this._refreshActions();
    this.map?.invalidate();
  }
  deactivate() {}

  _build() {
    this.ws = new Workspace(this.root);
    this.ws.grid.classList.add("planning-workspace");
    // Row 1 (9 rows): Map (8) + Mission Config (4)
    this._buildMap();
    this._buildMissionConfig();
    // Row 2 (6 rows): Pathfinding Engine (5) + Pathfinding Console (7)
    this._buildPathfinder();
    this._buildTerminal();
    // Row 3 (5 rows): Missile Profile (6) + DEM Terrain Profile (6)
    this._buildMissileConfig();
    this._buildDemConfig();
    this.built = true;
  }

  _refreshActions() {
    const isRust = this.state.config.engine === "rust";
    const engineChip = el("span", {
      class: `chip ${isRust ? "chip--rust" : "chip--cpp"} topbar__engine-chip`,
      title: isRust ? "Active engine: Rust CFA*" : "Active engine: C++ A*",
    }, [
      el("span", { class: "dot" }),
      el("span", { text: isRust ? `Rust · CFA* (k=${this.state.config.downsample_k})` : "C++ · A*" }),
    ]);

    const launch = el("button", { class: "btn btn--primary btn--sm", disabled: !this.state.plan, onClick: () => this._launch() }, [
      el("span", { text: "Proceed to Mission" }), el("span", { class: "mi", text: "arrow_forward" }),
    ]);
    this.launchBtn = launch;
    this.app.setActions([engineChip, this.app.hiddenWidgetsControl(this.ws), launch]);
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

        const pinToggle = el("div", { class: "seg seg--full", role: "tablist", style: { marginTop: "2px" } }, [
          segBtn("Launch", true, () => this._setPinMode("start")),
          segBtn("Target", false, () => this._setPinMode("target")),
        ]);
        this.pinToggle = pinToggle;

        this.startFields = gpsFields();
        this.targetFields = gpsFields();
        this.startFields.wire(() => this._readFields("start"));
        this.targetFields.wire(() => this._readFields("target"));

        this.cfgImpact = numInput(DEFAULTS.impact_angle_deg, (v) => this._onImpactChange(v), { min: -MAX_IMPACT_DEG, max: -MIN_IMPACT_DEG });
        this.cfgDeton = numInput(DEFAULTS.detonation_radius_m, (v) => (this.state.config.detonation_radius_m = v));
        this.cfgWind = numInput(DEFAULTS.wind_speed_ref_ms, (v) => (this.state.config.wind_speed_ref_ms = v));
        this.cfgWindDir = textInput("auto", (v) => (this.state.config.wind_from_deg = v === "" ? null : Number(v)));
        this.cfgMaxDur = numInput(DEFAULTS.max_flight_time_s, (v) => (this.state.config.max_flight_time_s = v), { min: 1, max: MAX_FLIGHT_TIME_S, step: 1 });

        body.append(
          field("DEM Terrain", this.demSelect),
          field("Missile Profile", this.profileSelect),
          subsection("place", "Endpoints"),
          el("div", { class: "field" }, [el("span", { class: "label", text: "Map click target" }), pinToggle]),
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
          (this.impactHint = el("p", { class: "hint", style: { marginTop: "4px" } })),
          el("p", { class: "hint", style: { marginTop: "2px" }, text: `Flight aborts after this limit (max ${MAX_FLIGHT_TIME_S}s).` }),
        );
        this._updateImpactStyle();
      } });
  }

  _buildPathfinder() {
    this.ws.add({ id: "plan-pathfinder", title: "Pathfinding Engine", cols: 5, rows: 7,
      build: (body, widget) => {
        this.pfWidget = widget;

        // Engine selector cards: Rust (CFA*) vs C++ (A*)
        this.engineCards = {
          rust: this._makeEngineCard("rust", "Rust", "CFA*", "Coarse-to-Fine A*"),
          cpp: this._makeEngineCard("cpp", "C++", "A*", "Legacy full-grid A*"),
        };
        const enginePicker = el("div", { class: "engine-picker", role: "group", "aria-label": "Pathfinding engine" }, [
          this.engineCards.rust.node,
          this.engineCards.cpp.node,
        ]);

        this.engineNotice = el("div", { class: "pf-notice", hidden: true });

        // CFA* downsample factor k (Rust-only)
        this.kInput = el("input", {
          class: "input pf-k-input",
          type: "number",
          min: String(MIN_K),
          max: String(MAX_K),
          step: "1",
          value: String(DEFAULTS.downsample_k),
          "aria-label": "CFA* downsample factor k",
        });
        const commitK = (clampVal) => {
          if (this.kInput.value === "") return;
          let k = Math.round(Number(this.kInput.value));
          if (!Number.isFinite(k)) return;
          if (clampVal) {
            k = Math.max(MIN_K, Math.min(MAX_K, k));
            if (String(k) !== this.kInput.value) this.kInput.value = String(k);
          } else if (k < MIN_K) {
            return;
          }
          this._setDownsampleK(k, false);
        };
        this.kInput.addEventListener("input", () => commitK(false));
        this.kInput.addEventListener("change", () => commitK(true));

        this.kPresetBtns = K_PRESETS.map((k) => {
          const btn = el("button", {
            type: "button",
            class: "k-preset",
            text: `${k}×`,
            title: `Set downsample factor k = ${k}`,
            onClick: () => this._setDownsampleK(k, true),
          });
          btn.dataset.k = String(k);
          return btn;
        });
        this.kRatioBadge = el("span", { class: "pf-pill mono" });
        this.coarsePreview = el("div", { class: "pf-coarse-preview mono" });

        this.kBlock = el("div", { class: "pf-param-card pf-param-card--rust" }, [
          el("div", { class: "pf-param-card__head" }, [
            el("span", { class: "label", text: "CFA* Downsample Factor (k)" }),
            this.kRatioBadge,
          ]),
          el("div", { class: "pf-k-row" }, [
            this.kInput,
            el("div", { class: "pf-k-presets", role: "group", "aria-label": "Downsample presets" }, this.kPresetBtns),
          ]),
          this.coarsePreview,
        ]);

        // Heuristic weight slider
        this.hwVal = el("span", { class: "pf-pill mono", text: DEFAULTS.heuristic_weight.toFixed(1) });
        this.hwTag = el("span", { class: "pf-mode-tag" });
        this.hwSlider = el("input", {
          class: "range",
          type: "range",
          min: "1",
          max: "4",
          step: "0.1",
          value: String(DEFAULTS.heuristic_weight),
          "aria-label": "Heuristic weight",
        });
        this.hwSlider.addEventListener("input", () => {
          const v = Number(this.hwSlider.value);
          this.state.config.heuristic_weight = v;
          this._updateHeuristicReadout(v);
        });

        this.pfHint = el("p", { class: "hint" });
        const hwBlock = el("div", { class: "pf-param-card" }, [
          el("div", { class: "pf-param-card__head" }, [
            el("span", { class: "label", text: "Heuristic Weight (w)" }),
            el("div", { style: { display: "flex", alignItems: "center", gap: "6px" } }, [this.hwTag, this.hwVal]),
          ]),
          this.hwSlider,
          this.pfHint,
        ]);

        // Primary action button
        this.planBtn = el("button", { class: "btn btn--primary btn--block pf-run-btn", onClick: () => this._runPlan() }, [
          el("span", { class: "mi", text: "route" }), el("span", { text: "Run Rust CFA*" }),
        ]);

        // Compact 3-stat summary strip
        this.planWpts = el("div", { class: "pf-stat__v mono", text: "—" });
        this.planLen = el("div", { class: "pf-stat__v mono", text: "—" });
        this.planTime = el("div", { class: "pf-stat__v mono", text: "—" });
        const summary = el("div", { class: "pf-summary" }, [
          statCell("Waypoints", this.planWpts),
          statCell("Route Length", this.planLen),
          statCell("Solve Time", this.planTime),
        ]);

        body.append(
          el("div", { class: "pf-stack" }, [
            enginePicker,
            this.engineNotice,
            this.kBlock,
            hwBlock,
            this.planBtn,
            summary,
          ]),
        );

        this._updateHeuristicReadout(DEFAULTS.heuristic_weight);
        this._syncEngineUI();
      } });
  }

  _makeEngineCard(id, name, algo, desc) {
    const statusDot = el("span", { class: "engine-card__dot" });
    const statusText = el("span", { class: "engine-card__status-text", text: "Checking" });
    const node = el("button", {
      type: "button",
      class: `engine-card engine-card--${id}`,
      dataset: { engine: id },
      "aria-pressed": String(this.state.config.engine === id),
      onClick: () => this._selectEngine(id),
    }, [
      el("div", { class: "engine-card__top" }, [
        el("span", { class: "engine-card__name", text: name }),
        el("span", { class: "engine-card__algo mono", text: algo }),
      ]),
      el("div", { class: "engine-card__desc", text: desc }),
      el("div", { class: "engine-card__status" }, [statusDot, statusText]),
    ]);
    return { node, statusDot, statusText };
  }

  _selectEngine(engineId) {
    if (this.state.config.engine === engineId) return;
    this.state.config.engine = engineId;
    this._syncEngineUI();
    this._refreshActions();
  }

  _setDownsampleK(k, syncInput = true) {
    const clean = Math.max(MIN_K, Math.min(MAX_K, Math.round(k)));
    this.state.config.downsample_k = clean;
    if (syncInput && this.kInput) this.kInput.value = String(clean);
    this._updateDownsampleUI();
    this._refreshActions();
  }

  _updateDownsampleUI() {
    const k = this.state.config.downsample_k;
    if (this.kRatioBadge) {
      this.kRatioBadge.textContent = k === 1 ? "1:1 full res" : `1:${k * k} coarse cells`;
    }
    if (this.kPresetBtns) {
      this.kPresetBtns.forEach((b) => {
        b.classList.toggle("is-active", Number(b.dataset.k) === k);
      });
    }
    if (this.coarsePreview) {
      const m = this.state.dem?.meta;
      if (m) {
        const cr = Math.max(1, Math.floor(m.height / k));
        const cc = Math.max(1, Math.floor(m.width / k));
        const resM = (m.resolution_deg * 111320) * k;
        this.coarsePreview.textContent = `Coarse grid: ${nf(cc)} × ${nf(cr)} px · ~${nf(resM)} m/cell`;
      } else {
        this.coarsePreview.textContent = `Coarse stride: ${k} px per macro-cell`;
      }
    }
  }

  _updateHeuristicReadout(w) {
    if (this.hwVal) this.hwVal.textContent = w.toFixed(1);
    if (this.hwTag) {
      this.hwTag.textContent = w <= 1.05 ? "Optimal" : w <= 2.4 ? "Balanced" : "Greedy";
      this.hwTag.dataset.tone = w <= 1.05 ? "ok" : w <= 2.4 ? "primary" : "warn";
    }
  }

  _syncEngineUI() {
    if (!this.pfWidget) return;
    const engine = this.state.config.engine;
    const isRust = engine === "rust";
    const info = this.app.store.engines?.engines;

    // Header chip
    this.pfWidget.setMeta(el("span", { class: `chip ${isRust ? "chip--rust" : "chip--cpp"}` }, [
      el("span", { class: "dot" }),
      el("span", { text: isRust ? "CFA* · Rust" : "A* · C++" }),
    ]));

    // Engine cards active & availability states
    for (const id of ["rust", "cpp"]) {
      const card = this.engineCards[id];
      const active = engine === id;
      card.node.setAttribute("aria-pressed", String(active));
      card.node.classList.toggle("is-active", active);
      const st = info?.[id];
      if (st) {
        card.node.classList.toggle("is-unavailable", !st.available);
        card.statusDot.classList.toggle("is-ok", st.available);
        card.statusDot.classList.toggle("is-err", !st.available);
        card.statusText.textContent = st.available ? "Ready" : "Not built";
      } else {
        card.statusText.textContent = "Available";
      }
    }

    // Inline availability notice if selected engine is not compiled
    const selectedInfo = info?.[engine];
    if (this.engineNotice) {
      if (selectedInfo && !selectedInfo.available) {
        this.engineNotice.hidden = false;
        clear(this.engineNotice);
        this.engineNotice.append(
          el("span", { class: "mi", text: "warning" }),
          el("span", { text: selectedInfo.error || `${selectedInfo.label} backend is not built in this Python environment.` }),
        );
      } else {
        this.engineNotice.hidden = true;
      }
    }

    // Toggle CFA* k block visibility
    if (this.kBlock) this.kBlock.hidden = !isRust;
    this._updateDownsampleUI();

    // Update heuristic hint copy for engine context
    if (this.pfHint) {
      this.pfHint.textContent = isRust
        ? "CFA* searches a k× downsampled coarse grid first, then refines corridors at full resolution. Heuristic weight biases search speed vs. optimality."
        : "Full-resolution C++ A* over DEM pixels. 1.0 guarantees the shortest path; higher values trade optimality for speed.";
    }

    if (!this._planning) this._setPlanBtn(false);
  }

  _buildMissileConfig() {
    this.ws.add({ id: "plan-missile", title: "Missile Configuration", cols: 6, rows: 5,
      build: (body, widget) => {
        this.missileWidget = widget;
        widget.setMeta(el("button", { class: "btn btn--sm btn--ghost", title: "New profile from this one", onClick: () => this._openProfileForm() }, [el("span", { class: "mi", text: "add" }), el("span", { text: "New" })]));
        this.missileBody = body; this._renderMissile();
      } });
  }

  _buildDemConfig() {
    this.ws.add({ id: "plan-dem", title: "DEM Terrain Profile", cols: 6, rows: 5,
      build: (body) => { this.demBody = body; this._renderDem(); } });
  }

  _buildTerminal() {
    this.ws.add({ id: "plan-terminal", title: "Pathfinding Console", cols: 7, rows: 7, instrument: false, flush: true,
      build: (body, widget) => {
        widget.setMeta(el("button", {
          class: "btn btn--sm btn--ghost",
          title: "Clear console output",
          onClick: () => this._clearConsole(),
        }, [el("span", { class: "mi", text: "delete_sweep" }), el("span", { text: "Clear" })]));

        this.term = el("div", { class: "terminal" });
        body.append(this.term);
        this._log("[gnc] planning console ready — select endpoints and run pathfinding.", "");
      } });
  }

  _clearConsole() {
    if (!this.term) return;
    clear(this.term);
    this._log("[gnc] console cleared.", "");
  }

  // --- catalog / selection --------------------------------------------------
  _syncCatalog() {
    const dems = this.app.store.dems, profiles = this.app.store.profiles;
    setOptions(this.demSelect, dems.map((d) => ({ value: d.name, label: prettyDem(d.name) })));
    setOptions(this.profileSelect, profiles.map((p) => ({ value: p.name, label: p.name })));
    // Start with the smallest available tile on every platform. Large merged
    // DEMs remain selectable, but loading them fully can consume many GiB.
    if (dems.length && !this.state.dem) {
      const safest = dems.reduce((a, b) => demPixels(a) <= demPixels(b) ? a : b);
      this._selectDem(safest.name);
    }
    if (profiles.length && !this.state.profile) this._selectProfile(profiles[0].name);

    // Log detected engines once on first sync
    if (!this._loggedEngines && this.app.store.engines?.engines) {
      this._loggedEngines = true;
      const e = this.app.store.engines.engines;
      const rState = e.rust?.available ? "ready" : "unavailable";
      const cState = e.cpp?.available ? "ready" : "unavailable";
      this._log(`[sys] engines detected · Rust CFA* (${rState}) · C++ A* (${cState})`, "");
    }
  }

  async _selectDem(name) {
    const meta = this.app.store.dems.find((d) => d.name === name);
    if (!meta) return;
    this.demSelect.value = name;
    this.state.dem = { meta, grid: null };
    this._renderDem();
    this._updateDownsampleUI();
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
    this._refreshPullup(); // engage range depends on cruise speed + max-g
  }

  // --- impact angle / terminal pull-up --------------------------------------
  _onImpactChange(v) {
    this.state.config.impact_angle_deg = v;
    this._updateImpactStyle();
    this._refreshPullup();
  }

  // Colour the input + hint amber past the recommended angle (still allowed up to
  // the hard cap the numInput clamps to). Steep dives are energy-marginal.
  _updateImpactStyle() {
    if (!this.cfgImpact) return;
    const v = this.state.config.impact_angle_deg;
    const mag = v == null ? null : Math.abs(v);
    const tooSteep = mag != null && mag > RECOMMENDED_IMPACT_DEG;
    const tooShallow = mag != null && mag < MIN_IMPACT_DEG;
    const flag = tooSteep || tooShallow;
    this.cfgImpact.classList.toggle("is-warn", flag);
    if (this.impactHint) {
      this.impactHint.classList.toggle("is-warn", flag);
      this.impactHint.textContent = tooSteep
        ? `Steeper than the recommended ${RECOMMENDED_IMPACT_DEG}° — energy-marginal dive (hard cap ${MAX_IMPACT_DEG}°).`
        : tooShallow
          ? `Shallower than ${MIN_IMPACT_DEG}° skims the target — clamped to ${MIN_IMPACT_DEG}° on launch.`
          : `Dive angle at impact (negative = nose-down). Usable ${MIN_IMPACT_DEG}–${MAX_IMPACT_DEG}°, recommended ≤ ${RECOMMENDED_IMPACT_DEG}°.`;
    }
  }

  // Recompute where terminal guidance engages and mark it on the map. Cleared
  // (null) until a route, profile, and target are all available.
  _refreshPullup() {
    if (!this.map) return;
    const b = this.state.profile?.basic;
    const traj = this.state.plan?.trajectory;
    this._pullupGps = (b && traj && this.state.target)
      ? terminalPullupGps(b.cruise_speed, b.max_g_force, this.state.config.impact_angle_deg, traj, this.state.target)
      : null;
    this.map.setPullup(this._pullupGps);
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
    if (which === "target") this._refreshPullup();
    this._refreshActions();
  }

  _readFields(which) {
    const f = which === "start" ? this.startFields : this.targetFields;
    const v = f.get();
    if (v) { this.state[which] = v; if (which === "start") this.map.setStart(v); else { this.map.setTarget(v); this._refreshPullup(); } }
  }

  // --- rendering panels -----------------------------------------------------
  _renderMissile() {
    if (!this.missileBody) return;
    clear(this.missileBody);
    const p = this.state.profile;
    if (!p) { this.missileBody.append(emptyState("deployed_code", "No profile", "Select a missile profile.")); return; }
    const b = p.basic, d = p.detailed || {}, w = p.warhead || {};
    this.missileBody.append(el("div", { class: "profile-card" }, [
      el("div", { class: "profile-card__head" }, [
        el("div", { class: "profile-card__name", text: p.name }),
        w.warhead_name ? el("span", { class: "chip", text: w.warhead_name }) : null,
      ]),
      el("div", { class: "profile-card__spec" }, [
        kv("Cruise", `${nf(b.cruise_speed)} km/h`), kv("Max speed", `${nf(b.max_speed)} km/h`),
        kv("Max load", `${nf(b.max_g_force, 1)} g`), kv("Range", `${nf(b.max_range)} km`),
        kv("Cruise AGL", `${nf(b.cruise_agl_min)}–${nf(b.cruise_agl_max)} m`), kv("Turn rate", `${nf(b.sustained_turn_rate, 1)}°/s`),
        kv("Mass", `${nf(d.mass_kg)} kg`), kv("Fuel", `${nf(d.fuel_capacity_kg)} kg`),
        kv("IMU grade", d.imu_grade || "—"), kv("Blast radius", `${nf(w.blast_radius_m)} m`),
        kv("GPS rate", `${nf(d.gps_update_rate_hz)} Hz`), kv("Min altitude", `${nf(b.min_altitude)} m`),
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
      el("div", { class: "form-grid", style: { marginTop: "10px" } }, [
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
    const px = demPixels(m);
    const rows = [
      kvRow("File", prettyDem(m.name)),
      kvRow("Dimensions", `${nf(m.width)} × ${nf(m.height)} px`),
      kvRow("Total cells", `${(px / 1e6).toFixed(1)} M px`),
      kvRow("Resolution", `${(m.resolution_deg * 3600).toFixed(1)}″ · ${nf(m.resolution_deg * 111320)} m`),
      kvRow("Latitude", `${m.bounds.south.toFixed(2)}° – ${m.bounds.north.toFixed(2)}°`),
      kvRow("Longitude", `${m.bounds.west.toFixed(2)}° – ${m.bounds.east.toFixed(2)}°`),
    ];
    if (g) rows.push(kvRow("Elevation", `${nf(g.z_min)} – ${nf(g.z_max)} m`));
    this.demBody.append(el("div", { class: "dem-spec-grid" }, rows));
    if (isLargeDem(m)) {
      this.demBody.append(el("div", { class: "pf-notice", style: { marginTop: "12px" } }, [
        el("span", { class: "mi", text: "memory" }),
        el("span", { text: "Large terrain tile (500M+ px): full DEM arrays may require 10+ GiB RAM. Increase CFA* downsample k or use a smaller tile for quick checks." }),
      ]));
    }
  }

  // --- run ------------------------------------------------------------------
  async _runPlan() {
    if (this._planning) { this._abortPlan(); return; }
    if (!this.state.dem) return toast("Select a DEM first", "err");
    if (!this.state.start || !this.state.target) return toast("Place a launch and target point", "err");

    const engine = this.state.config.engine;
    const isRust = engine === "rust";
    const k = Math.max(MIN_K, Math.min(MAX_K, Math.round(Number(this.state.config.downsample_k) || DEFAULTS.downsample_k)));
    this.state.config.downsample_k = k;
    const tag = isRust ? "[cfa*]" : "[a*]";

    this._planning = true;
    this._planCtrl = new AbortController();
    this._setPlanBtn(true);

    const paramSummary = isRust
      ? `Rust CFA* · k=${k} · w=${this.state.config.heuristic_weight.toFixed(1)}`
      : `C++ A* · w=${this.state.config.heuristic_weight.toFixed(1)}`;
    this._log(`${tag} planning ${prettyDem(this.state.dem.meta.name)} (${paramSummary})`, "");
    this._log(`${tag} launch ${fmtLat(this.state.start[0])} ${fmtLon(this.state.start[1])} → target ${fmtLat(this.state.target[0])} ${fmtLon(this.state.target[1])}`, "");
    const cursor = this._log(`${tag} searching… (click Abort to cancel)`, "run");

    try {
      const payload = {
        dem_name: this.state.dem.meta.name,
        start_gps: this.state.start,
        target_gps: this.state.target,
        engine,
        heuristic_weight: this.state.config.heuristic_weight,
        downsample_k: k,
      };
      const res = await api.plan(payload, this._planCtrl.signal);
      cursor.remove();
      res.log.forEach((line) => this._log(line, line.includes("[done]") ? "ok" : ""));
      this.state.plan = res;
      this.map.setPlan(res.trajectory);
      this.map.fitPoints(res.trajectory);
      this._refreshPullup();
      this.planWpts.textContent = nf(res.waypoints);
      this.planLen.textContent = fmtDist(routeLength(res.trajectory));
      this.planTime.textContent = res.elapsed_s != null ? `${Number(res.elapsed_s).toFixed(2)} s` : "—";
      this._refreshActions();
      toast(`Route planned (${res.algorithm || (isRust ? "CFA*" : "A*")})`, "ok");
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
    if (!this.planBtn) return;
    clear(this.planBtn);
    const isRust = this.state.config.engine === "rust";
    if (running) {
      this.planBtn.classList.remove("btn--primary");
      this.planBtn.classList.add("btn--danger");
      this.planBtn.append(el("span", { class: "mi", text: "stop" }), el("span", { text: "Abort Pathfinding" }));
    } else {
      this.planBtn.classList.remove("btn--danger");
      this.planBtn.classList.add("btn--primary");
      this.planBtn.append(
        el("span", { class: "mi", text: "route" }),
        el("span", { text: isRust ? "Run Rust CFA*" : "Run C++ A*" }),
      );
    }
  }

  _log(text, kind = "") {
    const tag = text.match(/^\[[^\]]+\]/)?.[0] || "";
    const rest = text.slice(tag.length);
    const tagClass = tag === "[cfa*]" ? "t-tag t-tag--rust"
      : tag === "[a*]" ? "t-tag t-tag--cpp"
      : tag === "[dem]" ? "t-tag t-tag--dem"
      : tag === "[traj]" || tag === "[spline]" ? "t-tag t-tag--traj"
      : "t-tag";
    const line = el("div", { class: `terminal__line ${kind === "ok" ? "is-ok" : kind === "err" ? "is-err" : kind === "warn" ? "is-warn" : ""}` });
    if (kind === "run") {
      line.append(
        el("span", { class: tagClass, text: tag }),
        document.createTextNode(rest + " "),
        el("span", { class: "terminal__cursor" }),
      );
    } else {
      line.innerHTML = `<span class="${tagClass}">${escapeHtml(tag)}</span>${escapeHtml(rest)}`;
    }
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
      pullup_gps: this._pullupGps || null,
    };
    toast("Plan armed — opening Mission Control", "ok");
    this.app.navigateTo("mission");
  }
}

// --- small builders ---------------------------------------------------------
function statCell(label, valNode) {
  return el("div", { class: "pf-stat" }, [
    el("div", { class: "pf-stat__k", text: label }),
    valNode,
  ]);
}
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
function demPixels(dem) { return Number(dem?.width || 0) * Number(dem?.height || 0); }
function isLargeDem(dem) { return demPixels(dem) >= 500_000_000; }
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
function escapeHtml(s) { return s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c])); }
