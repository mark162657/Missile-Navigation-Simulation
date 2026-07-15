// Mission Control: the live monitoring workspace. Map + 3D viewer in the centre,
// a tabbed navigation/controls/weather monitor, a PFD, deviation charts, and the
// stage banner. Plays a recorded flight (replay) or drives the armed plan live
// over the simulation WebSocket.

import { el, $, clear, toast, nf, fmtDist, fmtTime, fmtLat, fmtLon, cssVar } from "./util.js";
import { api, connectLive } from "./api.js";
import { Workspace } from "./widgets.js";
import { MapPanel } from "./map_panel.js";
import { Viewer3D } from "./viewer3d.js";
import { PFD } from "./pfd.js";
import { MiniChart } from "./charts.js";
import { StageBanner } from "./stagebanner.js";
import { Player } from "./player.js";
import { terminalPullupGps } from "./guidance.js";

const SPEEDS = [1, 2, 5, 10];

export class MissionScreen {
  constructor(root, app) {
    this.root = root; this.app = app;
    this.built = false;
    this.player = null;
    this.live = null;
    this._armedPlan = null;  // the plan object this screen is currently set up for
    this.runState = "empty"; // 'empty' | 'ready' | 'running' | 'done'
    this._lastIdx = -1;
    this.wind = null;
    this.missileLabel = "TLAM-01";
    this.viewFactor = 1;     // sim-seconds per real-second for the live view (1 = real time)
    this.monDetailOpen = new Map();
  }

  activate() {
    if (!this.built) this._build();
    this._mountBanner();
    this._syncToPlan();
    this.map?.invalidate(); this.viewer?.invalidate();
  }

  deactivate() {
    this.player?.pause();
    if (this._monRaf != null) { cancelAnimationFrame(this._monRaf); this._monRaf = null; }
    // Leaving Mission Control ends any in-flight simulation; the run is marked
    // done so returning shows a Relaunch (not a stuck Abort) button.
    if (this.live) { this.live.stop(); this.live = null; this._planning = false; if (this.runState === "running") this.runState = "done"; }
  }

  // Mission Control always targets the currently armed plan. When a new route is
  // armed in Planning, set up a fresh ready-to-launch scene; when none is armed,
  // show the empty state. An in-progress run for the same plan is left alone.
  _syncToPlan() {
    const plan = this.app.store.plan;
    if (!plan) {
      if (this.live) { this.live.stop(); this.live = null; }
      this._armedPlan = null;
      this.runState = "empty";
      this.map?.setPullup(null); this.viewer?.setPullup(null);
      this.map?.setInfo("<b>No route armed</b> — plan a mission first");
      this._refreshActions();
      this._renderMonitor(null);
      return;
    }
    if (plan !== this._armedPlan) {
      this._armedPlan = plan;
      this._prepareLive(plan);
    }
    this._refreshActions();
  }

  _mountBanner() {
    const mount = $("#stageBannerMount");
    clear(mount);
    this.banner = new StageBanner(mount);
    if (this.player?.current) this.banner.update(this.player.current);
  }

  // --- build ----------------------------------------------------------------
  _build() {
    this.ws = new Workspace(this.root);
    this._buildMap();
    this._buildViewer();
    this._buildMonitor();
    this._buildPFD();
    this._buildStats();
    this.built = true;
  }

  _buildMap() {
    this.ws.add({ id: "m-map", title: "Tactical Map", cols: 5, rows: 7, instrument: true, flush: true,
      build: (body) => { this.map = new MapPanel(body, { legend: "mission" }); } });
  }

  _buildViewer() {
    this.ws.add({ id: "m-viewer", title: "3D Attitude · Third-person", cols: 4, rows: 7, instrument: true, flush: true,
      build: (body, widget) => {
        const canvas = el("canvas");
        this.viewHud = el("div", { class: "instr__hud instr__hud--bl mono" });
        const terrainBtn = icon("terrain", "Toggle terrain mesh", () => { this.viewer.showTerrain = !this.viewer.showTerrain; this.viewer.invalidate(); });
        body.append(el("div", { class: "instr" }, [canvas,
          el("div", { class: "instr__controls" }, [
            icon("center_focus_strong", "Recenter on missile", () => { this.viewer.follow = true; this.viewer.resetView(); }),
            icon("zoom_out_map", "Frame whole mission", () => this.viewer.frameCamera()),
            terrainBtn,
          ]),
          el("div", { class: "instr__hud instr__hud--tl", html: "<b>Drag</b> orbit · <b>scroll</b> zoom" }),
          this.viewHud,
          el("div", { class: "instr__legend" }, [leg("--c-planned", "Planned", true), leg("--c-boost", "Boost"), leg("--c-actual", "Cruise"), leg("--c-terminal", "Terminal")]),
        ]));
        this.viewer = new Viewer3D(canvas);
        this.viewer.label = this.missileLabel;
      } });
  }

  _buildMonitor() {
    this.ws.add({ id: "m-monitor", title: "Flight Monitor", cols: 3, rows: 11, flush: true,
      build: (body) => {
        const tabs = ["Navigation", "Controls", "Weather"];
        this.monBody = el("div", { style: { padding: "12px 14px", overflow: "auto", height: "100%" } });
        const tabbar = el("div", { class: "widget__section-tabs" }, tabs.map((t, i) =>
          el("button", { class: "seg__btn", role: "tab", "aria-selected": String(i === 0), text: t,
            onClick: (e) => { this.monTab = t; Array.from(tabbar.children).forEach((b) => b.setAttribute("aria-selected", String(b === e.currentTarget))); this._renderMonitor(); } })));
        this.monTab = "Navigation";
        body.append(tabbar, this.monBody);
        // Live telemetry rebuilds this panel ~10×/s; a full rebuild mid-click
        // destroys the <details> summary before the toggle fires, so diagnostics
        // won't expand during flight. Pause the per-frame rebuild while the pointer
        // is over the panel (so clicks / scrolls land), then refresh on leave.
        this._monHover = false;
        this.monBody.addEventListener("pointerenter", () => { this._monHover = true; });
        this.monBody.addEventListener("pointerleave", () => { this._monHover = false; this._renderMonitor(); });
        this._renderMonitor();
      } });
  }

  _buildPFD() {
    this.ws.add({ id: "m-pfd", title: "Primary Flight Display", cols: 5, rows: 4, instrument: true, flush: true,
      build: (body) => { this.pfd = new PFD(body); } });
  }

  _buildStats() {
    this.ws.add({ id: "m-stats", title: "Navigation & Guidance Deviation", cols: 4, rows: 4,
      build: (body) => {
        const grid = el("div", { style: { display: "grid", gridTemplateColumns: "1fr 1fr", gap: "14px" } });
        body.append(grid);
        this.charts = {
          pos: new MiniChart(grid, { title: "Position error", unit: "m", color: "--c-actual", threshold: 50 }),
          alt: new MiniChart(grid, { title: "Altitude error", unit: "m", color: "--primary" }),
          agl: new MiniChart(grid, { title: "Height AGL", unit: "m", color: "--success" }),
          spd: new MiniChart(grid, { title: "Ground speed", unit: "m/s", color: "--warn" }),
        };
      } });
  }

  // --- transport ------------------------------------------------------------
  _refreshActions() {
    const hasPlan = !!this.app.store.plan;

    // Primary button drives the live mission: Launch → Abort → Relaunch.
    this.playBtn = el("button", { class: "btn btn--primary btn--sm", disabled: !hasPlan, onClick: () => this._onMainButton() },
      [el("span", { class: "mi" }), el("span", {})]);
    this._applyMainButton();

    // Ghost button always available to arm a brand-new route.
    const newBtn = el("button", { class: "btn btn--sm btn--ghost", title: "Plan a new route", onClick: () => this.app.navigateTo("planning") },
      [el("span", { class: "mi", text: "route" }), el("span", { text: "New mission" })]);

    // Live playback speed (sim-seconds per real-second). 1× = real time. Takes
    // effect at the next launch (the running sim's pace is fixed once it starts).
    this.speedSeg = el("div", { class: "seg", title: "Live speed — applies at launch (1× = real time)" },
      SPEEDS.map((s) => el("button", { class: "seg__btn", "aria-selected": String(s === this.viewFactor), text: `${s}×`,
        onClick: (e) => { this.viewFactor = s; Array.from(this.speedSeg.children).forEach((b) => b.setAttribute("aria-selected", String(b === e.currentTarget))); } })));

    this.app.setActions([this.app.hiddenWidgetsControl(this.ws), newBtn, this.speedSeg, this.playBtn]);
  }

  // Reflect runState on the primary button (icon + label + danger styling).
  _applyMainButton() {
    if (!this.playBtn) return;
    const mi = this.playBtn.querySelector(".mi");
    const label = this.playBtn.querySelector("span:last-child");
    const spec = {
      empty:   ["play_arrow", "Launch",   false],
      ready:   ["play_arrow", "Launch",   false],
      running: ["stop",       "Abort",    true],
      done:    ["replay",     "Relaunch", false],
    }[this.runState] || ["play_arrow", "Launch", false];
    mi.textContent = spec[0];
    label.textContent = spec[1];
    this.playBtn.classList.toggle("btn--danger", spec[2]);
    this.playBtn.classList.toggle("btn--primary", !spec[2]);
    this.playBtn.disabled = !this.app.store.plan;
  }

  _onMainButton() {
    if (!this.app.store.plan) return;
    if (this.runState === "running") this._abortRun();
    else this._launchLive();   // ready / done → (re)launch fresh
  }

  // Build the ready-to-launch scene for an armed plan (route drawn, no flight yet).
  _prepareLive(plan) {
    this.live?.stop(); this.live = null; this._planning = false;
    this.player?.destroy();
    this.wind = plan.config;
    this._setupScene({
      startGps: plan.start_gps, targetGps: plan.target_gps,
      demName: plan.dem_name, demGrid: plan.dem_grid, trajectory: plan.trajectory,
    });
    this._setPlayer(new Player([], "live"));
    this.banner?.reset();
    this.runState = "ready";
    this._applyMainButton();
    this.map.setInfo("<b>Route armed</b> — press Launch to fly");
  }

  async _setupScene({ startGps, targetGps, demName, demGrid, trajectory }) {
    let grid = demGrid;
    if (!grid && demName) { try { grid = await api.demGrid(demName); } catch { /* ignore */ } }
    if (grid) { this.map.setDEM(grid); this.viewer.setDEM(grid); }
    this.viewer.setStart(startGps); this.map.setStart(startGps);
    if (targetGps) { this.viewer.setTarget(targetGps); this.map.setTarget(targetGps); }
    this.map.setPlan(trajectory); this.viewer.setPlan(trajectory);
    const pullup = this._planPullup(this.app.store.plan, trajectory, targetGps);
    this.map.setPullup(pullup); this.viewer.setPullup(pullup);
    this.map.resetPath(); this.viewer.resetPath();
    this._lastIdx = -1;
    this.viewer.follow = true; this.viewer.resetView(); this._fitMap();
  }

  // Terminal pull-up point for the map: prefer the value computed at plan time,
  // else derive it from the armed plan's profile + impact angle (mirrors the
  // planner). Static per plan — the engage range depends only on config.
  _planPullup(plan, trajectory, targetGps) {
    if (!plan) return null;
    if (plan.pullup_gps) return plan.pullup_gps;
    const b = plan.profile?.basic;
    if (!b || !trajectory || !targetGps) return null;
    return terminalPullupGps(b.cruise_speed, b.max_g_force, plan.config?.impact_angle_deg, trajectory, targetGps);
  }

  _setPlayer(player) {
    this.player = player;
    Object.values(this.charts).forEach((c) => c.reset());
    player.onFrame((f, i) => this._renderFrame(f, i));
  }

  _fitMap() {
    const traj = this.app.store.plan?.trajectory;
    if (traj?.length) return this.map.fitPoints(traj.map((p) => [p[0], p[1]]));
    const g = this.map.grid; if (g) this.map.fit(g.bounds);
  }

  // --- live -----------------------------------------------------------------
  _launchLive() {
    const plan = this.app.store.plan;
    if (!plan) return;
    // Clear any previous flight so each launch starts clean.
    this.map.resetPath(); this.viewer.resetPath(); this._lastIdx = -1;
    Object.values(this.charts).forEach((c) => c.reset());
    this.banner?.reset();
    this.player?.destroy();
    this.player = new Player([], "live");
    this._setPlayer(this.player);
    this.runState = "running";
    this._planning = true;
    this._applyMainButton();
    toast(plan.trajectory?.length ? "Launching — flying planned route…" : "Planning route — the search can take a while…");
    this.live = connectLive(
      { profile: plan.profile_name, config: { ...plan.config, dem_name: plan.dem_name, start_gps: plan.start_gps, target_gps: plan.target_gps, trajectory: plan.trajectory, view_factor: this.viewFactor } },
      {
        onMessage: (msg) => this._onLiveMessage(msg),
        onError: () => toast("Live simulation error", "err"),
        // Fires on normal completion (not on an operator abort, which suppresses it).
        onClose: () => { this.live = null; this._planning = false; if (this.runState === "running") { this.runState = "done"; this._applyMainButton(); } },
      });
  }

  // Abort the running simulation and reset to a clean ready-to-launch state, so a
  // new mission can start immediately without reloading.
  _abortRun() {
    this.live?.stop(); this.live = null; this._planning = false;
    this.map.resetPath(); this.viewer.resetPath(); this._lastIdx = -1;
    Object.values(this.charts).forEach((c) => c.reset());
    this.banner?.reset();
    this.player?.destroy();
    this._setPlayer(new Player([], "live"));
    this.runState = "ready";
    this._applyMainButton();
    this.map.setInfo("<b>Mission aborted</b> — press Launch to fly again");
    toast("Mission aborted — ready to relaunch", "warn");
  }

  _onLiveMessage(msg) {
    if (msg.type === "frame") {
      if (this._planning) { this._planning = false; toast("Route ready — flying", "ok"); }
      this.player.append(msg.frame);
    }
    else if (msg.type === "log") { if (/^\[(plan|launch|abort|done)\]/.test(msg.line || "")) toast(msg.line); }
    else if (msg.type === "result") {
      this.app.store.mission = { id: "live", frames: this.player.frames, result: msg.result };
      toast(`Simulation complete — ${msg.result.outcome}`, "ok");
    } else if (msg.type === "error") toast(msg.message, "err");
  }

  // --- per-frame render -----------------------------------------------------
  _renderFrame(frame, index) {
    if (!frame) return;
    this.banner?.update(frame);
    this.pfd?.update(frame);

    // Live frames arrive in order, so append incrementally to the flown path.
    this.map.pushPath(frame.true.lat, frame.true.lon, frame.stage);
    this.viewer.pushFrame(frame);
    this._lastIdx = index;

    this.map.setMissile({ lat: frame.true.lat, lon: frame.true.lon, yaw: frame.att.yaw, label: this.missileLabel, stage: frame.stage });

    // charts
    this.charts.pos.push(frame.err.pos_m);
    this.charts.alt.push(frame.err.alt_m);
    this.charts.agl.push(frame.true.agl ?? 0);
    this.charts.spd.push(frame.vel.ground_speed);

    // HUDs
    this.map.setInfo(`ALT <b>${nf(frame.true.alt)}</b> m · SPD <b>${nf(frame.vel.ground_speed)}</b> m/s`);
    this.viewHud.innerHTML = `HDG <b>${nf(frame.att.yaw)}</b>° · FPA <b>${nf(frame.att.fpa, 1)}</b>°<br>AGL <b>${frame.true.agl != null ? nf(frame.true.agl) : "—"}</b> m`;

    // Skip the rebuild while the operator is reading / expanding the monitor
    // (see _buildMonitor); it refreshes to the latest frame on pointer-leave.
    if (!this._monHover) this._scheduleMonitor(frame);
  }

  // Coalesce the monitor rebuild to at most one per animation frame. Live frames
  // arrive ~10×/s (more at higher view speeds) and each rebuild tears down and
  // recreates the whole panel; doing that synchronously on every WebSocket
  // message contends with the 3D viewer's WebGL renders and visibly freezes the
  // telemetry while the operator orbits the view. Rendering only the latest
  // frame on rAF keeps the numbers live and lets the browser interleave work.
  _scheduleMonitor(frame) {
    this._monFrame = frame;
    if (this._monRaf != null) return;
    this._monRaf = requestAnimationFrame(() => {
      this._monRaf = null;
      if (!this._monHover && this._monFrame) this._renderMonitor(this._monFrame);
    });
  }

  // --- monitor tabs ---------------------------------------------------------
  _renderMonitor(frame = this.player?.current) {
    if (!this.monBody) return;
    // Live telemetry rebuilds this panel every frame. Capture native <details>
    // state before clearing it so operator-expanded diagnostics stay usable.
    for (const node of this.monBody.querySelectorAll(".mon-details[data-detail-key]")) {
      this.monDetailOpen.set(node.dataset.detailKey, node.open);
    }
    clear(this.monBody);
    if (!frame) { this.monBody.append(state("monitoring", "No telemetry", this.app.store.plan ? "Press Launch to fly the armed route." : "Plan a route, then launch.")); return; }
    if (this.monTab === "Navigation") this._monNav(frame);
    else if (this.monTab === "Controls") this._monControls(frame);
    else this._monWeather(frame);
  }

  _monNav(f) {
    const t = f.tercom;   // live-only TERCOM detail block
    const n = f.nav;      // live-only nav timing block
    const k = f.kalman;   // live-only Kalman filter snapshot
    this.monBody.append(
      sectionTitle("Fused estimate · EKF"),
      kvs([
        ["Est latitude", fmtLat(f.est.lat)], ["Est longitude", fmtLon(f.est.lon)], ["Est altitude", `${nf(f.est.alt)} m`],
      ]),
      sectionTitle("Estimate vs truth"),
      kvs([
        ["Position error", `${nf(f.err.pos_m, 1)} m`, f.err.pos_m > 100 ? "danger" : f.err.pos_m > 50 ? "warn" : "ok"],
        ["Altitude error", `${nf(f.err.alt_m, 1)} m`, Math.abs(f.err.alt_m) > 30 ? "warn" : "ok"],
      ]),
      sectionTitle("Sensor fusion status"),
      kvs([
        ["GPS", f.flags.gps ? "VALID" : "NO FIX", f.flags.gps ? "ok" : "danger"],
        ["TERCOM", f.flags.tercom ? "MATCHING" : "STANDBY", f.flags.tercom ? "ok" : ""],
        ["INS", "DEAD-RECKON", ""],
        ["Distance flown", fmtDist(f.progress.traveled_m)],
        ["To target", f.progress.to_target_m != null ? fmtDist(f.progress.to_target_m) : "—"],
      ]),
    );

    // Live-only expandable detail: Kalman filter state + covariance.
    if (k) {
      const s = k.state, sg = k.sigma;
      this.monBody.append(details("Kalman filter · state & covariance", [
        subLabel("Fused estimate · ENU (m, m/s)"),
        ...kvRows([
          ["Position E · N · U", `${nf(s.east_m, 0)} · ${nf(s.north_m, 0)} · ${nf(s.up_m, 0)}`],
          ["Velocity E · N · U", `${nf(s.vel_east_ms, 1)} · ${nf(s.vel_north_ms, 1)} · ${nf(s.vel_up_ms, 1)}`],
        ]),
        subLabel("1σ uncertainty (√diag P)"),
        ...kvRows([
          ["Position σ  E · N · U", `${nf(sg.pos_e_m, 2)} · ${nf(sg.pos_n_m, 2)} · ${nf(sg.pos_u_m, 2)} m`],
          ["Velocity σ  E · N · U", `${nf(sg.vel_e_ms, 3)} · ${nf(sg.vel_n_ms, 3)} · ${nf(sg.vel_u_ms, 3)} m/s`],
          ["Horizontal position σ", `${nf(k.pos_sigma_h_m, 2)} m`, k.pos_sigma_h_m > 30 ? "warn" : "ok"],
          ["3-D position σ", `${nf(k.pos_sigma_3d_m, 2)} m`],
          ["3-D velocity σ", `${nf(k.vel_sigma_3d_ms, 3)} m/s`],
          ["Process noise σ", `${nf(k.process_noise_std, 3)}`],
        ]),
      ], true, this.monDetailOpen));
    }

    // Live-only expandable detail: TERCOM contour-matching internals.
    if (t) {
      const matched = (t.matched_lat != null && t.matched_lon != null)
        ? `${fmtLat(t.matched_lat)} ${fmtLon(t.matched_lon)}` : "—";
      this.monBody.append(details("TERCOM · terrain matching", [
        bar("Correlation (NCC)", t.correlation ?? 0, 0, 1, t.match ? "ok" : "", 3),
        ...kvRows([
          ["Status", t.active ? "FIX ACCEPTED" : (t.suitable ? "SEARCHING" : "TERRAIN TOO FLAT"), t.active ? "ok" : (t.suitable ? "" : "warn")],
          ["Match found", t.match ? "yes" : "no", t.match ? "ok" : ""],
          ["Terrain roughness", `${nf(t.roughness_m, 1)} m σ`, t.suitable ? "ok" : "warn"],
          ["Roughness threshold", `${nf(t.threshold_m, 1)} m σ`],
          ["Fixes accepted", `${nf(t.fixes)}`],
          ["Search window", t.search_size ? `${nf(t.search_size)} px` : "—"],
          ["Lateral accuracy", t.lateral_acc_m != null ? `±${nf(t.lateral_acc_m, 1)} m` : "—"],
          ["Update rate", t.period_s ? `${nf(1 / t.period_s, 1)} Hz` : "—"],
          ["Matched position", matched],
        ]),
      ], false, this.monDetailOpen));
    }
    if (n) {
      this.monBody.append(details("GPS · INS timing", kvRows([
        ["GPS update rate", `${nf(1 / n.gps_period_s, 0)} Hz`],
        ["GPS fixes accepted", `${nf(n.gps_fixes)}`],
        ["INS integration rate", `${nf(1 / n.ins_period_s, 0)} Hz`],
        ["INS distance integrated", n.ins_distance_m != null ? fmtDist(n.ins_distance_m) : "—"],
      ]), false, this.monDetailOpen));
    }
    if (!t && !n && !k) this.monBody.append(note("Live EKF / TERCOM / GPS internals stream from the simulation; recorded flights log fused state only."));
  }

  _monControls(f) {
    const c = f.ctrl;   // live-only control / PID block
    this.monBody.append(
      sectionTitle("Guidance state"),
      kvs([
        ["Flight stage", f.stage],
        ["Guidance mode", c ? (c.terminal ? "TERMINAL" : "CRUISE") : "—", c && c.terminal ? "warn" : ""],
        ["Terminal latched", c ? (c.terminal ? "YES" : "no") : "—", c && c.terminal ? "warn" : ""],
        ["Heading (yaw)", `${nf(f.att.yaw, 1)}°`],
        ["Flight-path angle", `${nf(f.att.fpa, 2)}°`],
        ["Pitch", `${nf(f.att.pitch, 2)}°`],
        ["Roll", `${nf(f.att.roll, 2)}°`],
      ]),
    );

    if (c) {
      this.monBody.append(
        sectionTitle("Command outputs"),
        el("div", {}, [
          bar("Throttle", c.throttle ?? 0, 0, 1, "", 2),
          bar("Turn accel", c.accel_turn ?? 0, -50, 50, "", 2),
          bar("Climb accel", c.accel_climb ?? 0, -50, 50, "", 2),
        ]),
        kvs([
          ["Target altitude", c.target_alt != null ? `${nf(c.target_alt)} m` : "—"],
          ["Target speed", c.target_spd != null ? `${nf(c.target_spd)} m/s` : "—"],
          ["Vertical-speed command", c.vs_cmd != null ? `${nf(c.vs_cmd, 1)} m/s` : "—"],
        ]),
      );
    }

    this.monBody.append(
      sectionTitle("Velocity vector (ENU)"),
      kvs([
        ["East", `${nf(f.vel.east, 1)} m/s`], ["North", `${nf(f.vel.north, 1)} m/s`],
        ["Vertical", `${nf(f.vel.up, 1)} m/s`, f.vel.up < -5 ? "warn" : ""], ["Ground speed", `${nf(f.vel.ground_speed, 1)} m/s`],
      ]),
    );

    if (c && c.alt_pid) this.monBody.append(pidDetails("Altitude PID · vertical-speed hold", c.alt_pid, "m/s²", this.monDetailOpen));
    if (c && c.spd_pid) this.monBody.append(pidDetails("Speed PID · throttle", c.spd_pid, "", this.monDetailOpen));
    if (!c) this.monBody.append(note("Autopilot / PID output is available on the live simulation stream; recorded telemetry logs kinematic state only."));
  }

  _monWeather(f) {
    const w = this.wind || (this.app.store.plan && this.app.store.plan.config);
    this.monBody.append(
      sectionTitle("Wind field"),
      w ? kvs([
        ["Reference speed", `${nf(w.wind_speed_ref_ms, 1)} m/s`],
        ["Wind from", w.wind_from_deg === "" || w.wind_from_deg == null ? "auto crosswind" : `${nf(w.wind_from_deg)}°`],
        ["Turbulence", "Dryden gusts"],
      ]) : note("Wind field not recorded for this flight."),
      sectionTitle("Local conditions"),
      kvs([
        ["Altitude MSL", `${nf(f.true.alt)} m`],
        ["Height AGL", f.true.agl != null ? `${nf(f.true.agl)} m` : "—"],
        ["Ground elev.", f.true.ground_alt != null ? `${nf(f.true.ground_alt)} m` : "—"],
      ]),
    );
  }
}

// --- helpers ----------------------------------------------------------------
function icon(name, title, onClick) { return el("button", { class: "instr__btn", title, "aria-label": title, onClick }, [el("span", { class: "mi", text: name })]); }
function leg(colorVar, label, dashed) { return el("span", {}, [el("i", { style: { color: `var(${colorVar})`, borderTopStyle: dashed ? "dashed" : "solid" } }), el("span", { text: label })]); }
function sectionTitle(t) { return el("div", { class: "label", style: { margin: "14px 0 8px" }, text: t }); }
function subLabel(t) { return el("div", { class: "mon-sublabel", text: t }); }
function note(t) { return el("p", { class: "hint", style: { marginTop: "10px", lineHeight: "1.5" }, text: t }); }
function state(icon, title, msg) { return el("div", { class: "state" }, [el("span", { class: "mi", text: icon }), el("span", { class: "state__title", text: title }), el("span", { class: "state__msg", text: msg })]); }
function kvs(rows) {
  return el("div", {}, kvRows(rows));
}
// Return an array of .kv rows (so they can be spread into a details/section body).
function kvRows(rows) {
  return rows.map(([k, v, cls]) =>
    el("div", { class: "kv" }, [el("span", { class: "kv__k", text: k }), el("span", { class: `kv__v ${cls ? "is-" + cls : ""}`, text: v })]));
}

// A collapsible <details> section with a monospace summary and arbitrary body nodes.
function details(summaryText, body, defaultOpen = false, openState = null) {
  const open = openState?.has(summaryText) ? openState.get(summaryText) : defaultOpen;
  const d = el("details", { class: "mon-details", dataset: { detailKey: summaryText } });
  if (open) d.setAttribute("open", "");
  if (openState) d.addEventListener("toggle", () => openState.set(summaryText, d.open));
  d.append(
    el("summary", { class: "mon-details__summary" }, [
      el("span", { class: "mi", text: "chevron_right" }),
      el("span", { text: summaryText }),
    ]),
    el("div", { class: "mon-details__body" }, [].concat(body)),
  );
  return d;
}

// A labelled horizontal meter for a value within [min,max] (handles signed ranges).
function bar(label, value, min, max, cls = "", digits = 2) {
  const span = (max - min) || 1;
  const frac = clamp01((value - min) / span);
  const zeroFrac = clamp01((0 - min) / span);
  const fillColor = cls === "ok" ? "--success" : cls === "warn" ? "--warn" : "--primary";
  // For signed ranges draw the fill from the zero baseline; otherwise from the left.
  const signed = min < 0 && max > 0;
  let left, width;
  if (signed) { left = Math.min(frac, zeroFrac); width = Math.abs(frac - zeroFrac); }
  else { left = 0; width = frac; }
  return el("div", { class: "mon-bar" }, [
    el("div", { class: "mon-bar__head" }, [
      el("span", { class: "mon-bar__label", text: label }),
      el("span", { class: `mon-bar__val mono ${cls ? "is-" + cls : ""}`, text: nf(value, digits) }),
    ]),
    el("div", { class: "mon-bar__track" }, [
      signed ? el("i", { class: "mon-bar__zero", style: { left: `${zeroFrac * 100}%` } }) : null,
      el("i", { class: "mon-bar__fill", style: { left: `${left * 100}%`, width: `${width * 100}%`, background: `var(${fillColor})` } }),
    ]),
  ]);
}
function clamp01(v) { return Math.max(0, Math.min(1, v)); }

// A PID controller detail block: gains, live P/I/D split with bars, and output.
function pidDetails(title, p, unit, openState = null) {
  const u = unit ? ` ${unit}` : "";
  const lo = p.out_min, hi = p.out_max;
  const body = [
    el("div", { class: "kv" }, [
      el("span", { class: "kv__k", text: "Gains Kp · Ki · Kd" }),
      el("span", { class: "kv__v mono", text: `${nf(p.kp, 3)} · ${nf(p.ki, 3)} · ${nf(p.kd, 3)}` }),
    ]),
    bar("P term", p.p, lo, hi, "", 3),
    bar("I term", p.i, lo, hi, "warn", 3),
    bar("D term", p.d, lo, hi, "", 3),
    bar("Output (clamped)", p.out, lo, hi, "ok", 3),
    ...kvRows([
      ["Error", `${nf(p.error, 3)}${u}`],
      ["Integrator", `${nf(p.integral, 3)}`],
      ["Output limits", `${nf(p.out_min, 2)} … ${nf(p.out_max, 2)}${u}`],
    ]),
  ];
  return details(title, body, false, openState);
}
