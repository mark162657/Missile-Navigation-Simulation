// Final Report: a scrubbable replay of the whole flight (map + 3D viewer +
// telemetry readout driven by a timeline you drag), and a mission report card
// (verdict, impact geometry, deviation stats, a success score, and JSON export).

import { el, $, clear, toast, nf, fmtDist, fmtTime, fmtLat, fmtLon, clamp } from "./util.js";
import { api } from "./api.js";
import { Workspace } from "./widgets.js";
import { MapPanel } from "./map_panel.js";
import { Viewer3D } from "./viewer3d.js";
import { Player } from "./player.js";

const SPEEDS = [0.5, 1, 2, 5, 10];
const STAGE_COLORS = { BOOST: "--c-boost", CRUISE: "--c-planned", TERMINAL: "--c-terminal", IMPACT: "--c-terminal", PRE_LAUNCHED: "--ink-3" };

export class ReportScreen {
  constructor(root, app) { this.root = root; this.app = app; this.built = false; this.player = null; this.data = null; this._lastIdx = -1; }

  async activate() {
    if (!this.built) this._build();
    if (!this.app.store.missions) { try { this.app.store.missions = await api.missions(); } catch { this.app.store.missions = []; } }
    this._refreshActions();
    if (!this.data) {
      if (this.app.store.mission) this._loadData(this.app.store.mission);
      else if (this.app.store.missions?.length) this._loadReplay(this.app.store.missions[0].id);
    }
    this.map?.invalidate(); this.viewer?.invalidate();
  }
  deactivate() { this.player?.pause(); }

  _build() {
    this.ws = new Workspace(this.root);
    this._buildMap();
    this._buildViewer();
    this._buildReadout();
    this._buildTimeline();
    this._buildReport();
    this.built = true;
  }

  _refreshActions() {
    const missions = this.app.store.missions || [];
    const src = el("select", { class: "select", style: { width: "230px" } });
    if (this.app.store.mission) src.append(el("option", { value: "current", text: "◆ Current mission" }));
    missions.forEach((m) => src.append(el("option", { value: m.id, text: `${short(m.id)} · ${m.final_stage}` })));
    src.value = this.data?.id === this.app.store.mission?.id && this.app.store.mission ? "current" : (this.data?.id || (missions[0]?.id ?? ""));
    src.addEventListener("change", () => {
      if (src.value === "current") this._loadData(this.app.store.mission);
      else this._loadReplay(src.value);
    });
    const exportBtn = el("button", { class: "btn btn--sm btn--ghost", onClick: () => this._export() }, [el("span", { class: "mi", text: "download" }), el("span", { text: "Export" })]);
    this.app.setActions([this.app.hiddenWidgetsControl(this.ws), src, exportBtn]);
  }

  // --- widgets --------------------------------------------------------------
  _buildMap() {
    this.ws.add({ id: "r-map", title: "Ground Track", cols: 5, rows: 6, instrument: true, flush: true,
      build: (body) => { this.map = new MapPanel(body, { legend: "mission" }); } });
  }
  _buildViewer() {
    this.ws.add({ id: "r-viewer", title: "3D Replay · Posture", cols: 4, rows: 6, instrument: true, flush: true,
      build: (body) => { const c = el("canvas"); body.append(el("div", { class: "instr" }, [c, el("div", { class: "instr__hud instr__hud--tl", html: "<b>Drag</b> orbit · <b>scroll</b> zoom" }), el("div", { class: "instr__controls" }, [icon("center_focus_strong", "Recenter on missile", () => { this.viewer.follow = true; this.viewer.resetView(); }), icon("zoom_out_map", "Frame whole mission", () => this.viewer.frameCamera())])])); this.viewer = new Viewer3D(c); this.viewer.label = "TLAM"; } });
  }
  _buildReadout() {
    this.ws.add({ id: "r-readout", title: "State @ Playhead", cols: 3, rows: 6, flush: true,
      build: (body) => { this.readout = el("div", { style: { padding: "12px 14px", overflow: "auto", height: "100%" } }); body.append(this.readout); } });
  }
  _buildTimeline() {
    this.ws.add({ id: "r-timeline", title: "Timeline", cols: 12, rows: 2, flush: true,
      build: (body) => {
        this.playBtn = el("button", { class: "timeline__play", "aria-label": "Play", onClick: () => this._toggle() }, [el("span", { class: "mi", text: "play_arrow" })]);
        this.stagesEl = el("div", { class: "timeline__stages" });
        this.progEl = el("div", { class: "timeline__progress", style: { width: "0%" } });
        this.headEl = el("div", { class: "timeline__playhead", style: { left: "0%" } });
        this.track = el("div", { class: "timeline__track" }, [this.stagesEl, this.progEl, this.headEl]);
        this.timeEl = el("div", { class: "timeline__time", text: "00:00.0 / 00:00.0" });
        this.speedSeg = el("div", { class: "seg timeline__speeds" }, SPEEDS.map((s) => el("button", { class: "seg__btn", "aria-selected": String(s === 1), text: `${s}×`, onClick: (e) => { this.player?.setSpeed(s); Array.from(this.speedSeg.children).forEach((b) => b.setAttribute("aria-selected", String(b === e.currentTarget))); } })));
        body.append(el("div", { class: "timeline" }, [el("div", { class: "timeline__row" }, [this.playBtn, el("div", { class: "timeline__scrub-wrap" }, [this.track]), this.timeEl, this.speedSeg])]));
        this._wireScrub();
      } });
  }
  _buildReport() {
    this.ws.add({ id: "r-report", title: "Mission Report", cols: 12, rows: 8, flush: true,
      build: (body) => { this.reportBody = el("div", { style: { padding: "18px", overflow: "auto", height: "100%" } }); body.append(this.reportBody); this._renderReport(); } });
  }

  _wireScrub() {
    let dragging = false;
    const seek = (e) => { const r = this.track.getBoundingClientRect(); const f = clamp((e.clientX - r.left) / r.width, 0, 1); this.player?.seekFraction(f); };
    this.track.addEventListener("pointerdown", (e) => { dragging = true; this.track.setPointerCapture(e.pointerId); this.player?.pause(); this._syncPlay(); seek(e); });
    this.track.addEventListener("pointermove", (e) => dragging && seek(e));
    this.track.addEventListener("pointerup", (e) => { dragging = false; this.track.releasePointerCapture?.(e.pointerId); });
  }

  // --- data -----------------------------------------------------------------
  async _loadReplay(id) {
    try { const data = await api.mission(id); this._loadData(data); toast(`Loaded ${short(id)}`, "ok"); }
    catch (e) { toast(`Load failed: ${e.message}`, "err"); }
  }

  async _loadData(data) {
    if (!data?.frames?.length) { toast("No telemetry to report", "err"); return; }
    this.data = data;
    this.player?.destroy();
    this.stats = computeStats(data.frames, data.result);

    // scene
    const s0 = data.frames[0].true;
    const startGps = [s0.lat, s0.lon, s0.alt];
    const demName = data.dem_name || this._demForPoint(s0.lat, s0.lon);
    let grid = data.dem_grid;
    if (!grid && demName) { try { grid = await api.demGrid(demName); } catch { /* ignore */ } }
    if (grid) { this.map.setDEM(grid); this.viewer.setDEM(grid); }
    this.viewer.setStart(startGps); this.map.setStart(startGps);
    const tgt = data.result?.target_gps || data.trajectory?.[data.trajectory.length - 1];
    if (tgt) { this.viewer.setTarget(tgt); this.map.setTarget(tgt); }
    this.map.setPlan(data.trajectory || null); this.viewer.setPlan(data.trajectory || null);
    this.viewer.follow = true; this.viewer.resetView();
    this._fitMap();

    // stage segments on timeline
    this._renderStageSegments(data.frames);

    this.player = new Player(data.frames, "replay");
    this._lastIdx = -1;
    this.player.onFrame((f, i) => this._renderFrame(f, i));
    this.player.onEnd(() => this._syncPlay());
    this._renderFrame(data.frames[0], 0);
    this._renderReport();
    this._syncPlay();
  }

  _demForPoint(lat, lon) {
    const d = (this.app.store.dems || []).find((d) => d.bounds && lat >= d.bounds.south && lat <= d.bounds.north && lon >= d.bounds.west && lon <= d.bounds.east);
    return d?.name;
  }

  _fitMap() {
    const pts = this.data?.frames?.map((f) => [f.true.lat, f.true.lon]);
    if (pts?.length) return this.map.fitPoints(pts);
    if (this.map.grid) this.map.fit(this.map.grid.bounds);
  }

  _renderStageSegments(frames) {
    clear(this.stagesEl);
    const total = frames[frames.length - 1].t || 1;
    let start = 0, cur = frames[0].stage;
    for (let i = 1; i <= frames.length; i++) {
      const st = frames[i]?.stage;
      if (st !== cur || i === frames.length) {
        const from = frames[start].t, to = frames[Math.min(i, frames.length - 1)].t;
        this.stagesEl.append(el("div", { class: "timeline__stage-seg", style: { width: `${((to - from) / total) * 100}%`, background: `var(${STAGE_COLORS[cur] || "--ink-3"})` } }));
        start = i; cur = st;
      }
    }
  }

  // --- playback -------------------------------------------------------------
  _toggle() { this.player?.toggle(); this._syncPlay(); }
  _syncPlay() { const p = this.player?.playing; if (this.playBtn) this.playBtn.querySelector(".mi").textContent = p ? "pause" : "play_arrow"; }

  _renderFrame(frame, index) {
    if (!frame) return;
    if (index === this._lastIdx + 1) { this.map.pushPath(frame.true.lat, frame.true.lon, frame.stage); this.viewer.pushFrame(frame); }
    else { const upto = this.player.frames.slice(0, index + 1); this.map.setPathPoints(upto.map((f) => ({ lat: f.true.lat, lon: f.true.lon, stage: f.stage }))); this.viewer.setPathFrames(upto); }
    this._lastIdx = index;
    this.map.setMissile({ lat: frame.true.lat, lon: frame.true.lon, yaw: frame.att.yaw, label: "TLAM", stage: frame.stage });

    const dur = this.player.duration || 1;
    this.progEl.style.width = `${(frame.t / dur) * 100}%`;
    this.headEl.style.left = `${(frame.t / dur) * 100}%`;
    this.timeEl.textContent = `${fmtTime(frame.t)} / ${fmtTime(dur)}`;
    this._renderReadout(frame);
    this._syncPlay();
  }

  _renderReadout(f) {
    clear(this.readout);
    this.readout.append(
      el("div", { class: "chip", dataset: { stage: f.stage }, style: { marginBottom: "10px", color: `var(${STAGE_COLORS[f.stage] || "--ink"})` } }, [el("span", { text: f.stage })]),
      title("Position"),
      kvs([["Latitude", fmtLat(f.true.lat)], ["Longitude", fmtLon(f.true.lon)], ["Altitude MSL", `${nf(f.true.alt)} m`], ["Height AGL", f.true.agl != null ? `${nf(f.true.agl)} m` : "—"]]),
      title("Motion"),
      kvs([["Ground speed", `${nf(f.vel.ground_speed)} m/s`], ["Vertical speed", `${nf(f.vel.up, 1)} m/s`], ["Heading", `${nf(f.att.yaw, 1)}°`], ["Flight-path", `${nf(f.att.fpa, 2)}°`]]),
      title("Navigation error"),
      kvs([["Position error", `${nf(f.err.pos_m, 1)} m`, f.err.pos_m > 100 ? "danger" : f.err.pos_m > 50 ? "warn" : "ok"], ["Altitude error", `${nf(f.err.alt_m, 1)} m`], ["To target", f.progress.to_target_m != null ? fmtDist(f.progress.to_target_m) : "—"]]),
    );
  }

  // --- report card ----------------------------------------------------------
  _renderReport() {
    clear(this.reportBody);
    if (!this.data) { this.reportBody.append(state("summarize", "No mission loaded", "Select a mission to generate its report.")); return; }
    const r = this.data.result || {};
    const outcome = (r.outcome || "TIMEOUT").toUpperCase();
    const s = this.stats;
    const score = successScore(r, s);

    const rep = el("div", { class: "report" });
    // verdict
    rep.append(el("div", { class: "verdict", dataset: { verdict: outcome } }, [
      el("div", { class: "verdict__badge" }, [el("span", { class: "mi", text: verdictIcon(outcome) })]),
      el("div", {}, [el("div", { class: "verdict__title", text: verdictTitle(outcome) }), el("div", { class: "verdict__sub", text: verdictSub(outcome, r) })]),
      el("div", { class: "verdict__score" }, [scoreRing(score), el("div", { class: "label", style: { marginTop: "6px" }, text: "Mission score" })]),
    ]));

    // metric tiles
    const tiles = [
      tile("Miss distance", r.miss_distance_m != null ? fmtDist(r.miss_distance_m, 1) : "—", r.miss_distance_m != null && r.blast_radius_m ? (r.miss_distance_m <= r.blast_radius_m ? "within blast" : "outside blast") : ""),
      tile("Impact angle", r.impact_angle_deg != null ? `${nf(r.impact_angle_deg, 1)}°` : "—", "flight-path at impact"),
      tile("Impact speed", r.impact_speed_ms != null ? `${nf(r.impact_speed_ms)}` : "—", "m/s"),
      tile("Warhead", r.detonated == null ? "—" : (r.detonated ? "Detonated" : "Dud"), r.warhead_name || ""),
      tile("Flight time", r.flight_time_s != null ? fmtTime(r.flight_time_s) : fmtTime(s.duration), "mm:ss"),
      tile("Distance flown", fmtDist(r.distance_flown_m ?? s.distanceFlown), ""),
      tile("Peak position error", `${nf(s.maxPosError, 1)}`, "m · est vs truth"),
      tile("Peak altitude", `${nf(s.maxAlt)}`, "m MSL"),
    ];
    rep.append(el("div", { class: "report__grid" }, tiles));

    // analysis
    rep.append(el("div", { class: "verdict", style: { display: "block", padding: "18px 22px" } }, [
      el("div", { class: "label", style: { marginBottom: "10px" }, text: "Assessment" }),
      el("p", { style: { color: "var(--ink-2)", lineHeight: "1.6", maxWidth: "75ch" }, text: assessment(outcome, r, s) }),
      el("div", { style: { display: "flex", gap: "10px", marginTop: "16px", flexWrap: "wrap" } }, [
        el("button", { class: "btn btn--primary btn--sm", onClick: () => this._export() }, [el("span", { class: "mi", text: "download" }), el("span", { text: "Export report (JSON)" })]),
        el("button", { class: "btn btn--sm", onClick: () => this._copySummary() }, [el("span", { class: "mi", text: "content_copy" }), el("span", { text: "Copy summary" })]),
      ]),
    ]));

    this.reportBody.append(rep);
  }

  _export() {
    if (!this.data) return;
    const payload = {
      generated_utc: new Date().toISOString(),
      mission_id: this.data.id,
      outcome: this.data.result?.outcome,
      result: this.data.result,
      statistics: this.stats,
      success_score: successScore(this.data.result || {}, this.stats),
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
    const a = el("a", { href: URL.createObjectURL(blob), download: `report_${this.data.id}.json` });
    document.body.append(a); a.click(); a.remove();
    toast("Report exported", "ok");
  }
  async _copySummary() {
    const r = this.data.result || {};
    const txt = `Mission ${this.data.id}\nOutcome: ${r.outcome}\nMiss: ${r.miss_distance_m != null ? r.miss_distance_m.toFixed(1) + " m" : "—"}\nImpact angle: ${r.impact_angle_deg != null ? r.impact_angle_deg.toFixed(1) + "°" : "—"}\nScore: ${successScore(r, this.stats)}/100`;
    try { await navigator.clipboard.writeText(txt); toast("Summary copied", "ok"); } catch { toast("Copy failed", "err"); }
  }
}

// --- stats / scoring --------------------------------------------------------
function computeStats(frames, result) {
  let maxPosError = 0, maxAlt = 0, maxSpeed = 0, maxAgl = 0;
  for (const f of frames) {
    maxPosError = Math.max(maxPosError, f.err.pos_m);
    maxAlt = Math.max(maxAlt, f.true.alt);
    maxSpeed = Math.max(maxSpeed, f.vel.ground_speed);
    if (f.true.agl != null) maxAgl = Math.max(maxAgl, f.true.agl);
  }
  const last = frames[frames.length - 1];
  return {
    duration: last.t, samples: frames.length,
    maxPosError, maxAlt, maxSpeed, maxAgl,
    distanceFlown: last.progress.traveled_m,
    finalPosError: last.err.pos_m,
  };
}

function successScore(r, s) {
  const o = (r.outcome || "").toUpperCase();
  let score = { HIT: 92, MISS: 52, CFIT: 24, TIMEOUT: 16, ABORTED: 8 }[o] ?? 20;
  if (r.miss_distance_m != null) {
    const blast = r.blast_radius_m || 40;
    if (o === "HIT") score += clamp((blast - r.miss_distance_m) / blast, 0, 1) * 8;
    else if (o === "MISS") score -= clamp((r.miss_distance_m - blast) / (blast * 12), 0, 1) * 22;
  }
  if (s?.maxPosError != null) score -= clamp((s.maxPosError - 60) / 600, 0, 1) * 12; // navigation drift penalty
  return Math.round(clamp(score, 0, 100));
}

function assessment(o, r, s) {
  const parts = [];
  if (o === "HIT") parts.push(`Target neutralised. The missile impacted ${r.miss_distance_m != null ? r.miss_distance_m.toFixed(1) + " m" : "within lethal radius"} from the aim point, inside the ${r.blast_radius_m ?? 40} m blast radius.`);
  else if (o === "MISS") parts.push(`Warhead reached the terminal phase but missed the lethal radius${r.miss_distance_m != null ? ` by ${(r.miss_distance_m - (r.blast_radius_m || 40)).toFixed(0)} m` : ""}.`);
  else if (o === "CFIT") parts.push("Controlled flight into terrain — the missile struck ground short of the target. Review the cruise-altitude band and pathfinder clearance over rough terrain.");
  else if (o === "TIMEOUT") parts.push("The flight-time guard fired before impact; this run was likely capped for a short test rather than flown to the target.");
  else parts.push("The run terminated before a target verdict was reached.");
  parts.push(`Peak navigation error was ${s.maxPosError.toFixed(1)} m (estimate vs. truth), peaking speed ${s.maxSpeed.toFixed(0)} m/s and altitude ${s.maxAlt.toFixed(0)} m MSL over ${fmtTime(s.duration)} of flight.`);
  return parts.join(" ");
}

function verdictIcon(o) { return { HIT: "gps_fixed", MISS: "adjust", CFIT: "warning", TIMEOUT: "timer_off", ABORTED: "block" }[o] || "help"; }
function verdictTitle(o) { return { HIT: "TARGET HIT", MISS: "MISS", CFIT: "CFIT", TIMEOUT: "TIMED OUT", ABORTED: "ABORTED" }[o] || o; }
function verdictSub(o, r) {
  if (o === "HIT") return "Warhead detonated within lethal radius";
  if (o === "MISS") return "Impact outside lethal radius";
  if (o === "CFIT") return "Impacted terrain before target";
  if (o === "TIMEOUT") return "Flight-time guard reached before impact";
  return "Run terminated early";
}

function scoreRing(score) {
  const c = score >= 75 ? "--success" : score >= 45 ? "--warn" : "--danger";
  const R = 42, C = 2 * Math.PI * R;
  const off = C * (1 - score / 100);
  const svg = `<svg width="96" height="96" viewBox="0 0 96 96">
    <circle cx="48" cy="48" r="${R}" fill="none" stroke="var(--line-2)" stroke-width="7"/>
    <circle cx="48" cy="48" r="${R}" fill="none" stroke="var(${c})" stroke-width="7" stroke-linecap="round"
      stroke-dasharray="${C}" stroke-dashoffset="${off}" transform="rotate(-90 48 48)"/>
  </svg>`;
  return el("div", { class: "verdict__score-ring" }, [el("div", { html: svg }), el("div", { class: "verdict__score-num", style: { color: `var(${c})` }, text: String(score) })]);
}

// --- small builders ---------------------------------------------------------
function short(id) { return id.replace(/^flight_/, "").replace(/\+00-00$/, "").replace("T", " "); }
function icon(name, title, onClick) { return el("button", { class: "instr__btn", title, "aria-label": title, onClick }, [el("span", { class: "mi", text: name })]); }
function leg(colorVar, label) { return el("span", {}, [el("i", { style: { color: `var(${colorVar})` } }), el("span", { text: label })]); }
function title(t) { return el("div", { class: "label", style: { margin: "12px 0 6px" }, text: t }); }
function state(ic, t, m) { return el("div", { class: "state" }, [el("span", { class: "mi", text: ic }), el("span", { class: "state__title", text: t }), el("span", { class: "state__msg", text: m })]); }
function kvs(rows) { return el("div", {}, rows.map(([k, v, cls]) => el("div", { class: "kv" }, [el("span", { class: "kv__k", text: k }), el("span", { class: `kv__v ${cls ? "is-" + cls : ""}`, text: v })]))); }
function tile(k, v, u) { return el("div", { class: "metric-tile" }, [el("div", { class: "metric-tile__k", text: k }), el("div", { class: "metric-tile__v" }, [document.createTextNode(v), u ? el("span", { class: "metric-tile__u", text: u }) : ""])]); }
