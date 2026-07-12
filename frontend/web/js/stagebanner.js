// Prominent flight-stage banner: current stage, a stage progress rail, and the
// headline mission metrics (mission time + distance-to-target progress).

import { el, fmtTime, fmtDist, nf } from "./util.js";

const STAGES = ["PRE_LAUNCHED", "BOOST", "CRUISE", "TERMINAL", "IMPACT"];
const STAGE_LABEL = { PRE_LAUNCHED: "Pre-launch", BOOST: "Boost", CRUISE: "Cruise", TERMINAL: "Terminal", IMPACT: "Impact" };
const STAGE_SUB = {
  PRE_LAUNCHED: "Awaiting ignition",
  BOOST: "Solid booster · open-loop pitch program",
  CRUISE: "Midcourse · path-following guidance",
  TERMINAL: "Terminal guidance · impact-angle control",
  IMPACT: "Flight terminated",
};

export class StageBanner {
  constructor(mount) {
    this.nameEl = el("span", { class: "stage-banner__name", text: "—" });
    this.subEl = el("span", { class: "stage-banner__sub", text: "" });
    this.dotEl = el("span", { class: "stage-banner__dot" });

    this.pips = STAGES.filter((s) => s !== "PRE_LAUNCHED").map((s) =>
      el("div", { class: "stage-pip", dataset: { stage: s } }, [
        el("div", { class: "stage-pip__bar" }), el("div", { class: "stage-pip__label", text: STAGE_LABEL[s] }),
      ]));

    this.timeEl = el("div", { class: "stage-metric__v", text: "00:00.0" });
    this.distEl = el("div", { class: "stage-metric__v", text: "—" });
    this.progressEl = el("div", { class: "stage-banner__progress", style: { width: "0%" } });

    this.node = el("div", { class: "stage-banner", dataset: { stage: "PRE_LAUNCHED" } }, [
      el("div", { class: "stage-banner__stage" }, [this.dotEl,
        el("div", {}, [this.nameEl, el("div", {}, [this.subEl])])]),
      el("div", { class: "stage-banner__timeline" }, this.pips),
      el("div", { class: "stage-banner__metrics" }, [
        metric(this.timeEl, "Mission time"),
        metric(this.distEl, "To target"),
      ]),
      this.progressEl,
    ]);
    mount.appendChild(this.node);
    this._initialDist = null;
  }

  update(frame) {
    const stage = frame.stage;
    this.node.dataset.stage = stage;
    this.nameEl.textContent = (STAGE_LABEL[stage] || stage).toUpperCase();
    this.subEl.textContent = STAGE_SUB[stage] || "";

    const idx = STAGES.indexOf(stage);
    this.pips.forEach((p) => {
      const pi = STAGES.indexOf(p.dataset.stage);
      p.classList.toggle("is-active", pi === idx);
      p.classList.toggle("is-past", pi < idx);
    });

    this.timeEl.textContent = fmtTime(frame.t);
    const d = frame.progress.to_target_m;
    this.distEl.textContent = d != null ? fmtDist(d) : "—";
    if (d != null) {
      if (this._initialDist == null || d > this._initialDist) this._initialDist = d;
      const prog = this._initialDist ? Math.max(0, Math.min(1, 1 - d / this._initialDist)) : 0;
      this.progressEl.style.width = `${(prog * 100).toFixed(1)}%`;
    }
  }

  reset() { this._initialDist = null; this.progressEl.style.width = "0%"; }
}

function metric(valEl, label) {
  return el("div", { class: "stage-metric" }, [valEl, el("div", { class: "stage-metric__k", text: label })]);
}
