// Minimal canvas line charts for the error/deviation monitors. Autoscaling,
// optional threshold band, tabular readout. Cheap redraws on demand.

import { el, fitCanvas, cssVar, clamp, nf } from "./util.js";

export class MiniChart {
  constructor(mount, { title, unit = "", color = "--primary", threshold = null, digits = 1 } = {}) {
    this.title = title; this.unit = unit; this.colorVar = color; this.threshold = threshold; this.digits = digits;
    this.data = [];
    this.valEl = el("span", { class: "chart__val", text: "—" });
    this.canvas = el("canvas");
    this.node = el("div", { class: "chart" }, [
      el("div", { class: "chart__head" }, [el("span", { class: "chart__title", text: title }), this.valEl]),
      this.canvas,
    ]);
    mount.appendChild(this.node);
    window.addEventListener("themechange", () => this.draw());
    this._ro = new ResizeObserver(() => this.draw());
    this._ro.observe(this.canvas);
  }

  push(v) { this.data.push(v); if (this.data.length > 600) this.data.shift(); this._update(v); this.draw(); }
  setData(arr) { this.data = arr.slice(-600); this._update(this.data[this.data.length - 1]); this.draw(); }
  reset() { this.data = []; this.valEl.textContent = "—"; this.draw(); }

  _update(v) {
    if (v == null) return;
    const over = this.threshold != null && Math.abs(v) > this.threshold;
    this.valEl.textContent = `${nf(v, this.digits)} ${this.unit}`.trim();
    this.valEl.style.color = over ? cssVar("--warn") : cssVar("--ink");
  }

  draw() {
    const [W, H, , ctx] = fitCanvas(this.canvas);
    ctx.clearRect(0, 0, W, H);
    if (this.data.length < 2) return;
    const color = cssVar(this.colorVar) || "#5b93ff";
    let min = Math.min(...this.data), max = Math.max(...this.data);
    if (this.threshold != null) { min = Math.min(min, -this.threshold); max = Math.max(max, this.threshold); }
    if (max - min < 1e-6) { max += 1; min -= 1; }
    const pad = (max - min) * 0.12; min -= pad; max += pad;
    const x = (i) => (i / (this.data.length - 1)) * W;
    const y = (v) => H - ((v - min) / (max - min)) * H;

    if (this.threshold != null) {
      ctx.fillStyle = cssVar("--warn-soft") || "rgba(255,207,51,.12)";
      const yt = y(this.threshold), yb = y(-this.threshold);
      ctx.fillRect(0, 0, W, Math.max(0, yt));
      ctx.fillRect(0, Math.min(H, yb), W, H - Math.min(H, yb));
    }

    // area fill
    ctx.beginPath(); ctx.moveTo(0, H);
    this.data.forEach((v, i) => ctx.lineTo(x(i), y(v)));
    ctx.lineTo(W, H); ctx.closePath();
    const grad = ctx.createLinearGradient(0, 0, 0, H);
    grad.addColorStop(0, hexA(color, 0.22)); grad.addColorStop(1, hexA(color, 0));
    ctx.fillStyle = grad; ctx.fill();

    ctx.beginPath();
    this.data.forEach((v, i) => (i ? ctx.lineTo(x(i), y(v)) : ctx.moveTo(x(i), y(v))));
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.lineJoin = "round"; ctx.stroke();

    // last point
    const lv = this.data[this.data.length - 1];
    ctx.beginPath(); ctx.arc(W - 1, y(lv), 2.4, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
  }
}

function hexA(color, a) {
  // color may be hex (#rrggbb) or a css value; fall back to rgba on hex only.
  if (color.startsWith("#") && color.length >= 7) {
    const r = parseInt(color.slice(1, 3), 16), g = parseInt(color.slice(3, 5), 16), b = parseInt(color.slice(5, 7), 16);
    return `rgba(${r},${g},${b},${a})`;
  }
  return color;
}
