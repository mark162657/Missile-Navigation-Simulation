// A compact Primary Flight Display in the spirit of a 787 PFD: speed tape (left),
// altitude tape (right, baro/AGL switchable), and a centre attitude panel with a
// pitch ladder, heading, and flight-path-angle marker. Roll is shown even though
// the 3-DoF model keeps it ~0, so the instrument is ready when attitude arrives.

import { el, fitCanvas, cssVar, nf } from "./util.js";

export class PFD {
  constructor(mount) {
    this.altMode = "baro"; // 'baro' (MSL) | 'agl'
    this.frame = null;

    this.spdCanvas = el("canvas");
    this.centerCanvas = el("canvas");
    this.altCanvas = el("canvas");
    this.readout = el("div", { class: "pfd__readout mono" }, [el("b", { text: "—" })]);

    this.altModeBtn = el("button", { class: "chip pfd__alt-mode", title: "Switch altitude reference" }, [el("span", { text: "MSL" })]);
    this.altModeBtn.addEventListener("click", () => {
      this.altMode = this.altMode === "baro" ? "agl" : "baro";
      this.altModeBtn.firstChild.textContent = this.altMode === "baro" ? "MSL" : "AGL";
      this.draw();
    });

    this.node = el("div", { class: "pfd" }, [
      el("div", { class: "pfd__tape" }, [el("div", { class: "pfd__tape-label", text: "SPD kt" }), this.spdCanvas]),
      el("div", { class: "pfd__center" }, [this.readout, this.centerCanvas]),
      el("div", { class: "pfd__tape" }, [el("div", { class: "pfd__tape-label", text: this.altMode === "baro" ? "ALT m" : "AGL m" }), this.altCanvas, this.altModeBtn]),
    ]);
    mount.appendChild(this.node);
    window.addEventListener("themechange", () => this.draw());
    // Repaint the tapes + attitude at the new size on any resize (fullscreen etc.).
    this._ro = new ResizeObserver(() => this.draw());
    this._ro.observe(this.node);
  }

  update(frame) { this.frame = frame; this.draw(); }

  draw() {
    if (!this.frame) return;
    this._tape(this.spdCanvas, this.frame.vel.ground_speed * 1.94384, 40, "spd"); // m/s -> kt
    const alt = this.altMode === "baro" ? this.frame.true.alt : (this.frame.true.agl ?? 0);
    this._tape(this.altCanvas, alt, 200, "alt");
    this._attitude();
  }

  _tape(canvas, value, spacing, kind) {
    const [W, H, , ctx] = fitCanvas(canvas, { maxPixels: 220_000 });
    const ink = cssVar("--instr-ink"), dim = cssVar("--instr-ink-dim");
    ctx.clearRect(0, 0, W, H);
    const cy = H / 2;
    const pxPerUnit = 34 / spacing; // px per value unit
    const range = H / pxPerUnit;
    const top = value + range / 2, bot = value - range / 2;
    const first = Math.ceil(bot / spacing) * spacing;

    ctx.font = "10px " + (cssVar("--font-mono") || "monospace");
    ctx.textBaseline = "middle";
    for (let v = first; v <= top; v += spacing) {
      const y = cy + (value - v) * pxPerUnit;
      ctx.strokeStyle = "rgba(140,160,200,.25)"; ctx.lineWidth = 1;
      const tickX = kind === "spd" ? W : 0;
      ctx.beginPath(); ctx.moveTo(tickX, y); ctx.lineTo(kind === "spd" ? W - 8 : 8, y); ctx.stroke();
      ctx.fillStyle = dim;
      ctx.textAlign = kind === "spd" ? "right" : "left";
      ctx.fillText(nf(v, 0), kind === "spd" ? W - 11 : 11, y);
    }
    // current value bug
    ctx.fillStyle = cssVar("--primary");
    ctx.globalAlpha = 0.16; ctx.fillRect(0, cy - 12, W, 24); ctx.globalAlpha = 1;
    ctx.strokeStyle = cssVar("--primary"); ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.moveTo(0, cy - 12); ctx.lineTo(W, cy - 12); ctx.moveTo(0, cy + 12); ctx.lineTo(W, cy + 12); ctx.stroke();
    ctx.fillStyle = ink; ctx.font = "700 13px " + (cssVar("--font-mono") || "monospace");
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(nf(value, 0), W / 2, cy);
  }

  _attitude() {
    const [W, H, , ctx] = fitCanvas(this.centerCanvas, { maxPixels: 350_000 });
    const f = this.frame;
    const pitch = f.att.pitch || 0;   // deg (mostly ~0 in 3-DoF)
    const roll = (f.att.roll || 0) * Math.PI / 180;
    const fpa = f.att.fpa || 0;
    const ink = cssVar("--instr-ink"), dim = cssVar("--instr-ink-dim");
    const sky = cssVar("--primary"), ground = cssVar("--c-actual");

    ctx.clearRect(0, 0, W, H);
    const cx = W / 2, cy = H / 2;
    const pxPerDeg = 4;

    ctx.save(); ctx.translate(cx, cy); ctx.rotate(-roll);
    // Artificial-horizon fill. Sky above the horizon, ground below — each band is
    // drawn far larger than the viewport so it ALWAYS fills the panel, even in a
    // steep dive (pitch << 0) or during boost (pitch ~ +90). The opacity is high
    // enough that an all-ground / all-sky panel reads as brown / blue, never black.
    const horizonY = pitch * pxPerDeg;
    const R = Math.hypot(W, H) + Math.abs(horizonY) + 40; // half-extent past the viewport
    ctx.globalAlpha = 0.22; ctx.fillStyle = sky; ctx.fillRect(-R, horizonY - 2 * R, 2 * R, 2 * R);
    ctx.globalAlpha = 0.32; ctx.fillStyle = ground; ctx.fillRect(-R, horizonY, 2 * R, 2 * R);
    ctx.globalAlpha = 1;
    // horizon line
    ctx.strokeStyle = ink; ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.moveTo(-W, horizonY); ctx.lineTo(W, horizonY); ctx.stroke();
    // pitch ladder (wide range so rungs stay in view at steep attitudes)
    ctx.strokeStyle = dim; ctx.fillStyle = dim; ctx.font = "8px " + (cssVar("--font-mono") || "monospace"); ctx.textAlign = "center"; ctx.textBaseline = "middle";
    for (let d = -80; d <= 80; d += 10) {
      if (d === 0) continue;
      const y = horizonY - d * pxPerDeg;
      if (y < -cy - 4 || y > cy + 4) continue; // skip rungs off the panel
      const w = d % 20 === 0 ? 18 : 11;
      ctx.beginPath(); ctx.moveTo(-w, y); ctx.lineTo(w, y); ctx.stroke();
      if (d % 20 === 0) { ctx.fillText(String(Math.abs(d)), -w - 8, y); ctx.fillText(String(Math.abs(d)), w + 8, y); }
    }
    ctx.restore();

    // Off-screen horizon cue: when the horizon leaves the panel, a chevron at the
    // edge points to it so a nose-down / nose-up attitude is unmistakable.
    const horizonScreenY = cy + horizonY * Math.cos(roll);
    if (horizonScreenY < 6 || horizonScreenY > H - 6) {
      const down = horizonScreenY > H - 6; // horizon below panel => pitched down
      const ey = down ? H - 7 : 7, dir = down ? -1 : 1;
      ctx.fillStyle = ink; ctx.globalAlpha = 0.8;
      ctx.beginPath(); ctx.moveTo(cx, ey - dir * 5); ctx.lineTo(cx - 5, ey + dir * 3); ctx.lineTo(cx + 5, ey + dir * 3); ctx.closePath(); ctx.fill();
      ctx.globalAlpha = 1;
    }

    // fixed aircraft reference
    ctx.strokeStyle = cssVar("--warn"); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx - 22, cy); ctx.lineTo(cx - 8, cy); ctx.moveTo(cx + 8, cy); ctx.lineTo(cx + 22, cy);
    ctx.moveTo(cx, cy - 3); ctx.lineTo(cx, cy + 3); ctx.stroke();

    // flight-path vector marker (clamped to the panel so it never disappears)
    const fy = Math.max(8, Math.min(H - 8, cy - fpa * pxPerDeg));
    ctx.strokeStyle = cssVar("--success"); ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.arc(cx, fy, 4, 0, Math.PI * 2);
    ctx.moveTo(cx - 8, fy); ctx.lineTo(cx - 4, fy); ctx.moveTo(cx + 4, fy); ctx.lineTo(cx + 8, fy); ctx.moveTo(cx, fy - 4); ctx.lineTo(cx, fy - 8);
    ctx.stroke();

    // heading readout at top
    this.readout.firstChild.textContent = `HDG ${nf(f.att.yaw, 0)}°`;

    // FPA label bottom
    ctx.fillStyle = dim; ctx.font = "9px " + (cssVar("--font-mono") || "monospace"); ctx.textAlign = "center";
    ctx.fillText(`FPA ${nf(fpa, 1)}°`, cx, H - 8);
  }
}
