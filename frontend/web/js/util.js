// Small DOM + formatting helpers shared across the app.

export function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null || v === false) continue;
    if (k === "class") node.className = v;
    else if (k === "html") node.innerHTML = v;
    else if (k === "text") node.textContent = v;
    else if (k === "dataset") Object.assign(node.dataset, v);
    else if (k.startsWith("on") && typeof v === "function") node.addEventListener(k.slice(2).toLowerCase(), v);
    else if (k === "style" && typeof v === "object") Object.assign(node.style, v);
    else node.setAttribute(k, v);
  }
  for (const c of [].concat(children)) {
    if (c == null || c === false) continue;
    node.appendChild(typeof c === "string" ? document.createTextNode(c) : c);
  }
  return node;
}

export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

export function clear(node) { while (node.firstChild) node.removeChild(node.firstChild); return node; }

// --- number formatting -----------------------------------------------------
export const nf = (v, d = 0) =>
  v == null || Number.isNaN(v) ? "—" : Number(v).toLocaleString("en-US", { minimumFractionDigits: d, maximumFractionDigits: d });

export function fmtDist(m, d = 1) {
  if (m == null || Number.isNaN(m)) return "—";
  return Math.abs(m) >= 1000 ? `${nf(m / 1000, d)} km` : `${nf(m, 0)} m`;
}
export function fmtTime(s) {
  if (s == null || Number.isNaN(s)) return "—";
  const sign = s < 0 ? "-" : "";
  s = Math.abs(s);
  const mm = Math.floor(s / 60), ss = s % 60;
  return `${sign}${String(mm).padStart(2, "0")}:${ss.toFixed(1).padStart(4, "0")}`;
}
export const fmtLat = (v) => `${Math.abs(v).toFixed(5)}°${v >= 0 ? "N" : "S"}`;
export const fmtLon = (v) => `${Math.abs(v).toFixed(5)}°${v >= 0 ? "E" : "W"}`;

export function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }
export function lerp(a, b, t) { return a + (b - a) * t; }

// --- toast ------------------------------------------------------------------
export function toast(msg, kind = "") {
  const host = $("#toasts");
  if (!host) return;
  const icon = kind === "err" ? "error" : kind === "ok" ? "check_circle" : "info";
  const t = el("div", { class: `toast ${kind ? "toast--" + kind : ""}` }, [
    el("span", { class: "mi", "aria-hidden": "true", text: icon }),
    el("span", { text: msg }),
  ]);
  host.appendChild(t);
  setTimeout(() => { t.style.opacity = "0"; t.style.transition = "opacity .3s"; setTimeout(() => t.remove(), 320); }, 3600);
}

// --- reduced motion ---------------------------------------------------------
export const prefersReducedMotion = () =>
  window.matchMedia("(prefers-reduced-motion: reduce)").matches;

// Debounced resize observer helper.
export function onResize(node, cb) {
  const ro = new ResizeObserver(() => cb());
  ro.observe(node);
  return () => ro.disconnect();
}

// Size a canvas to its display box at devicePixelRatio; returns [w,h,dpr] in CSS px.
export function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, Math.round(rect.width));
  const h = Math.max(1, Math.round(rect.height));
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [w, h, dpr, ctx];
}

// Resolve a CSS custom property to its computed value.
export function cssVar(name, root = document.documentElement) {
  return getComputedStyle(root).getPropertyValue(name).trim();
}
