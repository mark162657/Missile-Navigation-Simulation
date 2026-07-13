// Widget framework: a CSS-grid workspace of draggable, resizable, collapsible,
// full-screenable, hideable panels. Each screen builds a Workspace and adds
// Widgets to it. Layout is a 12-track grid; widgets carry a column/row span the
// user can resize by the corner handle, reorder by dragging the header, and
// hide (restorable from the topbar "Widgets" menu).

import { el, clamp } from "./util.js";

const COLS = 12;
const ROW_H = 78;   // px per grid row track
const MIN_COLS = 3;
const MIN_ROWS = 2;

export class Widget {
  constructor(spec, workspace) {
    this.spec = spec;
    this.workspace = workspace;
    this.id = spec.id;
    this.cols = spec.cols ?? 4;
    this.rows = spec.rows ?? 4;
    this.collapsed = false;
    this.fullscreen = false;
    this.hidden = false;
    this._build();
  }

  _build() {
    const s = this.spec;
    this.body = el("div", { class: `widget__body ${s.instrument ? "is-instrument" : ""} ${s.flush ? "is-flush" : ""}` });

    this.meta = el("div", { class: "widget__head-meta" });
    this.tools = el("div", { class: "widget__tools" });

    this.head = el("div", { class: "widget__head" }, [
      el("span", { class: "widget__grip", "aria-hidden": "true" }, [el("span", { class: "mi", text: "drag_indicator" })]),
      el("span", { class: "widget__title", title: s.title, text: s.title }),
      this.meta,
      el("span", { class: "widget__spacer" }),
      this.tools,
    ]);

    this._tool("collapse", "expand_less", "Collapse", () => this.toggleCollapse());
    this._tool("fullscreen", "open_in_full", "Full screen", () => this.toggleFullscreen());
    this._tool("hide", "close", "Hide", () => this.hide());

    this.resizeHandle = el("div", { class: "widget__resize", title: "Resize" });

    this.node = el("div", { class: "widget", "data-widget": this.id }, [this.head, this.body, this.resizeHandle]);
    this._applySpan();

    this._wireDrag();
    this._wireResize();

    if (s.build) s.build(this.body, this);
  }

  _tool(name, icon, title, onClick) {
    const b = el("button", { class: "widget__tool", "data-tool": name, title, "aria-label": title, onClick });
    b._icon = el("span", { class: "mi", text: icon });
    b.appendChild(b._icon);
    this.tools.appendChild(b);
    return b;
  }

  _applySpan() {
    this.node.style.gridColumn = `span ${this.cols}`;
    this.node.style.gridRow = `span ${this.collapsed ? 1 : this.rows}`;
  }

  setMeta(node) { this.meta.replaceChildren(typeof node === "string" ? document.createTextNode(node) : node); }

  toggleCollapse() {
    this.collapsed = !this.collapsed;
    this.node.classList.toggle("is-collapsed", this.collapsed);
    this.tools.querySelector('[data-tool="collapse"] .mi').textContent = this.collapsed ? "expand_more" : "expand_less";
    this._applySpan();
    this.workspace.emit("layout", this);
  }

  toggleFullscreen() {
    this.fullscreen = !this.fullscreen;
    this.node.classList.toggle("is-fullscreen", this.fullscreen);
    this.tools.querySelector('[data-tool="fullscreen"] .mi').textContent = this.fullscreen ? "close_fullscreen" : "open_in_full";
    this.workspace.emit("layout", this);
  }

  hide() {
    this.hidden = true;
    if (this.fullscreen) this.toggleFullscreen();
    this.node.style.display = "none";
    this.workspace.onHiddenChange();
    this.workspace.emit("layout", this);
  }

  show() {
    this.hidden = false;
    this.node.style.display = "";
    this.workspace.onHiddenChange();
    this.workspace.emit("layout", this);
  }

  // --- header drag to reorder (with a grid drop placeholder) ---------------
  _wireDrag() {
    let startX, startY, moved, offX, offY, placeholder;

    const onDown = (e) => {
      if (e.target.closest(".widget__tool") || this.fullscreen) return;
      if (e.button !== undefined && e.button !== 0) return;
      const r = this.node.getBoundingClientRect();
      startX = e.clientX; startY = e.clientY; offX = e.clientX - r.left; offY = e.clientY - r.top;
      moved = false;
      // Listen on window so pointerup ALWAYS fires, even off the header.
      window.addEventListener("pointermove", onMove);
      window.addEventListener("pointerup", onUp, { once: true });
      window.addEventListener("pointercancel", onUp, { once: true });
    };

    const begin = () => {
      moved = true;
      const r = this.node.getBoundingClientRect();
      // Placeholder keeps the grid slot while the node floats.
      placeholder = el("div", { class: "widget-placeholder" });
      placeholder.style.gridColumn = this.node.style.gridColumn;
      placeholder.style.gridRow = this.node.style.gridRow;
      this.node.parentNode.insertBefore(placeholder, this.node);
      Object.assign(this.node.style, {
        position: "fixed", width: `${r.width}px`, height: `${r.height}px`,
        left: `${r.left}px`, top: `${r.top}px`, margin: "0",
      });
      this.node.classList.add("is-dragging");
      document.body.style.cursor = "grabbing";
    };

    const onMove = (e) => {
      if (!moved) { if (Math.hypot(e.clientX - startX, e.clientY - startY) < 6) return; begin(); }
      this.node.style.left = `${e.clientX - offX}px`;
      this.node.style.top = `${e.clientY - offY}px`;
      this._movePlaceholder(e.clientX, e.clientY, placeholder);
    };

    const onUp = () => {
      window.removeEventListener("pointermove", onMove);
      if (!moved) return;
      // Snap: drop the node where the placeholder sits, back into grid flow.
      this.node.classList.remove("is-dragging");
      this.node.style.position = ""; this.node.style.left = ""; this.node.style.top = "";
      this.node.style.width = ""; this.node.style.height = ""; this.node.style.margin = "";
      document.body.style.cursor = "";
      if (placeholder) { this.workspace.grid.insertBefore(this.node, placeholder); placeholder.remove(); placeholder = null; }
      this._applySpan();
      this.workspace.emit("layout", this);
    };

    this.head.addEventListener("pointerdown", onDown);
  }

  _movePlaceholder(x, y, placeholder) {
    const grid = this.workspace.grid;
    const sibs = Array.from(grid.children).filter(
      (n) => n !== this.node && n !== placeholder && n.style.display !== "none");
    for (const sib of sibs) {
      const r = sib.getBoundingClientRect();
      if (x >= r.left && x <= r.right && y >= r.top && y <= r.bottom) {
        const before = (y - r.top) < r.height / 2 || (Math.abs(y - (r.top + r.height / 2)) < 8 && (x - r.left) < r.width / 2);
        grid.insertBefore(placeholder, before ? sib : sib.nextSibling);
        return;
      }
    }
  }

  // --- corner resize (snap to grid tracks) --------------------------------
  _wireResize() {
    let startX, startY, startCols, startRows, colW;
    const onDown = (e) => {
      e.preventDefault(); e.stopPropagation();
      const gridRect = this.workspace.grid.getBoundingClientRect();
      const gap = parseFloat(getComputedStyle(this.workspace.grid).columnGap) || 12;
      colW = (gridRect.width - gap * (COLS - 1)) / COLS + gap;
      startX = e.clientX; startY = e.clientY; startCols = this.cols; startRows = this.rows;
      this.resizeHandle.setPointerCapture(e.pointerId);
      this.resizeHandle.addEventListener("pointermove", onMove);
      this.resizeHandle.addEventListener("pointerup", onUp);
    };
    const onMove = (e) => {
      const dCols = Math.round((e.clientX - startX) / colW);
      const dRows = Math.round((e.clientY - startY) / (ROW_H + 12));
      this.cols = clamp(startCols + dCols, MIN_COLS, COLS);
      this.rows = clamp(startRows + dRows, MIN_ROWS, 16);
      if (this.collapsed) { this.collapsed = false; this.node.classList.remove("is-collapsed"); }
      this._applySpan();
    };
    const onUp = (e) => {
      this.resizeHandle.releasePointerCapture?.(e.pointerId);
      this.resizeHandle.removeEventListener("pointermove", onMove);
      this.resizeHandle.removeEventListener("pointerup", onUp);
      this.workspace.emit("layout", this);
    };
    this.resizeHandle.addEventListener("pointerdown", onDown);
  }
}

export class Workspace {
  constructor(mount) {
    this.grid = el("div", { class: "workspace" });
    this.grid.style.gridTemplateColumns = `repeat(${COLS}, 1fr)`;
    this.grid.style.gridAutoRows = `${ROW_H}px`;
    this.grid.style.gridAutoFlow = "row dense";
    mount.appendChild(this.grid);
    this.widgets = new Map();
    this._handlers = {};
  }

  add(spec) {
    const w = new Widget(spec, this);
    this.widgets.set(w.id, w);
    this.grid.appendChild(w.node);
    return w;
  }

  get(id) { return this.widgets.get(id); }

  on(evt, fn) { (this._handlers[evt] ||= []).push(fn); }
  emit(evt, ...args) { (this._handlers[evt] || []).forEach((fn) => fn(...args)); }

  onHiddenChange() { this.emit("hidden-change", this.hiddenWidgets()); }
  hiddenWidgets() { return Array.from(this.widgets.values()).filter((w) => w.hidden); }

  destroy() { this.grid.remove(); this.widgets.clear(); this._handlers = {}; }
}
