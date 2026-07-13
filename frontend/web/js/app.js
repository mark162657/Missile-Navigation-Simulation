// Application shell: theme, navigation, screen routing, and the shared mission
// store that carries a planned route from Planning → Mission → Report.

import { $, $$, el, toast } from "./util.js";
import { api } from "./api.js";
import { PlanningScreen } from "./planning.js";
import { MissionScreen } from "./mission.js";
import { ReportScreen } from "./report.js";

const SCREENS = {
  planning: { name: "Planning", crumb: "Route & mission configuration", ctor: PlanningScreen },
  mission: { name: "Mission Control", crumb: "Live telemetry & guidance", ctor: MissionScreen },
  report: { name: "Final Report", crumb: "Post-flight analysis", ctor: ReportScreen },
};

class App {
  constructor() {
    this.store = {
      dems: [], profiles: [],
      plan: null,          // planned route: { dem_name, start_gps, target_gps, trajectory, profile, config }
      mission: null,       // loaded telemetry: { id, frames, result }
      liveTrajectory: null,
    };
    this.screens = {};
    this.current = null;
    this.appEl = $("#app");
    this._boot();
  }

  async _boot() {
    this._initTheme();
    this._initNav();

    // Prefetch catalog so screens open instantly.
    try {
      const [dems, profiles] = await Promise.all([api.dems(), api.profiles()]);
      this.store.dems = dems.filter((d) => !d.error);
      this.store.profiles = profiles;
    } catch (e) {
      toast(`Backend unreachable: ${e.message}`, "err");
    }

    const initial = (location.hash || "#planning").slice(1);
    this.go(SCREENS[initial] ? initial : "planning");
    window.addEventListener("hashchange", () => {
      const s = location.hash.slice(1);
      if (SCREENS[s] && s !== this.current) this.go(s);
    });
  }

  // --- theme ----------------------------------------------------------------
  _initTheme() {
    const saved = localStorage.getItem("gnc-theme");
    const theme = saved || (matchMedia("(prefers-color-scheme: light)").matches ? "light" : "dark");
    this._setTheme(theme);
    $$("[data-theme-set]").forEach((b) =>
      b.addEventListener("click", () => this._setTheme(b.dataset.themeSet)));
  }
  _setTheme(theme) {
    document.documentElement.dataset.theme = theme;
    localStorage.setItem("gnc-theme", theme);
    $$("[data-theme-set]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.themeSet === theme)));
    window.dispatchEvent(new CustomEvent("themechange", { detail: theme }));
  }

  // --- navigation -----------------------------------------------------------
  _initNav() {
    $$(".nav__item").forEach((b) =>
      b.addEventListener("click", () => this.go(b.dataset.screen)));
    $("#navToggle").addEventListener("click", () => this.appEl.classList.toggle("nav-collapsed"));
  }

  // --- routing --------------------------------------------------------------
  go(id) {
    const def = SCREENS[id];
    if (!def) return;
    if (this.current && this.screens[this.current]) this.screens[this.current].deactivate?.();

    $$(".nav__item").forEach((b) => b.setAttribute("aria-current", b.dataset.screen === id ? "page" : "false"));
    $$(".screen").forEach((s) => (s.hidden = s.id !== `screen-${id}`));
    $("#screenName").textContent = def.name;
    $("#screenCrumb").textContent = def.crumb;
    this.setActions([]);
    $("#stageBannerMount").replaceChildren();

    if (!this.screens[id]) {
      const root = $(`#screen-${id}`);
      this.screens[id] = new def.ctor(root, this);
    }
    this.current = id;
    history.replaceState(null, "", `#${id}`);
    this.screens[id].activate?.();
  }

  navigateTo(id) { this.go(id); }

  // --- topbar actions (per-screen) -----------------------------------------
  setActions(nodes) {
    const host = $("#topbarActions");
    host.replaceChildren();
    [].concat(nodes).filter(Boolean).forEach((n) => host.appendChild(n));
  }

  // Build a "restore hidden widgets" menu button bound to a workspace.
  hiddenWidgetsControl(workspace) {
    const btn = el("button", { class: "btn btn--sm btn--ghost", title: "Show hidden widgets" }, [
      el("span", { class: "mi", text: "widgets" }), el("span", { text: "Widgets" }),
    ]);
    const menu = el("div", {
      class: "widget-menu",
      style: { position: "fixed", zIndex: "var(--z-popover)", display: "none" },
    });
    document.body.appendChild(menu);

    const render = (hidden) => {
      btn.style.display = hidden.length ? "" : "none";
      menu.replaceChildren();
      hidden.forEach((w) => {
        menu.appendChild(el("button", {
          class: "btn btn--sm btn--ghost btn--block", onClick: () => { w.show(); close(); },
        }, [el("span", { class: "mi", text: "add" }), el("span", { text: w.spec.title })]));
      });
    };
    const open = () => {
      const r = btn.getBoundingClientRect();
      Object.assign(menu.style, {
        display: "flex", flexDirection: "column", gap: "2px", top: `${r.bottom + 6}px`, right: `${innerWidth - r.right}px`,
        background: "var(--surface-2)", border: "1px solid var(--line-2)", borderRadius: "var(--r-sm)",
        padding: "4px", boxShadow: "var(--shadow-float)", minWidth: "180px",
      });
    };
    const close = () => (menu.style.display = "none");
    btn.addEventListener("click", (e) => { e.stopPropagation(); menu.style.display === "flex" ? close() : open(); });
    document.addEventListener("click", close);
    workspace.on("hidden-change", render);
    render(workspace.hiddenWidgets());
    btn._cleanup = () => menu.remove();
    return btn;
  }
}

window.addEventListener("DOMContentLoaded", () => { window.app = new App(); });
