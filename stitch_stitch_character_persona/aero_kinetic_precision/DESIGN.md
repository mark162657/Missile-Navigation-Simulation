---
name: Aero-Kinetic Precision
colors:
  surface: '#10131b'
  surface-dim: '#10131b'
  surface-bright: '#363942'
  surface-container-lowest: '#0b0e16'
  surface-container-low: '#181c23'
  surface-container: '#1c2028'
  surface-container-high: '#272a32'
  surface-container-highest: '#31353d'
  on-surface: '#e0e2ed'
  on-surface-variant: '#c1c6d7'
  inverse-surface: '#e0e2ed'
  inverse-on-surface: '#2d3039'
  outline: '#8b90a0'
  outline-variant: '#414755'
  surface-tint: '#adc6ff'
  primary: '#adc6ff'
  on-primary: '#002e69'
  primary-container: '#4b8eff'
  on-primary-container: '#00285c'
  inverse-primary: '#005bc1'
  secondary: '#c0c1ff'
  on-secondary: '#1000a9'
  secondary-container: '#3131c0'
  on-secondary-container: '#b0b2ff'
  tertiary: '#ffb595'
  on-tertiary: '#571e00'
  tertiary-container: '#ef6719'
  on-tertiary-container: '#4c1a00'
  error: '#ffb4ab'
  on-error: '#690005'
  error-container: '#93000a'
  on-error-container: '#ffdad6'
  primary-fixed: '#d8e2ff'
  primary-fixed-dim: '#adc6ff'
  on-primary-fixed: '#001a41'
  on-primary-fixed-variant: '#004493'
  secondary-fixed: '#e1e0ff'
  secondary-fixed-dim: '#c0c1ff'
  on-secondary-fixed: '#07006c'
  on-secondary-fixed-variant: '#2f2ebe'
  tertiary-fixed: '#ffdbcc'
  tertiary-fixed-dim: '#ffb595'
  on-tertiary-fixed: '#351000'
  on-tertiary-fixed-variant: '#7c2e00'
  background: '#10131b'
  on-background: '#e0e2ed'
  surface-variant: '#31353d'
  simulation-red: '#FF3B30'
  guidance-blue: '#007AFF'
  status-green: '#34C759'
  warning-amber: '#FFCC00'
  terminal-gray: '#1C1C1E'
  data-neutral: '#8E8E93'
typography:
  headline-lg:
    fontFamily: JetBrains Mono
    fontSize: 24px
    fontWeight: '700'
    lineHeight: 32px
    letterSpacing: -0.02em
  headline-md:
    fontFamily: JetBrains Mono
    fontSize: 20px
    fontWeight: '600'
    lineHeight: 28px
  body-md:
    fontFamily: Inter
    fontSize: 14px
    fontWeight: '400'
    lineHeight: 20px
  body-sm:
    fontFamily: Inter
    fontSize: 12px
    fontWeight: '400'
    lineHeight: 18px
  data-mono:
    fontFamily: JetBrains Mono
    fontSize: 13px
    fontWeight: '500'
    lineHeight: 16px
  data-lg:
    fontFamily: JetBrains Mono
    fontSize: 18px
    fontWeight: '700'
    lineHeight: 24px
  label-caps:
    fontFamily: JetBrains Mono
    fontSize: 11px
    fontWeight: '700'
    lineHeight: 16px
    letterSpacing: 0.05em
  pfd-numeral:
    fontFamily: JetBrains Mono
    fontSize: 32px
    fontWeight: '700'
    lineHeight: 32px
rounded:
  sm: 0.125rem
  DEFAULT: 0.25rem
  md: 0.375rem
  lg: 0.5rem
  xl: 0.75rem
  full: 9999px
spacing:
  widget-gap: 1rem
  widget-padding: 1.25rem
  container-margin: 1.5rem
  nav-width: 240px
  nav-collapsed: 64px
---

## Brand & Style

The design system is engineered for high-stakes aerospace simulation, focusing on absolute precision, technical clarity, and rapid data interpretation. It prioritizes a "Mission Control" aesthetic—utilitarian, dense, and uncompromisingly professional.

The visual style is a hybrid of **Minimalism** and **Modern Corporate**, utilizing a widget-based architecture. This approach treats every data point, map, and PFD as a modular instrument. Surfaces are flat with subtle depth cues to indicate interactivity. The interface must remain unobtrusive, allowing the complex 3D trajectories and DEM (Digital Elevation Model) data to remain the focal point. The emotional response is one of calm, authoritative control and technical sophistication.

## Colors

The system operates primarily in **Dark Mode** to reduce eye strain during long-duration simulation monitoring and to provide maximum contrast for telemetry data. Light mode is available for planning and report generation in well-lit environments.

- **Primary (Guidance Blue):** Used for active trajectories, primary navigational paths, and "Pathfinder" logic.
- **Simulation Red:** Reserved exclusively for targets, impact zones, and critical flight failures.
- **Status Green:** Indicates successful parameter locks, "Detonated" status, and healthy system checks.
- **Neutral Palette:** Utilizes a range of deep grays to differentiate between the background, the collapsible navigation panel, and the draggable widgets. Map elements should use a desaturated base to allow colorful trajectory lines to pop.

## Typography

This system uses a dual-font strategy. **Inter** provides high legibility for UI labels, settings, and descriptive text. **JetBrains Mono** is the "Technical Soul" of the system, used for all numerical telemetry, PFD readouts, terminal windows, and mission-critical labels.

Data density is high. Use `data-mono` for most widget content to ensure alignment and readability in tight spaces. For the PFD (Primary Flight Display) and main banners, use `pfd-numeral` to ensure speed and altitude can be read at a glance. Headers should be concise and uppercase where possible to maintain a disciplined, military-spec feel.

## Layout & Spacing

The layout is a **Widget-Based System** built on a flexible grid. The core structure consists of a collapsible left navigation panel and a main workspace area where widgets live.

### Widget Philosophy
Each functional unit (Statistics, Terminal, PFD, Mission Config) is a "Widget." 
- Widgets must be resizable and movable.
- Use a 1rem (16px) gutter between widgets.
- The map is the "Underlay" or the "Master Widget" that typically fills the central viewport.

### Responsive Behavior
- **Desktop:** 12-column grid for widget placement.
- **Tablet:** 6-column grid; widgets stack vertically or hide in overflow tabs.
- **Navigation:** The left panel collapses to icons only to maximize the simulation workspace. When hidden, a thin edge-trigger allows for quick expansion.

## Elevation & Depth

To maintain the clean, minimalist aesthetic, the system avoids heavy shadows. Instead, it uses **Tonal Layers** and **Low-Contrast Outlines**.

1.  **Base Layer:** The darkest surface (`#000000` or deep gray).
2.  **Widget Surface:** Raised slightly via a subtle top-border or a very faint 4px ambient shadow with 0.2 opacity.
3.  **Active Indicators:** Elements like the current flight stage banner use high-contrast backgrounds (e.g., Guidance Blue) with no elevation, relying on color to denote importance.
4.  **Terminal Windows:** Should appear recessed into the UI, using a darker background than the surrounding widgets to simulate depth.

## Shapes

The design uses **Soft (0.25rem)** roundedness. This provides a modern touch while maintaining the rigid, "instrument" feel of a professional control panel. 

- **Widgets:** 4px border-radius.
- **Buttons:** 4px border-radius for technical buttons; larger components like the "Launch" button may use a slightly more pronounced radius but never fully pill-shaped.
- **Data Tags:** Sharp corners (0px) are permitted for small status labels within telemetry feeds to maximize horizontal space.

## Components

### Widgets
The primary container. Must include a header area with a drag handle (dotted pattern), a title in `label-caps`, and window controls (minimize/expand).

### Buttons
- **Primary:** Filled `guidance-blue`.
- **Destructive:** Outlined `simulation-red` for "Abort" or "Delete Missile."
- **Ghost:** Used within widget headers for settings/toggles.

### Data Monitors
Tables or lists using `data-mono`. Key/Value pairs should be aligned to a vertical center-line to allow for rapid scanning of parameter deviations. Use `status-green` for values within acceptable Kalman Filter thresholds.

### Terminal Window
Monospaced font on a `#0D0D0D` background. Use a blinking cursor and automatic scrolling. Progress bars within the terminal (e.g., for pathfinding) should be thin, 2px lines.

### PFD (Primary Flight Display)
A high-contrast component. The tape-style speed and altitude indicators should use `pfd-numeral`. The central area remains empty or shows a simplified pitch/roll ladder when the data is available.

### Flight Stage Banner
A full-width or prominent top-center bar. It uses a high-contrast background color that changes based on stage (e.g., "BOOST" in Amber, "TERMINAL GUIDANCE" in Red).

### 3D Visualization Window
A wireframe-only view of the tomahawk and terrain. Lines should be 1px wide. Trajectories should be rendered as glowing paths (Glow effect 2px).