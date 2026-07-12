# Product

## Register

product

## Users

A GNC (guidance, navigation & control) engineer/developer — the project author — and technical
reviewers (recruiters, engineers) he shows the simulator to. Context: desk, large screen, long
analytical sessions planning missions, watching live telemetry, and dissecting post-flight data.
The job to be done: plan a route over real DEM terrain, launch/monitor a simulated flight, and
audit every internal signal (Kalman, PID, TERCOM) the stack produces.

## Product Purpose

A browser mission-control terminal for a cruise-missile guidance simulator (FastAPI + vanilla ES
modules, zero build step). It exists to *showcase the algorithms*: pathfinding, sensor fusion,
guidance and flight dynamics. Success = a reviewer can fly a mission end-to-end and trust every
number on screen; nothing looks like a mockup.

## Brand Personality

Professional, precise, engineered. The register of real aerospace ground-segment software
(mission control consoles, avionics test rigs): dense but ordered, monospace where data lives,
quiet chrome, no decoration that doesn't inform.

## Anti-references

- Generic "AI-generated dashboard" look: gradient hero tiles, glassmorphism, identical rounded
  cards, floating pill badges, purple-on-dark neon.
- Consumer-app playfulness: emoji, bouncy motion, marketing copy.
- Game HUD kitsch: scanlines, fake glitch effects, sci-fi corner brackets.

## Design Principles

- **Data is the interface.** Every pixel of chrome must earn its place against one more telemetry
  value. Prefer denser, better-organized readouts over whitespace theatrics.
- **Truth over polish.** Never render a value that isn't real; show "—" honestly. States (armed,
  running, done) must always be legible at a glance.
- **Instrument, not poster.** Instrument panels (map, 3D, PFD) are dark and calibrated regardless
  of app theme; the surrounding shell is quiet and functional.
- **Progressive disclosure.** Headline values first; deep internals (PID terms, covariance,
  correlation scores) one deliberate click away, never lost.
- **Keyboard-and-mouse workstation.** Optimized for a desktop operator; degrade gracefully, don't
  chase phones.

## Accessibility & Inclusion

WCAG AA contrast targets for text in the shell; instrument overlays use the instrument ink ramp
(designed ≥4.5:1 on instrument backgrounds). Full reduced-motion support (`prefersReducedMotion`
already respected in players/animations). Stage colors are paired with text labels, never color
alone.
