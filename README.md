# Missile Guidance

Cruise-missile guidance, navigation, and mission-planning simulator over real-world DEM terrain (SRTM / merged GeoTIFF). Models 3-DoF flight physics, INS dead reckoning, GPS / TERCOM / radar-altimeter sensors, Kalman fusion, and C++ A* pathfinding with B-spline trajectory smoothing.

> **Work in progress.** This repository is under active development. APIs, file layout, and wiring between subsystems will change. The end-to-end simulation loop is **not** runnable yet — the guidance/control layer and the main scheduler are still being built. Use this for experimentation and learning, not production.

This is an **algorithm / navigation / guidance / pathfinding showcase**, not a hardware-fidelity simulator. The airframe is a 3-DoF point mass (no control surfaces, no attitude dynamics); maneuvers are commanded as accelerations and the plant enforces the g-envelope.

---

## What works today

- **Terrain** — load DEM tiles, query elevation, elevation patches (TERCOM / radar)
- **Planning** — C++ A* pathfinding over DEM pixels, B-spline path smoothing (`TrajectoryGenerator`)
- **Physics** — 3-DoF point-mass dynamics (RK4), ISA atmosphere, drag-polar aerodynamics, turbofan thrust, solid booster + `FlightSequencer` boost staging, wind/turbulence
- **Missile profiles** — per-missile JSON specs in `data/missiles/` (Tomahawk Block V, Kh-101)
- **Navigation modules** — INS, Kalman filter, GPS / TERCOM / IMU / timer (implemented, not fully wired)
- **Controls** — PID controller and the `ControlInput` plant boundary (built); autopilot wiring in progress
- **Visualization** — matplotlib mission plotter; optional Fastplotlib DEM viewers in `tests/`

## Not implemented / incomplete

- Main simulation runner (single loop: dynamics → sensors → nav → guidance → control)
- Autopilot `update()` — PID setup exists, the per-tick control law is unfinished (`missile/controls/autopilot.py`)
- Guidance path follower — L1 / pure-pursuit law (`missile/guidance/path_following.py` is a stub)
- Flight computer — BOOST → CRUISE → TERMINAL mode machine (`missile/controls/flight_computer.py` is a stub)
- End-to-end `NavigationComputer` fusion loop (currently driven by a toy constant-accel plant, not `dynamics.step()`)
- `datalink/` and `terminal/` packages (planned, empty)
- Automated test suite (manual scripts in `tests/` only)

---

## Project layout

```
src/
├── terrain/              DEM loading, coordinates, tile merge
├── missile/
│   ├── state.py          MissileState (truth vs estimate)
│   ├── profile.py        MissileProfile (basic + detailed + booster specs)
│   ├── config_store.py   Load/save JSON profiles
│   ├── navigation/       INS, KF, GPS, TERCOM, IMU, nav computer, timer
│   ├── planning/         C++ pathfinder wrapper, trajectory smoothing, cpp/
│   ├── guidance/         Path follower / L1 guidance (stub)
│   ├── controls/         PID, ControlInput, autopilot, flight computer, guidance law
│   └── datalink/         (planned)
├── simulation/
│   ├── physics/          dynamics (3-DoF, RK4), atmosphere, aerodynamics,
│   │                     engine, booster, sequencer, weather
│   └── sensors/          IMU, GPS receiver, radar + baro altimeter
├── terrain/              DEM loading, coordinates, tile merge
└── terminal/             (planned)

data/
├── dem/                  GeoTIFF elevation tiles (Siberia, Iran, SRTM)
└── missiles/             One JSON file per missile type

tests/                    Manual pathfinding / physics / nav / DEM scripts
```

Set `PYTHONPATH=src` when running Python from the project root.

**Dependency rule:** the arrow points one way — `simulation → missile`, never the reverse. The plant (`simulation.physics`) consumes types owned by the missile side (e.g. `ControlInput`); the missile avionics never import from `simulation`.

---

## Missile configurations

Each missile is a JSON file in `data/missiles/`:

- `tomahawk_block_v.json`
- `kh_101.json`

**Section 1 (basic)** — speeds, altitude envelope, g-limits (incl. boost-phase axial g), turn rates, range, cruise AGL band (user-facing specs).

**Section 2 (detailed)** — mass, fuel, IMU error model, sensor update rates (defaults provided; feeds INS via `MissileProfile.create_ins()`).

**Section 3 (booster)** — solid-rocket boost motor: thrust, burn time, propellant/casing mass, launch mode (defaults provided; consumed by the physics layer).

Load in Python:

```python
from missile.config_store import get_profile, load_profiles

profile = get_profile("Tomahawk Block V")
```

---

## Coordinate conventions

| Frame          | Fields                    | Use                                                     |
| -------------- | ------------------------- | ------------------------------------------------------- |
| **Geographic** | `lat`, `lon`, `alt` (MSL) | `MissileState`, INS, GPS, TERCOM                        |
| **Pixel**      | `row`, `col`              | DEM grid, C++ pathfinder                                |
| **ENU meters** | `east_m`, `north_m`, `up` | `CoordinateSystem`, dynamics, plotter (vs. launch origin) |

`MissileState` stores **geographic** position and **ENU** velocity — not pixels. The whole flight stack (physics, INS, guidance) works in **ENU / geographic**, z positive up. Convert at boundaries via `DEMLoader` or `CoordinateSystem`.

Truth vs estimate (the information barrier):

- `true_lat`, `true_lon`, `true_alt` — simulation ground truth (only sensors read this)
- `est_lat`, `est_lon`, `est_alt` — navigation estimate (INS + fusion); guidance and control read **only** these

---

## Pathfinding (C++ backend)

Build the C++ extension first (CMake project under `src/missile/planning/cpp/`), then run a benchmark with a DEM present in `data/dem/`:

```bash
PYTHONPATH=src python3 tests/test_pathfollower_load.py
```

This runs a ~600 km A* path over the Siberia DEM, smooths it into a 3D trajectory, and sizes the per-tick cost of the (not-yet-built) path follower.

> Note: `tests/test_get_path.py` is stale — `Pathfinding` now requires a `DEM_NAME` argument and the old `get_surfcae_distance` helper was removed.

---

## Design direction

**Two-lane simulation:** the universe maintains true position (advanced by `dynamics.step()`, the single truth integrator); the missile brain only sees noisy sensor measurements and maintains its own estimate (INS + Kalman filter). GPS provides horizontal fixes; radar altimeter provides AGL; TERCOM corrects horizontal position over rough terrain.

**GNC layering:**

- **Navigation** — *where am I?* INS dead-reckons; KF fuses GPS/TERCOM corrections into `est_*`.
- **Guidance** — *where should I go?* The path follower turns the planned trajectory into setpoints (lateral accel, target altitude, target speed) via L1 / pure-pursuit.
- **Control** — *how do I make the plant do it?* The autopilot turns setpoints into `ControlInput` (throttle, turn accel, climb accel) via PIDs + gravity feedforward.

During **boost**, guidance and control are dormant: the `FlightSequencer` inside the physics flies open-loop (programmed pitch-over) until turbofan handoff at cruise.

---

## Project work in progress...
