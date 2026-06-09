# Missile Guidance

Cruise-missile navigation and mission-planning simulator over real-world DEM terrain (SRTM / merged GeoTIFF). Models INS dead reckoning, GPS / TERCOM / radar-altimeter sensors, Kalman fusion, and C++ A* pathfinding with B-spline trajectory smoothing.

> **Work in progress.** This repository is under active development. APIs, file layout, and wiring between subsystems will change. Navigation fusion and the full simulation loop are **not** end-to-end runnable yet. Use this for experimentation and learning, not production.

---

## What works today

- **Terrain** — load DEM tiles, query elevation, elevation patches (TERCOM / radar)
- **Planning** — C++ A* pathfinding over DEM pixels, B-spline path smoothing (`TrajectoryGenerator`)
- **Missile profiles** — per-missile JSON specs in `data/missiles/` (Tomahawk Block V, Kh-101)
- **Navigation modules** — INS, Kalman filter, GPS / TERCOM / timer (implemented, not fully wired)
- **Visualization** — matplotlib mission plotter; optional Fastplotlib DEM viewers in `tests/`

## Not implemented / incomplete

- Simulation runner (truth physics → sensors → nav → control loop)
- Autopilot / proportional navigation (`src/missile/control/autopilot.py` is empty)
- End-to-end `NavigationComputer` loop (GPS → KF → INS feedback)
- Web / desktop planner UIs (removed from this repo)
- Automated test suite (manual scripts in `tests/` only)

---

## Project layout

```
src/
├── terrain/              DEM loading, coordinates, tile merge
├── missile/
│   ├── state.py          MissileState (truth vs estimate)
│   ├── profile.py        MissileProfile (basic + detailed specs)
│   ├── config_store.py   Load/save JSON profiles
│   ├── navigation/       INS, KF, GPS, TERCOM, nav computer
│   ├── planning/         C++ pathfinder wrapper, trajectory
│   └── control/          Autopilot (stub)
├── simulation/sensors/   GPS receiver, radar altimeter, LIDAR (stub)
└── visualization/        Mission plotter

data/
├── dem/                  GeoTIFF elevation tiles
└── missiles/             One JSON file per missile type

tests/                    Manual pathfinding / DEM visualization scripts
```

Set `PYTHONPATH=src` when running Python from the project root.

---

## Missile configurations

Each missile is a JSON file in `data/missiles/`:

- `tomahawk_block_v.json`
- `kh_101.json`

**Section 1 (basic)** — speeds, altitude envelope, g-limits, turn rates, range, cruise AGL band (user-facing specs).

**Section 2 (detailed)** — mass, fuel, IMU error model, sensor update rates (defaults provided; feeds INS via `MissileProfile.create_ins()`).

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
| **ENU meters** | `east_m`, `north_m`       | `CoordinateSystem`, plotter (relative to launch origin) |


`MissileState` stores **geographic** position and **ENU** velocity — not pixels. Convert at boundaries via `DEMLoader` or `CoordinateSystem`.

Truth vs estimate:

- `true_lat`, `true_lon`, `true_alt` — simulation ground truth (sensors read this)
- `est_lat`, `est_lon`, `est_alt` — navigation estimate (INS + fusion)

---

## Pathfinding (C++ backend)

Build the C++ extension first:

```bash
cd src/missile/planning/cpp
make
```

Run a pathfinding benchmark (requires a DEM in `data/dem/`):

```bash
PYTHONPATH=src python3 tests/test_get_path.py
```

---

## Design direction

**Two-lane simulation:** the universe maintains true position; the missile brain only sees noisy sensor measurements and maintains its own estimate (INS + Kalman filter). GPS provides horizontal fixes; radar altimeter provides AGL; TERCOM corrects horizontal position over rough terrain.

---

## Project work in progress...

