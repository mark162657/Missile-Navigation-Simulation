# Missile Navigation Simulation

> **Work in Progress** — This repository is under active development. APIs, file layout, and wiring between subsystems will change without notice. The full simulation loop is **not** end-to-end runnable yet. Use this for experimentation and learning, not production.

Cruise-missile navigation and mission-planning simulator over real-world DEM terrain (SRTM / merged GeoTIFF). Models INS dead reckoning, GPS / TERCOM / radar-altimeter sensors, Kalman fusion, and C++ A\* pathfinding with B-spline trajectory smoothing.

---

## Status

| Area | Status |
|---|---|
| DEM terrain loading & queries | Working |
| C++ A\* pathfinder + B-spline smoothing | Working |
| Missile JSON profiles | Working |
| INS dead-reckoning | Implemented, unit-tested |
| Kalman filter | Implemented, unit-tested |
| GPS / TERCOM / timer sensors | Implemented, unit-tested |
| Navigation computer (fusion loop) | Implemented, unit-tested |
| Radar altimeter / barometric sensor | Implemented, unit-tested |
| Simulation runner (truth physics → sensors → nav → control) | **Not implemented** |
| Autopilot / proportional navigation | **Stub only** |
| Automated CI / end-to-end tests | **Not wired up** |

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
├── simulation/sensors/   GPS receiver, radar altimeter, baro altimeter
└── visualization/        Mission plotter

data/
├── dem/                  GeoTIFF elevation tiles (SRTM)
└── missiles/             One JSON file per missile type

tests/
├── navigation/           Unit tests for INS, KF, GPS, TERCOM, nav computer, sensors
└── *.py                  Manual pathfinding / DEM visualization scripts
```

Set `PYTHONPATH=src` when running Python from the project root.

---

## Quick start

### Build the C++ pathfinder

```bash
cd src/missile/planning/cpp
make
```

### Run navigation unit tests

```bash
PYTHONPATH=src pytest tests/navigation/
```

### Run a pathfinding benchmark

Requires a DEM tile in `data/dem/`:

```bash
PYTHONPATH=src python3 tests/test_get_path.py
```

---

## Missile configurations

Each missile is a JSON file in `data/missiles/`:

- `tomahawk_block_v.json`
- `kh_101.json`

**Section 1 (basic)** — speeds, altitude envelope, g-limits, turn rates, range, cruise AGL band.

**Section 2 (detailed)** — mass, fuel, IMU error model, sensor update rates. Feeds INS via `MissileProfile.create_ins()`.

```python
from missile.config_store import get_profile

profile = get_profile("Tomahawk Block V")
```

---

## Coordinate conventions

| Frame | Fields | Use |
|---|---|---|
| **Geographic** | `lat`, `lon`, `alt` (MSL) | `MissileState`, INS, GPS, TERCOM |
| **Pixel** | `row`, `col` | DEM grid, C++ pathfinder |
| **ENU meters** | `east_m`, `north_m` | `CoordinateSystem`, plotter (relative to launch origin) |

`MissileState` stores **geographic** position and **ENU** velocity. Convert at boundaries via `DEMLoader` or `CoordinateSystem`.

Truth vs estimate fields on `MissileState`:

- `true_lat / true_lon / true_alt` — simulation ground truth (sensors read from these)
- `est_lat / est_lon / est_alt` — navigation estimate (INS + Kalman fusion output)

---

## Design direction

**Two-lane simulation:** the universe maintains true position; the missile brain only sees noisy sensor measurements and maintains its own estimate (INS + Kalman filter). GPS provides horizontal fixes; radar altimeter provides AGL; TERCOM corrects horizontal position over rough terrain. The pathfinder plans a terrain-following route in DEM pixel space; `TrajectoryGenerator` smooths it to a continuous B-spline.

The next major piece is a simulation runner that drives the truth-state forward in time, feeds sensors, and closes the nav/control loop.
