# Missile Guidance

A Python simulation and mission-planning project that models cruise-missile guidance, navigation, and control over real digital elevation model (DEM) terrain. It combines terrain-aware route planning with a 3-DoF flight model, noisy sensors, navigation estimation, guidance, control, telemetry, and a browser-based mission-control terminal.

> **Status: working, under active development.** The current simulation, planning pipeline, web terminal, and recorded-flight replay are usable. Performance improvements, bug fixes, validation, and future enhancements are still needed. This is an experimentation and learning project, not production or hardware-fidelity software.

## Highlights

- Terrain loading, elevation queries, and coordinate conversion for GeoTIFF DEM data.
- C++ A* terrain pathfinding with B-spline trajectory smoothing.
- 3-DoF point-mass flight physics with RK4 integration, atmosphere, aerodynamics, engine/booster stages, wind, and turbulence.
- Navigation stack with INS, GPS, IMU, radar/barometric altitude, TERCOM, and Kalman-based estimation.
- Guidance and control layers including path following, terminal guidance, PID autopilot, and flight-phase management.
- Interactive CLI mission flow that plans, flies, logs telemetry, and writes a mission result.
- Browser-based planning, live monitoring, replay, and final-report experience.

## Scope and limitations

The simulator is an algorithm, navigation, guidance, and pathfinding showcase. The vehicle model is a 3-DoF point mass: it does not model airframe control surfaces or full attitude dynamics. Maneuvers are represented by commanded accelerations constrained by the flight model.

Some modules and integrations remain incomplete or need hardening, including the datalink area, broader validation, and performance work for large DEMs and pathfinding. The project should be treated as a work in progress.

## Quick start

### Prerequisites

- Python 3
- CMake and a C++14-compatible compiler for live terrain pathfinding
- Python packages: `numpy`, `rasterio`, `scipy`, `pybind11`, and `pytest`

Install the web-terminal dependencies:

```bash
pip install -r frontend/requirements.txt
```

Build the C++ pathfinding extension for live planning and simulation:

```bash
cmake -S src/missile/planning/cpp -B src/missile/planning/cpp/build
cmake --build src/missile/planning/cpp/build
```

### Run an interactive simulation

From the project root:

```bash
PYTHONPATH=src python3 src/main.py
```

The CLI collects mission parameters, runs the simulation, and writes telemetry to `data/logs/` and results to `data/results/`.

### Run the web control terminal

```bash
python3 frontend/run.py
```

Open `http://127.0.0.1:8000`. For development with automatic reload:

```bash
python3 frontend/run.py --reload
```

The frontend has its own detailed documentation in [frontend/README.md](frontend/README.md).

## Project layout

```text
src/
├── main.py                 Interactive mission setup and simulation runner
├── terrain/                DEM loading, terrain queries, coordinates, tile merging
├── missile/
│   ├── navigation/         INS, GPS, IMU, TERCOM, Kalman filtering, navigation
│   ├── planning/           C++ pathfinder wrapper, trajectory generation, C++ source
│   ├── guidance/           Path following and terminal guidance
│   ├── controls/           PID control, autopilot, flight computer
│   └── datalink/           Datalink scaffolding
├── simulation/
│   ├── physics/            Dynamics, atmosphere, aerodynamics, propulsion, weather
│   └── sensors/            IMU, GPS, radar, and barometric-altimeter models
└── launcher/               Supporting launcher utilities

frontend/                   FastAPI backend and browser mission-control application
data/
├── dem/                    GeoTIFF elevation data
├── missiles/               Missile-profile JSON files
├── logs/                   Flight telemetry CSV output
└── results/                Mission-result JSON output
tests/                      Automated tests, benchmarks, visualizers, and scripts
```

Imports are rooted at `src/`, so use `PYTHONPATH=src` when running code from the repository root.

## How it works

The simulation keeps a separation between ground truth and the vehicle's estimated state:

- The physics model advances the true position and velocity.
- Simulated sensors observe that truth with realistic uncertainty.
- The navigation stack estimates position from those measurements.
- Guidance uses the estimate to convert the planned route into flight setpoints.
- Control converts setpoints into plant inputs for the physics model.

The main coordinate systems are geographic latitude/longitude/altitude, DEM pixel coordinates for route planning, and ENU meters for local dynamics. Positive ENU `up` is vertical.

## Testing

Run the primary automated checks with:

```bash
PYTHONPATH=src pytest tests/navigation/ tests/test_controls_guidance.py -v
```

The `tests/` directory also includes manual integration, performance, weather, and visualization scripts. Some require local DEM files, optional visualization dependencies, or the compiled C++ pathfinder.

## Frontend

The web control terminal provides:

- **Planning** for selecting DEM and profile data, defining a route, and running pathfinding.
- **Mission Control** for live telemetry or recorded-flight replay.
- **Final Report** for timeline review, flight statistics, and mission-result export.

See [frontend/README.md](frontend/README.md) for its architecture, API, and UI-specific notes.

## Credits

The frontend was created entirely with **Fable 5** and **Claude Opus 4.8 High**.

The project also uses open-source Python and web dependencies, CMake, pybind11, GeoTIFF/SRTM-style terrain data, and—in map mode—Leaflet with OpenStreetMap and CARTO attribution as rendered by the frontend.

## License

No license has been specified for this repository yet.
