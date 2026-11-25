# Missile Guidance

A Python simulation and mission-planning suite that models cruise-missile guidance, navigation, and control (GNC) over Digital Elevation Model (DEM) terrain. It combines native terrain-aware route planning and B-spline trajectory smoothing with a 500 Hz 3-DoF flight simulation, noisy sensor models, multi-sensor Kalman navigation, autopilot control, flight telemetry, and a zero-build browser-based Web Control Terminal.

> **Status: active development.** This is an algorithm, GNC, and pathfinding experimentation project—not production or hardware-fidelity software.
>
> **Pathfinding note:** The new **Rust CFA\*** (Coarse-to-Fine A\*) engine is currently **incomplete / under active development**. Use the **C++ A\*** engine (`C++ · A*` in the Web Control Terminal) for working route planning and live mission execution.

---

## Highlights

- **Memory-mapped GeoTIFF terrain** — streams large GeoTIFF DEMs into a read-only `float32` `.npy` disk cache (`*.f32.npy`) on first use for instant memory-mapped startup, elevation/patch queries, coarse block-map downsampling, and WGS84 / pixel / local ENU coordinate transforms.
- **Dual native pathfinding architecture** with **B-spline trajectory smoothing**:
  - **C++ A\* (`missile_backend`)** — complete, operational weighted 8-connected A\* terrain planner (`pybind11` + CMake) with climb/descent grade limits and turn-angle penalties.
  - **Rust CFA\* (`pathfinder`, work in progress)** — next-generation Coarse-to-Fine A\* crate (`PyO3` + `maturin`) wired into Python and the web UI, with core search logic currently under construction.
- **3-DoF point-mass flight physics** — 500 Hz RK4 integration, solid-rocket boost phase, air-breathing cruise propulsion, flight stage sequencer, ISA atmosphere, mean wind shear, and Dryden turbulence gusts.
- **Sensors & navigation fusion** — simulated IMU, GPS receiver, barometric altimeter, and radar altimeter feeding strapdown INS, TERCOM terrain-contour correlation, and Kalman filter state estimation.
- **Multi-phase guidance & control** — boost climb guidance, lookahead route tracking, terminal dive guidance with configurable impact angle, and cascaded PID autopilot.
- **Web Control Terminal & CLI runner** — browser workspace for Planning, live Mission Control, Flight Replay, and Final Reports (2D hillshade map, 3D viewer, PFD, live PID/Kalman/TERCOM telemetry over WebSockets), plus an interactive terminal runner.

---

## Pathfinding Engines & Current State

The codebase contains two native pathfinding backends bridged to Python and exposed in the Web Control Terminal (`GET /api/engines`, `POST /api/plan`):

| Engine | Location | Build Tooling | Status | Details |
|---|---|---|---|---|
| **C++ · A\*** | `src/missile/planning/` (`cpp/pathfinder.cpp`) | CMake + `pybind11` | **Working (Recommended)** | Full weighted A\* over fine DEM pixels with terrain grade constraints, turn penalties, nodata rejection, and terrain-profile sampling. |
| **Rust · CFA\*** | `src/missile/pathfinding/` (`pathfinder_backend/`) | Cargo + `PyO3` + `maturin` | **Incomplete (WIP)** | Intended to scale to massive merged DEMs via Coarse-to-Fine A\* (`downsample_k`). Bindings and UI controls are wired up, but core search is not yet finished. |

### What remains in the Rust CFA\* implementation

- **Coarse-to-fine pipeline (`src/cfa.rs`)**: Currently empty; coarse block-map search (`DEMLoader.downsample(k)`) and corridor refinement on the fine grid have not been implemented yet.
- **Fine A\* search (`src/astar.rs`)**: Priority queue (`BinaryHeap`), direction tables, `CostWeight` struct, and parent-chain `reconstruct_path` are in place, but `get_movement_cost` currently returns a placeholder constant (`1.0`) and the neighbour expansion loop does not yet compute `g_score`/heuristic `f_score` or push neighbours into `open_set` / `visited`.
- **Python wrapper (`src/missile/pathfinding/pathfinding_backend.py`)**: Calls `pathfinder.rs_pathfinder(...)` with zero-copy NumPy slices, but currently overwrites `path = None` while benchmarking/scaffolding is in progress and does not yet accept `downsample_k` / `heuristic_weight` keyword arguments directly (handled defensively by `frontend/backend/planning.py`).

> **Tip when using the Web Control Terminal:** `frontend/backend/native_backend.py` currently lists `rust` as `DEFAULT_ENGINE`. Switch the engine selector in the **Planning** tab to **`C++ · A*`** before computing a route or launching a live simulation.

---

## Supported Platforms & Prerequisites

- **Windows 10 / 11** (64-bit)
- **macOS** (Apple Silicon or Intel)
- **Linux** (64-bit, with standard GCC/Clang and Python dev headers)

Compiled extension modules (`.pyd` on Windows, `.so` on macOS/Linux) are specific to both your OS/architecture and Python minor version (e.g., CPython 3.13 vs. 3.14). Always build native modules using the **exact Python interpreter** that launches the simulator.

### Required software

- **Python 3.10+** (64-bit)
- **CMake 3.15+** and a **C++14-compatible compiler** (required for the working C++ A\* backend):
  - **macOS**: Xcode Command Line Tools (`xcode-select --install`) and CMake (`brew install cmake`)
  - **Windows**: Visual Studio Build Tools (**Desktop development with C++**, MSVC, Windows 10/11 SDK) and CMake on `PATH`
  - **Linux**: `cmake`, `g++` or `clang++`, and `python3-dev`
- **Data files**: at least one GeoTIFF DEM in `data/dem/` and one missile profile JSON in `data/missiles/` (e.g., `tomahawk_block_v.json`).
- **Optional (for Rust CFA\* development)**: Rust toolchain supporting **Edition 2024** (`rustup`, Rust 1.85+) and `maturin` (`pip install maturin`).

---

## Python Dependencies

Installing `frontend/requirements.txt` provides all direct runtime and C++ build dependencies (Node.js and npm are **not** required):

| Package | Purpose |
|---|---|
| `fastapi` | REST API and live simulation WebSocket server |
| `pydantic` | Request/configuration validation |
| `uvicorn[standard]` | ASGI server, hot reload, and WebSocket transport |
| `numpy` | Physics, Kalman filter, coordinate arrays, and DEM memmaps |
| `rasterio` | GeoTIFF DEM reading, CRS metadata, and affine transforms |
| `scipy` | B-spline trajectory smoothing |
| `matplotlib` | Terrain colourmap and hillshade rendering utilities |
| `pybind11` | C++ A\* Python bindings and CMake integration |

Optional developer packages:

| Package | When needed |
|---|---|
| `maturin` | Building/developing the experimental Rust `pathfinder` PyO3 extension |
| `fastplotlib` | Manual GPU DEM & pathfinding visualizers in `tests/` |
| `pytest` | Running unit/integration test scripts |

---

## Setup & Build Instructions

Run all commands from the **project root**.

### macOS / Linux

#### 1. Create virtual environment and install dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r frontend/requirements.txt
```

#### 2. Build the C++ A\* pathfinder (required for live planning)

```bash
python_exe="$(python -c 'import sys; print(sys.executable)')"
cmake -S src/missile/planning/cpp -B src/missile/planning/cpp/build -DPython3_EXECUTABLE="$python_exe"
cmake --build src/missile/planning/cpp/build --config Release
```

Verify the compiled C++ module imports cleanly:

```bash
python -c "import sys; sys.path.insert(0, 'src'); from missile.planning import missile_backend; print('C++ A* pathfinder ready')"
```

#### 3. (Optional) Build the experimental Rust CFA\* crate

Only needed if you are working on `src/missile/pathfinding/pathfinder_backend`:

```bash
python -m pip install maturin
maturin develop --release --manifest-path src/missile/pathfinding/pathfinder_backend/Cargo.toml
```

---

### Windows (PowerShell)

#### 1. Create virtual environment and install dependencies

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r frontend\requirements.txt
```

*(If PowerShell script execution is restricted, omit `Activate.ps1` and invoke `.\.venv\Scripts\python.exe` directly in all commands below.)*

#### 2. Build the C++ A\* pathfinder (required for live planning)

```powershell
$pythonExe = python -c "import sys; print(sys.executable)"
cmake -S src\missile\planning\cpp -B src\missile\planning\cpp\build "-DPython3_EXECUTABLE=$pythonExe"
cmake --build src\missile\planning\cpp\build --config Release
```

*(If `cmake` or MSVC is not detected, run from the **x64 Native Tools Command Prompt for Visual Studio**.)*

Verify the compiled C++ module imports cleanly:

```powershell
python -c "import sys; sys.path.insert(0, 'src'); from missile.planning import missile_backend; print('C++ A* pathfinder ready')"
```

#### 3. (Optional) Build the experimental Rust CFA\* crate

```powershell
python -m pip install maturin
maturin develop --release --manifest-path src\missile\pathfinding\pathfinder_backend\Cargo.toml
```

---

## Running the Project

### 1. Web Control Terminal (Recommended)

```bash
python frontend/run.py
```

For backend auto-reload during development:

```bash
python frontend/run.py --reload
```

Open **`http://127.0.0.1:8000`** in your browser:
- **Planning**: Pick a DEM tile and missile profile, place Launch and Target pins, choose **`C++ · A*`** as the pathfinding engine, and generate a smoothed terrain-following route.
- **Mission Control**: Stream a live 500 Hz simulation over WebSocket (with adjustable playback pacing, live PID loops, TERCOM correlation diagnostics, and Kalman 1-σ covariance), or replay any saved CSV flight log from `data/logs/` (replays work even without native pathfinder binaries).
- **Final Report**: Inspect mission outcome verdicts, miss distance, flight profile charts, and saved JSON summaries.

See [frontend/README.md](frontend/README.md) for REST/WebSocket endpoint details.

### 2. Interactive CLI Simulation

macOS / Linux:

```bash
PYTHONPATH=src python src/terrain/main.py
```

Windows PowerShell:

```powershell
$env:PYTHONPATH = "src"
python src\terrain\main.py
```

The CLI runner walks interactively through missile profile selection/creation, DEM selection, GPS coordinates, wind conditions, and terminal dive parameters, using the C++ A\* planner (`src/missile/planning/`) and writing flight logs to `data/logs/` and mission verdicts to `data/results/`.

---

## Terrain Caching & Large DEM Memory Warning

1. **On-disk `float32` memmap cache**: On first load of a GeoTIFF (`data/dem/<tile>.tif`), `DEMLoader` streams the raster in 64 MB strips to build an uncompressed `*.f32.npy` cache alongside the `.tif` (`4 bytes` per pixel on disk) and memory-maps it on subsequent runs. Startup is near-instant and the OS pages in only touched regions.
2. **C++ A\* search memory**: While `DEMLoader` avoids loading the entire raster into RAM upfront, the current C++ A\* solver allocates ~8 bytes per DEM pixel per `find_path` call. On billion-pixel merged tiles (such as `merged_dem_iran_*.tif` at ~1.57B pixels), a single C++ A\* search can allocate **12+ GiB of RAM** and cause heavy swap usage.
   - Start with the smallest tile (`srtm_43_02.tif`) to verify your setup.
   - Addressing this memory footprint on continental/merged tiles is the primary motivation for the ongoing **Rust CFA\*** rewrite.

---

## Benchmarks & Visualizers

With `C++ · A*` built, you can run the isolated-process pathfinding baseline benchmark from the project root:

```bash
python tests/benchmark_pathfinding_baseline.py --lengths 5 10 20 --dems srtm_43_02.tif
```

Benchmark runs measure DEM load time, `find_path` latency, true per-search peak RSS, nodes explored, and detour ratio, saving `.csv` and `.txt` reports under `tests/results/`.

Interactive GPU terrain/path visualizers (require `pip install fastplotlib`):
- `tests/fastplotlib_dem_viewer.py`
- `tests/cpp_pathfinding_visualise_fatplotlib.py`

---

## Project Layout

```text
src/
|-- paths.py                        Project root and data path helpers
|-- terrain/
|   |-- main.py                     Simulation orchestrator & interactive CLI entrypoint
|   |-- dem_loader.py               Memory-mapped GeoTIFF loader & CoarseDEM downsampler
|   |-- coordinates.py              WGS84 <-> local ENU & earth-scale conversions
|   |-- merge_tiles.py              Multi-tile GeoTIFF merging utility
|   `-- fill_nodata.py              DEM void/nodata interpolation utility
|-- missile/
|   |-- profile.py                  Missile aerodynamic, mass, engine & booster parameters
|   |-- config_store.py             JSON profile validation and persistence
|   |-- state.py                    Kinematic truth/estimated state & FlightStage enum
|   |-- planning/                   Working C++ A* backend (pybind11) & B-spline generator
|   |   `-- cpp/                    CMakeLists.txt and pathfinder.cpp
|   |-- pathfinding/                WIP Rust CFA* backend (PyO3/maturin) & trajectory adapter
|   |   `-- pathfinder_backend/     Cargo crate (lib.rs, astar.rs, cfa.rs)
|   |-- navigation/                 Strapdown INS, GPS, TERCOM, Kalman filter & nav computer
|   |-- guidance/                   Boost, lookahead path follower, route tracker & terminal dive
|   `-- controls/                   PID controller, autopilot, guidance law & flight computer
`-- simulation/
    |-- clock.py                    Real-time simulation pacer
    |-- physics/                    3-DoF dynamics (RK4), aero, ISA atmosphere, engine, booster, wind
    |-- sensors/                    Noisy IMU, GPS receiver, radar & barometric altimeters
    `-- result/                     10 Hz CSV FlightLogger and JSON MissionResult verdicts

frontend/
|-- run.py                          Uvicorn launcher for the Web Control Terminal
|-- requirements.txt                Direct Python dependencies
|-- backend/                        FastAPI endpoints, engine preflight, planner & live runner
`-- web/                            Vanilla ES-module JS/CSS UI (2D map, 3D viewer, PFD, charts)

data/
|-- dem/                            GeoTIFF terrain tiles (*.tif) and memmap caches (*.f32.npy)
|-- missiles/                       Missile configuration profiles (*.json)
|-- logs/                           Recorded flight telemetry time-series (*.csv)
`-- results/                        Saved mission outcome summaries (*.json)

tests/                              Pathfinding baseline benchmarks, results & fastplotlib viewers
```

---

## How the Simulation Loop Works

At each 500 Hz (`dt = 0.002 s`) tick:

1. **Sequencer & Physics (`src/simulation/physics/`)** integrate true 3-DoF point-mass kinematics (RK4) under thrust, drag, gravity, atmospheric density, mean wind, and Dryden turbulence.
2. **Sensors (`src/simulation/sensors/`)** sample true kinematics and DEM elevation with realistic bias, noise, and update rates.
3. **Navigation (`src/missile/navigation/`)** propagates strapdown INS and fuses periodic GPS fixes, barometric/radar altitude, and TERCOM terrain-patch correlations inside a Kalman filter.
4. **Guidance (`src/missile/guidance/`)** generates speed, altitude, and lateral acceleration/heading commands across boost, terrain-following midcourse, and steep terminal dive phases.
5. **Control (`src/missile/controls/`)** runs PID loops in the autopilot to convert guidance targets into bounded plant throttle, climb acceleration, and turn acceleration commands.

Coordinate conventions:
- **Geographic**: WGS84 latitude (deg), longitude (deg), altitude MSL (m).
- **Raster**: DEM `(row, col)` pixel indices via GeoTIFF affine transforms.
- **Local tangent plane**: East-North-Up (**ENU**, metres, `+Up` vertical) for physics, wind, and Kalman filter state vectors.

---

## Scope & Limitations

- The airframe is modeled as a **3-DoF point mass** with bounded lateral/vertical load factors rather than full 6-DoF rigid-body attitude and control-surface deflection aerodynamics.
- The **Rust CFA\* pathfinder** (`src/missile/pathfinding/`) is still being implemented and does not yet return valid paths; rely on **C++ A\*** (`src/missile/planning/`) until CFA\* is completed.
- Sensor, TERCOM, and propulsion models are simplified engineering approximations intended for GNC visualization and software experimentation.

## Credits

The Web Control Terminal frontend was created with Fable 5 and Claude Opus 4.8 High. Built with Python, FastAPI, NumPy, SciPy, Rasterio, pybind11, PyO3, CMake, and SRTM/GeoTIFF elevation data.

## License

No license has been specified for this repository yet.
