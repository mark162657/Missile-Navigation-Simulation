# Web Control Terminal

A browser-based mission-control frontend for the missile-guidance simulator. Three
workspaces — **Planning**, **Mission Control**, and **Final Report** — over a widget
dashboard with dark/light themes, a self-contained tactical map, and a lightweight
line-art 3D viewer.

## Running

The web server uses **FastAPI + uvicorn**; the simulation stack it bridges to needs
the parent project's dependencies (numpy, rasterio, scipy) and — for *live* planning
and simulation — the C++ pathfinder built under `src/missile/planning/cpp/`.

```bash
# from the project root, with the sim's deps already available on your python3
pip install -r frontend/requirements.txt
python3 frontend/run.py            # http://127.0.0.1:8000
python3 frontend/run.py --reload   # dev auto-reload
```

Everything is served from `http://127.0.0.1:8000`. No build step — the UI is plain
ES-module JavaScript and CSS.

## What each screen does

### 1 · Planning
Pick a DEM and a missile profile, drop **launch** and **target** points on the map
(click, or type coordinates), tune the mission parameters, and set the **pathfinding**
heuristic in its own separated section. *Run Pathfinding* calls the real C++ A* + B-spline
backend and streams progress into the console; the route draws on the map. *Proceed to
Mission* arms the plan for live flight.

### 2 · Mission Control
The live monitoring workspace: tactical **map** + third-person **3D viewer** in the
centre, a tabbed **Navigation / Controls / Weather** monitor, a **PFD** (speed / altitude
tapes, attitude, FPA), **deviation charts** (position/altitude error, AGL, speed), and a
prominent **stage banner** with mission time and distance-to-target progress. Plays a
recorded flight (replay) or drives the armed plan live over a WebSocket.

### 3 · Final Report
A **timeline scrubber** replays the whole flight — drag it and the map, 3D posture, and
state readout all follow. Below, the **mission report** card gives the verdict, impact
geometry, deviation stats, a 0–100 success score, and JSON / clipboard export.

## The 3D viewer

Pure 2D-canvas line art (no WebGL): an orbit camera around the missile, a wireframe
terrain patch sampled from the DEM, a line-art Tomahawk oriented by the flight state
(heading + flight-path angle), the planned/flown trajectories with boost/terminal
segments coloured, an AGL drop-line, and the target marker. It redraws only on demand,
so it stays light on the CPU. **Drag** to orbit, **scroll** to zoom; the controls
recenter on the missile or frame the whole mission.

## Architecture

```
frontend/
├── run.py                 launcher (uvicorn)
├── requirements.txt
├── backend/
│   ├── app.py             FastAPI: REST + /ws/live
│   ├── bootstrap.py       puts ../src on sys.path
│   ├── dem_service.py     downsampled DEM grids + point elevation (rasterio)
│   ├── catalog.py         profiles, recorded flights, results
│   ├── frames.py          the unified telemetry frame (replay + live share it)
│   ├── planning.py        real A* + spline route planning
│   └── live_runner.py     drives Simulation.step() without the interactive prompts
└── web/
    ├── index.html         app shell
    ├── css/               tokens.css (theme) + app.css (components)
    └── js/                app · api · widgets · viewer3d · map2d · pfd · charts
                           · player · stagebanner · planning · mission · report
```

### Telemetry model
Both the recorded-log replayer and the live simulation emit the **same frame shape**
(`backend/frames.py`), so the UI has one schema to render. Recorded flights come from
`data/logs/*.csv`; verdicts from `data/results/*.json`, paired by nearest timestamp.

### API
| Method | Path | Purpose |
|---|---|---|
| GET | `/api/dems` | DEM tiles + bounds |
| GET | `/api/dems/{name}/grid` | downsampled elevation grid |
| GET | `/api/dems/{name}/elevation?lat=&lon=` | point elevation |
| GET | `/api/profiles` | missile profiles |
| GET | `/api/missions` | recorded flight summaries |
| GET | `/api/missions/{id}` | full telemetry + verdict |
| GET | `/api/results` | saved verdicts |
| POST | `/api/plan` | run A* + spline, return trajectory |
| WS | `/ws/live` | drive the live simulation, stream frames |

## Notes & limits

- **Live planning / simulation** need the C++ pathfinder built and enough RAM to load
  the chosen DEM (the Iran/Siberia tiles are large). Recorded-flight replay works with
  no C++ and drives every screen, so the UI is fully usable from the shipped data.
- The map is a **self-contained DEM hillshade** (no external tile provider), so it works
  offline and stays visually identical across light/dark — the "map unaffected by theme"
  requirement. A slippy-tile underlay could be layered in later if desired.
- Recorded telemetry logs kinematic state; detailed autopilot/PID signals surface only
  on the live stream, which the Controls tab notes.
```
