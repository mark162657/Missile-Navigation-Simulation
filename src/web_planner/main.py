import multiprocessing as mp
import queue as queue_module
import sys
import threading
import time
import asyncio
from datetime import datetime
from io import BytesIO
from pathlib import Path
from typing import List, Optional
from uuid import uuid4

import numpy as np
import rasterio
from rasterio.enums import Resampling
from fastapi import FastAPI, HTTPException, Response, WebSocket
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

# Add project root to path so we can import src
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.append(str(SRC_ROOT))

from src.terrain.dem_loader import DEMLoader
from src.pathfinder.pathfinding_backend import Pathfinding
from src.pathfinder.trajectory import TrajectoryGenerator
from src.missile.config_store import get_configuration, load_configurations
from missile_guidance.domain import GeoPoint, Mission, MissionPoint, MissileSpec
from missile_guidance.planning import PlannerService
from missile_guidance.simulation import SimulationEngine, SimulationScenario
from missile_guidance.simulation.replay import load_run
from missile_guidance.terrain import dem_coverage, terrain_roughness_std, tercom_suitable

app = FastAPI(title="Missile Guidance Web Planner Pro")

# Frontend lives outside the Python src tree so the app boundary stays clear.
FRONTEND_DIR = PROJECT_ROOT / "frontend" / "web_planner" / "dist"
LEGACY_FRONTEND_DIR = PROJECT_ROOT / "frontend" / "web_planner_legacy"
app.mount("/assets", StaticFiles(directory=str(FRONTEND_DIR / "assets"), check_dir=False), name="assets")
app.mount("/static", StaticFiles(directory=str(LEGACY_FRONTEND_DIR), check_dir=False), name="static")

DEM_DIR = PROJECT_ROOT / "data" / "dem"
MISSIONS_DIR = PROJECT_ROOT / "data" / "missions"
RUNS_DIR = PROJECT_ROOT / "data" / "runs"
TERMINAL_MAX_LINES = 400
PATHFINDING_TIMEOUT_SECONDS = None

# Global state for cache
cache = {
    "dem_name": None,
    "dem_loader": None,
    "pathfinder": None,
    "traj_gen": None,
    "preview_name": None,
    "preview_max_size": None,
    "preview_png": None,
}

terminal_lock = threading.Lock()
terminal_lines: list[dict] = []
terminal_seq = 0
cancelled_runs: set[str] = set()
active_runs: dict[str, dict] = {}
active_runs_lock = threading.Lock()
simulation_results: dict[str, dict] = {}
simulation_results_lock = threading.Lock()

ELEVATION_RAMP = np.array(
    [
        [38, 95, 91],
        [80, 128, 87],
        [147, 150, 88],
        [190, 158, 103],
        [172, 132, 116],
        [143, 147, 153],
        [232, 236, 240],
    ],
    dtype=np.float32,
)


def terminal_log(message: str, run_id: Optional[str] = None, stream: str = "pathfinder") -> None:
    global terminal_seq
    with terminal_lock:
        terminal_seq += 1
        terminal_lines.append(
            {
                "id": terminal_seq,
                "time": datetime.now().strftime("%H:%M:%S"),
                "stream": stream,
                "run_id": run_id,
                "message": message,
            }
        )
        del terminal_lines[:-TERMINAL_MAX_LINES]


def is_cancelled(run_id: Optional[str]) -> bool:
    return bool(run_id and run_id in cancelled_runs)


def raise_if_cancelled(run_id: Optional[str]) -> None:
    if is_cancelled(run_id):
        terminal_log("Abort acknowledged. Pathfinding result discarded.", run_id, "abort")
        raise HTTPException(status_code=499, detail="Pathfinding run aborted.")


def get_resources(dem_name: str, run_id: Optional[str] = None, log_terminal: bool = False):
    if cache["dem_name"] != dem_name:
        dem_path = DEM_DIR / dem_name
        if not dem_path.exists():
            raise HTTPException(status_code=404, detail=f"DEM file {dem_name} not found")

        if log_terminal:
            terminal_log(f"Loading DEM {dem_name}", run_id)
        start_time = time.perf_counter()
        cache["dem_loader"] = DEMLoader(dem_path)
        if log_terminal:
            shape = cache["dem_loader"].shape
            terminal_log(f"DEM loaded: {shape[0]} rows x {shape[1]} cols", run_id)
            terminal_log("Initialising pathfinding_backend.Pathfinding", run_id)
        cache["pathfinder"] = Pathfinding(dem_name)
        cache["traj_gen"] = TrajectoryGenerator(cache["pathfinder"].engine, cache["dem_loader"])
        cache["dem_name"] = dem_name
        if log_terminal:
            terminal_log(f"Pathfinding backend ready in {time.perf_counter() - start_time:.3f}s", run_id)
    elif log_terminal:
        terminal_log(f"Using cached DEM/pathfinder for {dem_name}", run_id)

    return cache["pathfinder"], cache["dem_loader"], cache["traj_gen"]


def _dem_path(dem_name: str) -> Path:
    dem_path = DEM_DIR / dem_name
    if not dem_path.exists() or dem_path.parent != DEM_DIR:
        raise HTTPException(status_code=404, detail=f"DEM file {dem_name} not found")
    return dem_path


def _dem_metadata(dem_name: str) -> dict:
    dem_path = _dem_path(dem_name)
    with rasterio.open(dem_path) as src:
        bounds = src.bounds
        return {
            "name": dem_name,
            "bounds": [bounds.left, bounds.bottom, bounds.right, bounds.top],
            "center": [(bounds.bottom + bounds.top) / 2, (bounds.left + bounds.right) / 2],
            "shape": [src.height, src.width],
            "nodata": src.nodata,
        }


def _read_dem_preview(dem_name: str, max_size: int) -> bytes:
    if (
        cache["preview_name"] == dem_name
        and cache["preview_max_size"] == max_size
        and cache["preview_png"] is not None
    ):
        return cache["preview_png"]

    from PIL import Image

    dem_path = _dem_path(dem_name)
    with rasterio.open(dem_path) as src:
        scale = min(max_size / src.width, max_size / src.height, 1.0)
        out_width = max(1, int(src.width * scale))
        out_height = max(1, int(src.height * scale))
        data = src.read(1, out_shape=(out_height, out_width), resampling=Resampling.bilinear).astype(np.float32)
        nodata = src.nodata

    valid = np.isfinite(data)
    if nodata is not None:
        valid &= data != nodata

    if not np.any(valid):
        raise HTTPException(status_code=422, detail=f"DEM file {dem_name} has no valid elevation samples")

    values = data[valid]
    low, high = np.percentile(values, [2, 98])
    if abs(high - low) < 1e-6:
        low, high = float(values.min()), float(values.max() + 1.0)

    normalized = np.clip((data - low) / max(1e-6, high - low), 0.0, 1.0)
    colorized = _apply_elevation_ramp(normalized)
    shaded = _apply_hillshade(colorized, data, valid)

    rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
    rgba[..., :3] = shaded
    rgba[..., 3] = np.where(valid, 225, 0).astype(np.uint8)

    image = Image.fromarray(rgba, mode="RGBA")
    handle = BytesIO()
    image.save(handle, format="PNG", optimize=True)
    png = handle.getvalue()

    cache["preview_name"] = dem_name
    cache["preview_max_size"] = max_size
    cache["preview_png"] = png
    return png


def _apply_elevation_ramp(normalized: np.ndarray) -> np.ndarray:
    stops = np.linspace(0.0, 1.0, len(ELEVATION_RAMP), dtype=np.float32)
    rgb = np.empty((*normalized.shape, 3), dtype=np.float32)
    for channel in range(3):
        rgb[..., channel] = np.interp(normalized, stops, ELEVATION_RAMP[:, channel])
    return rgb


def _apply_hillshade(rgb: np.ndarray, elevation: np.ndarray, valid: np.ndarray) -> np.ndarray:
    safe_elevation = np.where(valid, elevation, np.nanmedian(elevation[valid]))
    grad_y, grad_x = np.gradient(safe_elevation)
    slope = np.pi / 2.0 - np.arctan(np.hypot(grad_x, grad_y))
    aspect = np.arctan2(-grad_x, grad_y)
    azimuth = np.deg2rad(315.0)
    altitude = np.deg2rad(45.0)
    shade = np.sin(altitude) * np.sin(slope) + np.cos(altitude) * np.cos(slope) * np.cos(azimuth - aspect)
    shade = np.clip((shade + 1.0) / 2.0, 0.0, 1.0)
    shaded = rgb * (0.62 + 0.48 * shade[..., None])
    return np.clip(shaded, 0, 255).astype(np.uint8)


def _worker_log(message_queue, run_id: str, message: str, stream: str = "pathfinder") -> None:
    message_queue.put({"type": "log", "run_id": run_id, "stream": stream, "message": message})


def _pathfinding_worker(payload: dict, message_queue) -> None:
    run_id = payload["run_id"]
    try:
        dem_name = payload["dem_name"]
        dem_path = DEM_DIR / dem_name
        if not dem_path.exists():
            message_queue.put({"type": "error", "status": 404, "detail": f"DEM file {dem_name} not found"})
            return

        _worker_log(message_queue, run_id, f"Worker process booted for {dem_name}")
        start_time = time.perf_counter()
        _worker_log(message_queue, run_id, "Full-resolution DEM pathfinding enforced; display downsampling is not used for backend routing")
        _worker_log(message_queue, run_id, f"Loading DEM {dem_name}")
        dl = DEMLoader(dem_path)
        _worker_log(message_queue, run_id, f"DEM loaded: {dl.shape[0]} rows x {dl.shape[1]} cols")
        _worker_log(message_queue, run_id, "Initialising pathfinding_backend.Pathfinding")
        pf = Pathfinding(dem_name)
        tg = TrajectoryGenerator(pf.engine, dl)
        _worker_log(message_queue, run_id, f"Pathfinding backend ready in {time.perf_counter() - start_time:.3f}s")

        mission_points = [payload["start"]] + payload["waypoints"] + [payload["target"]]
        full_trajectory = []

        for i in range(len(mission_points) - 1):
            p1 = mission_points[i]
            p2 = mission_points[i + 1]
            _worker_log(
                message_queue,
                run_id,
                f"Leg {i + 1}/{len(mission_points) - 1}: ({p1['lat']:.6f}, {p1['lon']:.6f}) -> ({p2['lat']:.6f}, {p2['lon']:.6f})",
            )

            s_pixel = tuple(int(value) for value in dl.lat_lon_to_pixel(p1["lat"], p1["lon"]))
            t_pixel = tuple(int(value) for value in dl.lat_lon_to_pixel(p2["lat"], p2["lon"]))
            _worker_log(message_queue, run_id, f"Pixel start={s_pixel}, target={t_pixel}; calling pathfinding_backend.find_path()")

            leg_start = time.perf_counter()
            raw_path = pf.find_path(s_pixel, t_pixel, payload["heuristic_weight"])
            leg_duration = time.perf_counter() - leg_start

            if not raw_path:
                _worker_log(message_queue, run_id, f"Path not found (C++ returned empty list). Time: {leg_duration:.4f}s", "error")
                message_queue.put({"type": "error", "status": 404, "detail": f"Path not found for leg {i + 1}"})
                return

            _worker_log(message_queue, run_id, f"Path found! Length: {len(raw_path)}. Time: {leg_duration:.4f}s")
            _worker_log(message_queue, run_id, "Smoothing trajectory and sampling terrain elevation")
            leg_trajectory = tg.get_trajectory(raw_path, min_alt=payload["min_altitude"])
            _worker_log(message_queue, run_id, f"Trajectory generator returned {len(leg_trajectory)} GPS/altitude samples")

            if full_trajectory:
                full_trajectory.extend(leg_trajectory[1:])
            else:
                full_trajectory.extend(leg_trajectory)

            _worker_log(message_queue, run_id, f"Leg {i + 1} complete; accumulated samples={len(full_trajectory)}")

        serialized_path = [(float(p[0]), float(p[1]), float(p[2])) for p in full_trajectory]
        _worker_log(message_queue, run_id, f"Pathfinding complete. Final trajectory samples={len(serialized_path)}")
        message_queue.put(
            {
                "type": "result",
                "payload": {
                    "run_id": run_id,
                    "path": serialized_path,
                    "bounds": [float(dl.bounds.left), float(dl.bounds.bottom), float(dl.bounds.right), float(dl.bounds.top)],
                },
            }
        )
    except Exception as exc:
        message_queue.put({"type": "error", "status": 500, "detail": str(exc)})


def _cleanup_run(run_id: str) -> None:
    with active_runs_lock:
        active_runs.pop(run_id, None)


def _terminate_run(run_id: str, reason: str) -> None:
    with active_runs_lock:
        active = active_runs.get(run_id)
    if active:
        process = active["process"]
        if process.is_alive():
            process.terminate()
            process.join(timeout=2)
            if process.is_alive():
                process.kill()
                process.join(timeout=2)
        terminal_log(reason, run_id, "abort")
    _cleanup_run(run_id)

class Point(BaseModel):
    lat: float
    lon: float
    alt: float = 0.0

class PathRequest(BaseModel):
    run_id: Optional[str] = None
    dem_name: str
    start: Point
    target: Point
    waypoints: List[Point] = []
    heuristic_weight: float = 1.5
    min_altitude: float = 30.0
    mode: str = "full"

class MissionPointRequest(BaseModel):
    name: str = ""
    lat: float
    lon: float
    alt: float = 0.0

class MissionRequest(BaseModel):
    mission_id: Optional[str] = None
    name: str
    start: MissionPointRequest
    target: MissionPointRequest
    missile_name: str
    waypoints: List[MissionPointRequest] = []
    dem_name: Optional[str] = None

class RunRequest(BaseModel):
    mission: Optional[MissionRequest] = None
    mission_id: Optional[str] = None
    dt: float = 0.25
    max_time_s: float = 120.0


def _mission_point_from_request(point: MissionPointRequest, fallback_name: str) -> MissionPoint:
    return MissionPoint(
        name=point.name or fallback_name,
        location=GeoPoint(point.lat, point.lon, point.alt),
    )


def _mission_from_request(request: MissionRequest) -> Mission:
    return Mission(
        name=request.name,
        start=_mission_point_from_request(request.start, "Launch"),
        target=_mission_point_from_request(request.target, "Target"),
        missile_name=request.missile_name,
        waypoints=[
            _mission_point_from_request(point, f"Waypoint {index + 1}")
            for index, point in enumerate(request.waypoints)
        ],
        dem_name=request.dem_name,
        mission_id=request.mission_id or str(uuid4()),
    )


def _mission_file(mission_id: str) -> Path:
    return MISSIONS_DIR / f"{mission_id}.json"


def _load_mission(mission_id: str) -> Mission:
    path = _mission_file(mission_id)
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
    return Mission.load(path)

@app.get("/")
async def read_index():
    from fastapi.responses import FileResponse
    # If the built index.html exists, serve it
    index_path = FRONTEND_DIR / "index.html"
    if index_path.exists():
        return FileResponse(index_path)
    
    # Fallback to old static if dist doesn't exist yet (for dev)
    old_static = LEGACY_FRONTEND_DIR / "index.html"
    return FileResponse(old_static)

@app.get("/api/dems")
async def list_dems():
    tifs = sorted([p.name for p in DEM_DIR.glob("*.tif")])
    return {"dems": tifs}

@app.get("/api/missile-configs")
async def list_configs():
    return {"configs": load_configurations()}

@app.get("/api/missions")
async def list_missions():
    MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
    missions = []
    for path in sorted(MISSIONS_DIR.glob("*.json")):
        try:
            mission = Mission.load(path)
            missions.append(
                {
                    "mission_id": mission.mission_id,
                    "name": mission.name,
                    "missile_name": mission.missile_name,
                    "dem_name": mission.dem_name,
                    "status": mission.status.value,
                    "updated_at": mission.updated_at,
                }
            )
        except Exception as exc:
            missions.append({"mission_id": path.stem, "error": str(exc)})
    return {"missions": missions}

@app.post("/api/missions")
async def save_mission(request: MissionRequest):
    mission = _mission_from_request(request)
    path = mission.save(MISSIONS_DIR)
    return {"mission": mission.to_dict(), "path": str(path)}

@app.get("/api/missions/{mission_id}")
async def get_mission(mission_id: str):
    return {"mission": _load_mission(mission_id).to_dict()}

@app.get("/api/terrain/coverage/{dem_name}")
async def get_terrain_coverage(dem_name: str):
    return dem_coverage(_dem_path(dem_name))

@app.get("/api/terrain/roughness/{dem_name}")
async def get_terrain_roughness(dem_name: str, lat: float, lon: float, patch_size: int = 25, threshold_m: float = 5.0):
    dem = DEMLoader(_dem_path(dem_name))
    patch = dem.get_elevation_patch(lat, lon, patch_size, normalized=False)
    if patch is None:
        raise HTTPException(status_code=404, detail="Terrain patch is outside DEM coverage")
    roughness = terrain_roughness_std(patch)
    return {
        "roughness_std_m": roughness,
        "tercom_suitable": tercom_suitable(patch, threshold_m),
        "threshold_m": threshold_m,
    }

@app.post("/api/plan")
def plan_with_service(request: PathRequest):
    mission = Mission(
        name=request.run_id or "ad-hoc-plan",
        start=MissionPoint("Launch", GeoPoint(request.start.lat, request.start.lon, request.start.alt)),
        target=MissionPoint("Target", GeoPoint(request.target.lat, request.target.lon, request.target.alt)),
        missile_name="",
        waypoints=[
            MissionPoint(f"Waypoint {index + 1}", GeoPoint(point.lat, point.lon, point.alt))
            for index, point in enumerate(request.waypoints)
        ],
        dem_name=request.dem_name,
    )
    result = PlannerService(request.dem_name).plan_mission(
        mission,
        heuristic_weight=request.heuristic_weight,
        min_altitude=request.min_altitude,
    )
    return {
        "run_id": request.run_id,
        "path": result.path,
        "bounds": result.bounds,
    }

@app.get("/api/pathfinding-terminal")
async def get_pathfinding_terminal(after: int = 0):
    with terminal_lock:
        return {"logs": [line for line in terminal_lines if line["id"] > after]}

@app.post("/api/cancel-pathfinding/{run_id}")
async def cancel_pathfinding(run_id: str):
    cancelled_runs.add(run_id)
    _terminate_run(run_id, "Abort requested from web UI. Worker process terminated.")
    return {"cancelled": True, "run_id": run_id}

@app.get("/api/dem-info/{dem_name}")
async def get_dem_info(dem_name: str):
    return _dem_metadata(dem_name)

@app.get("/api/dem-preview/{dem_name}")
async def get_dem_preview(dem_name: str, max_size: int = 1400):
    max_size = max(256, min(max_size, 2200))
    png = _read_dem_preview(dem_name, max_size)
    return Response(
        content=png,
        media_type="image/png",
        headers={"Cache-Control": "public, max-age=3600"},
    )

@app.post("/api/plan-path")
def plan_path(request: PathRequest):
    run_id = request.run_id or f"run-{int(time.time() * 1000)}"
    cancelled_runs.discard(run_id)
    terminal_log("---- pathfinding run started ----", run_id)
    terminal_log(
        f"DEM={request.dem_name}, waypoints={len(request.waypoints)}, heuristic={request.heuristic_weight:.2f}, min_altitude={request.min_altitude:.1f}m",
        run_id,
    )

    payload = {
        "run_id": run_id,
        "dem_name": request.dem_name,
        "start": request.start.model_dump(),
        "target": request.target.model_dump(),
        "waypoints": [point.model_dump() for point in request.waypoints],
        "heuristic_weight": request.heuristic_weight,
        "min_altitude": request.min_altitude,
    }

    context = mp.get_context("spawn")
    message_queue = context.Queue()
    process = context.Process(target=_pathfinding_worker, args=(payload, message_queue), daemon=True)

    with active_runs_lock:
        active_runs[run_id] = {"process": process, "queue": message_queue}

    process.start()
    terminal_log(f"Worker process started with pid={process.pid}", run_id)

    start_time = time.perf_counter()
    last_heartbeat = start_time

    try:
        while True:
            try:
                event = message_queue.get(timeout=0.25)
            except queue_module.Empty:
                event = None

            if event:
                if event["type"] == "log":
                    terminal_log(event["message"], event.get("run_id", run_id), event.get("stream", "pathfinder"))
                elif event["type"] == "result":
                    process.join(timeout=2)
                    _cleanup_run(run_id)
                    terminal_log("---- pathfinding run finished ----", run_id)
                    return event["payload"]
                elif event["type"] == "error":
                    process.join(timeout=2)
                    _cleanup_run(run_id)
                    terminal_log(f"Pathfinding error: {event['detail']}", run_id, "error")
                    raise HTTPException(status_code=event.get("status", 500), detail=event["detail"])

            if is_cancelled(run_id):
                _terminate_run(run_id, "Abort acknowledged. Worker process terminated.")
                raise HTTPException(status_code=499, detail="Pathfinding run aborted.")

            if not process.is_alive():
                while True:
                    try:
                        event = message_queue.get_nowait()
                    except queue_module.Empty:
                        break
                    if event["type"] == "log":
                        terminal_log(event["message"], event.get("run_id", run_id), event.get("stream", "pathfinder"))
                    elif event["type"] == "result":
                        _cleanup_run(run_id)
                        terminal_log("---- pathfinding run finished ----", run_id)
                        return event["payload"]
                    elif event["type"] == "error":
                        _cleanup_run(run_id)
                        terminal_log(f"Pathfinding error: {event['detail']}", run_id, "error")
                        raise HTTPException(status_code=event.get("status", 500), detail=event["detail"])

                exit_code = process.exitcode
                _cleanup_run(run_id)
                terminal_log(f"Worker process exited unexpectedly with code {exit_code}", run_id, "error")
                raise HTTPException(status_code=500, detail=f"Pathfinding worker exited unexpectedly with code {exit_code}")

            now = time.perf_counter()
            if now - last_heartbeat >= 3:
                terminal_log(f"Worker still running... elapsed {now - start_time:.1f}s", run_id)
                last_heartbeat = now

            if PATHFINDING_TIMEOUT_SECONDS is not None and now - start_time >= PATHFINDING_TIMEOUT_SECONDS:
                _terminate_run(run_id, f"Pathfinding timed out after {PATHFINDING_TIMEOUT_SECONDS}s. Worker process terminated.")
                raise HTTPException(status_code=504, detail=f"Pathfinding timed out after {PATHFINDING_TIMEOUT_SECONDS}s.")
    finally:
        if not process.is_alive():
            _cleanup_run(run_id)
        elif is_cancelled(run_id):
            _terminate_run(run_id, "Abort acknowledged. Worker process terminated.")

@app.get("/api/runs")
async def list_simulation_runs():
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    runs = [
        {
            "run_id": path.stem,
            "path": str(path),
            "updated_at": path.stat().st_mtime,
        }
        for path in sorted(RUNS_DIR.glob("*.jsonl"))
    ]
    return {"runs": runs}

@app.post("/api/runs")
def launch_simulation_run(request: RunRequest):
    if request.mission is not None:
        mission = _mission_from_request(request.mission)
    elif request.mission_id is not None:
        mission = _load_mission(request.mission_id)
    else:
        raise HTTPException(status_code=400, detail="Provide either mission or mission_id")

    config = get_configuration(mission.missile_name)
    if config is None:
        raise HTTPException(status_code=404, detail=f"Missile configuration {mission.missile_name} not found")

    scenario = SimulationScenario(
        mission=mission,
        missile_spec=MissileSpec.from_mapping(config),
        dt=max(0.01, request.dt),
        max_time_s=max(0.01, request.max_time_s),
    )
    result = SimulationEngine(scenario, runs_dir=RUNS_DIR).run()
    payload = {
        "run_id": result.run_id,
        "mission_id": mission.mission_id,
        "samples": len(result.telemetry),
        "replay_path": str(result.replay_path) if result.replay_path is not None else None,
        "latest": result.telemetry[-1].to_dict() if result.telemetry else None,
    }
    with simulation_results_lock:
        simulation_results[result.run_id] = {
            "summary": payload,
            "telemetry": [sample.to_dict() for sample in result.telemetry],
        }
    return payload

@app.get("/api/runs/{run_id}")
async def get_simulation_run(run_id: str):
    with simulation_results_lock:
        cached = simulation_results.get(run_id)
    if cached is not None:
        return cached

    path = RUNS_DIR / f"{run_id}.jsonl"
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Run {run_id} not found")
    telemetry = load_run(path)
    return {
        "summary": {"run_id": run_id, "samples": len(telemetry), "replay_path": str(path)},
        "telemetry": telemetry,
    }

@app.websocket("/api/runs/{run_id}/stream")
async def stream_simulation_run(websocket: WebSocket, run_id: str):
    await websocket.accept()
    try:
        payload = await get_simulation_run(run_id)
        for sample in payload["telemetry"]:
            await websocket.send_json(sample)
            await asyncio.sleep(0.05)
        await websocket.close()
    except Exception as exc:
        await websocket.send_json({"error": str(exc)})
        await websocket.close()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
