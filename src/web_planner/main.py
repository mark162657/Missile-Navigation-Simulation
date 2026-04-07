import os
import sys
import json
from pathlib import Path
from typing import List, Tuple, Optional, Dict

from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

# Add project root to path so we can import src
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))

from src.terrain.dem_loader import DEMLoader
from src.pathfinder.pathfinding_backend import Pathfinding
from src.pathfinder.trajectory import TrajectoryGenerator
from src.missile.config_store import load_configurations, get_configuration

app = FastAPI(title="Missile Guidance Web Planner Pro")

# Path to static files
STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

DEM_DIR = PROJECT_ROOT / "data" / "dem"

# Global state for cache
cache = {
    "dem_name": None,
    "dem_loader": None,
    "pathfinder": None,
    "traj_gen": None
}

def get_resources(dem_name: str):
    if cache["dem_name"] != dem_name:
        dem_path = DEM_DIR / dem_name
        if not dem_path.exists():
            raise HTTPException(status_code=404, detail=f"DEM file {dem_name} not found")
        
        cache["dem_loader"] = DEMLoader(dem_path)
        cache["pathfinder"] = Pathfinding(dem_name)
        cache["traj_gen"] = TrajectoryGenerator(cache["pathfinder"].engine, cache["dem_loader"])
        cache["dem_name"] = dem_name
        
    return cache["pathfinder"], cache["dem_loader"], cache["traj_gen"]

class Point(BaseModel):
    lat: float
    lon: float

class PathRequest(BaseModel):
    dem_name: str
    start: Point
    target: Point
    waypoints: List[Point] = []
    heuristic_weight: float = 1.5

@app.get("/")
async def read_index():
    from fastapi.responses import FileResponse
    return FileResponse(STATIC_DIR / "index.html")

@app.get("/api/dems")
async def list_dems():
    tifs = sorted([p.name for p in DEM_DIR.glob("*.tif")])
    return {"dems": tifs}

@app.get("/api/missile-configs")
async def list_configs():
    return {"configs": load_configurations()}

@app.get("/api/dem-info/{dem_name}")
async def get_dem_info(dem_name: str):
    _, dl, _ = get_resources(dem_name)
    return {
        "name": dem_name,
        "bounds": [dl.bounds.left, dl.bounds.bottom, dl.bounds.right, dl.bounds.top],
        "center": [(dl.bounds.bottom + dl.bounds.top) / 2, (dl.bounds.left + dl.bounds.right) / 2]
    }

@app.post("/api/plan-path")
async def plan_path(request: PathRequest):
    pf, dl, tg = get_resources(request.dem_name)
    
    try:
        mission_points = [request.start] + request.waypoints + [request.target]
        full_trajectory = []
        
        for i in range(len(mission_points) - 1):
            p1 = mission_points[i]
            p2 = mission_points[i+1]
            
            s_pixel = dl.lat_lon_to_pixel(p1.lat, p1.lon)
            t_pixel = dl.lat_lon_to_pixel(p2.lat, p2.lon)
            
            raw_path = pf.find_path(s_pixel, t_pixel, request.heuristic_weight)
            if not raw_path:
                raise HTTPException(status_code=404, detail=f"Path not found for leg {i+1}")
            
            leg_trajectory = tg.get_trajectory(raw_path)
            
            if full_trajectory:
                # Avoid duplicating the junction point
                full_trajectory.extend(leg_trajectory[1:])
            else:
                full_trajectory.extend(leg_trajectory)
                
        # Convert all coordinates to standard Python floats to avoid NumPy serialization issues
        serialized_path = [
            (float(p[0]), float(p[1]), float(p[2])) for p in full_trajectory
        ]
                
        return {
            "path": serialized_path,
            "bounds": [float(dl.bounds.left), float(dl.bounds.bottom), float(dl.bounds.right), float(dl.bounds.top)]
        }
    except Exception as e:
        if isinstance(e, HTTPException): raise e
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
