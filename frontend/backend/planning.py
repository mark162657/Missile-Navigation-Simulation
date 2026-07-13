"""Mission planning: run the real C++ A* pathfinder + B-spline smoothing.

Returns the planned trajectory as geographic points the UI can draw on the map
and feed to the 3D viewer, plus a small log of what happened for the planning
terminal. The heavy work runs in a worker thread (see app.py) so it never blocks
the async event loop.
"""
from __future__ import annotations

import time

from . import bootstrap  # noqa: F401 - side effect: puts src/ on sys.path


def run_plan(dem_name: str, start_gps, target_gps, heuristic_weight: float = 2.0) -> dict:
    """Plan start -> target over a DEM. Raises on failure; caller maps to HTTP."""
    from missile.planning.pathfinding_backend import Pathfinding
    from missile.planning.trajectory import TrajectoryGenerator

    log: list[str] = []
    t0 = time.time()

    pf = Pathfinding(dem_name)
    if pf.engine is None:
        raise RuntimeError(
            "C++ pathfinding engine not built. Build src/missile/planning/cpp "
            "(CMake) before planning live routes."
        )
    log.append(f"[dem] {dem_name} loaded ({pf.rows}x{pf.cols} px)")

    start_rc = tuple(pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1]))
    target_rc = tuple(pf.dem_loader.lat_lon_to_pixel(target_gps[0], target_gps[1]))
    log.append(f"[a*] launch px {start_rc} -> target px {target_rc}  (w={heuristic_weight})")

    t_search = time.time()
    pixel_path = pf.find_path(start_rc, target_rc, heuristic_weight=heuristic_weight)
    if not pixel_path:
        raise RuntimeError("Pathfinding failed: no route start -> target.")
    log.append(f"[a*] path found: {len(pixel_path):,} cells in {time.time() - t_search:.2f}s")

    t_smooth = time.time()
    gen = TrajectoryGenerator(pf.engine, pf.dem_loader)
    traj = gen.get_trajectory(pixel_path)  # (N, 3) [lat, lon, ground_elev]
    log.append(f"[spline] smoothed to {len(traj):,} waypoints in {time.time() - t_smooth:.2f}s")
    log.append(f"[done] total {time.time() - t0:.2f}s")

    points = [[float(p[0]), float(p[1]), float(p[2])] for p in traj]
    return {
        "dem": dem_name,
        "start_gps": list(map(float, start_gps)),
        "target_gps": list(map(float, target_gps)),
        "heuristic_weight": heuristic_weight,
        "pixel_cells": len(pixel_path),
        "waypoints": len(points),
        "trajectory": points,
        "log": log,
        "elapsed_s": round(time.time() - t0, 3),
    }
