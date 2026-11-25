"""Mission planning: run either the Rust CFA* or legacy C++ A* pathfinder
followed by trajectory generation.

Returns the planned trajectory as geographic points the UI can draw on the map
and feed to the 3D viewer, plus a structured log for the planning console.
Heavy native/NumPy work runs in a worker thread (see app.py) so it never blocks
the FastAPI event loop.
"""
from __future__ import annotations

import inspect
import time

import numpy as np

from . import bootstrap  # noqa: F401 - side effect: puts src/ on sys.path
from .native_backend import DEFAULT_ENGINE, require_pathfinder_backend


class _DemTerrainProfileAdapter:
    """Fallback terrain-profile sampler matching the C++ engine's interface.

    `src/missile/pathfinding/trajectory.py` currently mirrors the C++
    TrajectoryGenerator and calls `self.engine.get_terrain_profile(rows, cols)`.
    Until trajectory.py is updated for the Rust backend (or Pathfinding exposes
    `.engine`), this adapter satisfies `get_terrain_profile` directly against
    `pf.dem` using the same nearest-pixel clamped lookup as C++.
    """

    def __init__(self, dem: np.ndarray) -> None:
        self._dem = dem

    def get_terrain_profile(self, row_path, col_path) -> np.ndarray:
        rows = np.rint(np.asarray(row_path, dtype=np.float64)).astype(np.int64)
        cols = np.rint(np.asarray(col_path, dtype=np.float64)).astype(np.int64)
        max_r, max_c = self._dem.shape[0] - 1, self._dem.shape[1] - 1
        rows = np.clip(rows, 0, max_r)
        cols = np.clip(cols, 0, max_c)
        return np.asarray(self._dem[rows, cols], dtype=np.float32)


def _call_rust_find_path(
    pf,
    start_rc: tuple[int, int],
    target_rc: tuple[int, int],
    *,
    downsample_k: int,
    heuristic_weight: float,
):
    """Invoke Rust `Pathfinding.find_path`, forwarding CFA* params if accepted."""
    sig = inspect.signature(pf.find_path)
    params = sig.parameters
    accepts_var_kw = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values())

    kwargs: dict = {}
    if "downsample_k" in params or accepts_var_kw:
        kwargs["downsample_k"] = downsample_k
    elif "k" in params:
        kwargs["k"] = downsample_k

    if "heuristic_weight" in params or accepts_var_kw:
        kwargs["heuristic_weight"] = heuristic_weight

    return pf.find_path(start_rc, target_rc, **kwargs)


def _build_trajectory_generator(gen_cls, pf):
    """Instantiate TrajectoryGenerator whether it expects (engine, dem) or (dem,)."""
    sig = inspect.signature(gen_cls.__init__)
    positional = [
        p for p in list(sig.parameters.values())[1:]  # skip self
        if p.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
    ]
    if len(positional) == 1:
        return gen_cls(pf.dem_loader)

    engine_ref = getattr(pf, "engine", None)
    if engine_ref is None:
        engine_ref = pf if hasattr(pf, "get_terrain_profile") else _DemTerrainProfileAdapter(pf.dem)
    return gen_cls(engine_ref, pf.dem_loader)


def run_plan(
    dem_name: str,
    start_gps,
    target_gps,
    heuristic_weight: float = 2.0,
    engine: str = DEFAULT_ENGINE,
    downsample_k: int = 4,
) -> dict:
    """Plan start -> target over a DEM using the selected engine ('rust' or 'cpp')."""
    engine_key = require_pathfinder_backend(engine)

    if engine_key == "rust":
        k = int(downsample_k)
        if k < 1:
            raise ValueError("CFA* downsample factor k must be at least 1.")
        return _run_rust_plan(dem_name, start_gps, target_gps, heuristic_weight, k)

    return _run_cpp_plan(dem_name, start_gps, target_gps, heuristic_weight)


def _run_rust_plan(
    dem_name: str,
    start_gps,
    target_gps,
    heuristic_weight: float,
    downsample_k: int,
) -> dict:
    from missile.pathfinding.pathfinding_backend import Pathfinding
    from missile.pathfinding.trajectory import TrajectoryGenerator

    log: list[str] = []
    t0 = time.time()

    pf = Pathfinding(dem_name)
    coarse_r = max(1, pf.rows // downsample_k)
    coarse_c = max(1, pf.cols // downsample_k)
    log.append(f"[dem] {dem_name} loaded ({pf.rows:,}×{pf.cols:,} px)")

    start_rc = tuple(int(v) for v in pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1]))
    target_rc = tuple(int(v) for v in pf.dem_loader.lat_lon_to_pixel(target_gps[0], target_gps[1]))
    log.append(
        f"[cfa*] Rust CFA* · k={downsample_k} (coarse ~{coarse_r:,}×{coarse_c:,}) "
        f"· w={heuristic_weight:.1f}"
    )
    log.append(f"[cfa*] launch px {start_rc} → target px {target_rc}")

    t_search = time.time()
    pixel_path = _call_rust_find_path(
        pf,
        start_rc,
        target_rc,
        downsample_k=downsample_k,
        heuristic_weight=heuristic_weight,
    )
    if not pixel_path:
        raise RuntimeError("Rust CFA* pathfinding failed: no route start -> target.")
    log.append(f"[cfa*] path found: {len(pixel_path):,} cells in {time.time() - t_search:.2f}s")

    t_traj = time.time()
    gen = _build_trajectory_generator(TrajectoryGenerator, pf)
    traj = gen.get_trajectory(pixel_path)  # (N, 3) [lat, lon, ground_elev]
    if traj is None or len(traj) == 0:
        raise RuntimeError("Trajectory generation returned no waypoints.")
    log.append(f"[traj] generated {len(traj):,} waypoints in {time.time() - t_traj:.2f}s")
    log.append(f"[done] Rust CFA* total {time.time() - t0:.2f}s")

    points = [[float(p[0]), float(p[1]), float(p[2])] for p in traj]
    return {
        "dem": dem_name,
        "engine": "rust",
        "algorithm": "CFA*",
        "start_gps": list(map(float, start_gps)),
        "target_gps": list(map(float, target_gps)),
        "heuristic_weight": heuristic_weight,
        "downsample_k": downsample_k,
        "pixel_cells": len(pixel_path),
        "waypoints": len(points),
        "trajectory": points,
        "log": log,
        "elapsed_s": round(time.time() - t0, 3),
    }


def _run_cpp_plan(
    dem_name: str,
    start_gps,
    target_gps,
    heuristic_weight: float,
) -> dict:
    from missile.planning.pathfinding_backend import Pathfinding
    from missile.planning.trajectory import TrajectoryGenerator

    log: list[str] = []
    t0 = time.time()

    pf = Pathfinding(dem_name)
    if pf.engine is None:  # Defensive: direct binary import above should catch this.
        raise RuntimeError("C++ pathfinding engine failed to initialise.")
    log.append(f"[dem] {dem_name} loaded ({pf.rows:,}×{pf.cols:,} px)")

    start_rc = tuple(int(v) for v in pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1]))
    target_rc = tuple(int(v) for v in pf.dem_loader.lat_lon_to_pixel(target_gps[0], target_gps[1]))
    log.append(f"[a*] C++ A* · w={heuristic_weight:.1f}")
    log.append(f"[a*] launch px {start_rc} → target px {target_rc}")

    t_search = time.time()
    pixel_path = pf.find_path(start_rc, target_rc, heuristic_weight=heuristic_weight)
    if not pixel_path:
        raise RuntimeError("C++ A* pathfinding failed: no route start -> target.")
    log.append(f"[a*] path found: {len(pixel_path):,} cells in {time.time() - t_search:.2f}s")

    t_traj = time.time()
    gen = TrajectoryGenerator(pf.engine, pf.dem_loader)
    traj = gen.get_trajectory(pixel_path)  # (N, 3) [lat, lon, ground_elev]
    if traj is None or len(traj) == 0:
        raise RuntimeError("Trajectory generation returned no waypoints.")
    log.append(f"[traj] generated {len(traj):,} waypoints in {time.time() - t_traj:.2f}s")
    log.append(f"[done] C++ A* total {time.time() - t0:.2f}s")

    points = [[float(p[0]), float(p[1]), float(p[2])] for p in traj]
    return {
        "dem": dem_name,
        "engine": "cpp",
        "algorithm": "A*",
        "start_gps": list(map(float, start_gps)),
        "target_gps": list(map(float, target_gps)),
        "heuristic_weight": heuristic_weight,
        "downsample_k": None,
        "pixel_cells": len(pixel_path),
        "waypoints": len(points),
        "trajectory": points,
        "log": log,
        "elapsed_s": round(time.time() - t0, 3),
    }
