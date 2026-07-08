"""
Load test: run a ~600 km A* path over the Siberia DEM, smooth it into the
trajectory the PathFollower will consume, then estimate whether a per-tick
PathFollower could chew through that many points in real time.

PathFollower is NOT built yet -- this only sizes the problem so we know what
search strategy it needs (naive O(N)/tick vs. a tracked local window).

Run directly:  python tests/test_pathfollower_load.py
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from missile.planning.pathfinding_backend import Pathfinding
from missile.planning.trajectory import TrajectoryGenerator

DEM_NAME = "merged_dem_sib_N54_N59_E090_E100.tif"

# ~600 km, mostly east-west at lat 57 (cos57 -> ~60.6 km/deg lon), in-bounds.
START_GPS = (57.0, 90.3)
END_GPS = (57.0, 100.2)

# Flight assumptions for the per-tick estimate.
CRUISE_SPEED_MS = 247.0           # ~890 km/h, Tomahawk-class cruise
CONTROL_RATES_HZ = (50.0, 100.0, 200.0)  # candidate control-loop rates


def haversine_km(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Great-circle distance between (lat, lon) points, in km."""
    r = 6371.0
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def main() -> None:
    gc_km = haversine_km(START_GPS, END_GPS)
    print("=" * 64)
    print(f"Siberia DEM load test  |  great-circle {gc_km:.1f} km")
    print(f"  start {START_GPS}  ->  end {END_GPS}")
    print("=" * 64)

    print("\n[1/3] Booting C++ engine (loads full DEM)...")
    t0 = time.time()
    pf = Pathfinding(DEM_NAME)
    print(f"      engine ready in {time.time() - t0:.2f}s")

    sr, sc = pf.dem_loader.lat_lon_to_pixel(*START_GPS)
    er, ec = pf.dem_loader.lat_lon_to_pixel(*END_GPS)
    print(f"      pixels: ({sr},{sc}) -> ({er},{ec})  on {pf.rows}x{pf.cols} grid")

    print("\n[2/3] Running A*...")
    t0 = time.time()
    raw_path = pf.find_path((sr, sc), (er, ec), heuristic_weight=1)
    astar_s = time.time() - t0
    if not raw_path:
        print("      ! no path found -- aborting")
        return
    n_raw = len(raw_path)
    print(f"      A* done in {astar_s:.2f}s  |  raw path points: {n_raw:,}")

    print("\n[3/3] Smoothing into 3D trajectory (B-spline, res_multi=5)...")
    t0 = time.time()
    traj = TrajectoryGenerator(pf.engine, pf.dem_loader)
    trajectory = traj.get_trajectory(raw_path)
    smooth_s = time.time() - t0
    n_pts = len(trajectory)
    print(f"      smoothing done in {smooth_s:.2f}s  |  trajectory points: {n_pts:,}")

    # ---- PathFollower cost estimate ----
    flight_s = (gc_km * 1000.0) / CRUISE_SPEED_MS
    spacing_m = (gc_km * 1000.0) / max(n_pts, 1)
    print("\n" + "=" * 64)
    print("PathFollower cost estimate")
    print("=" * 64)
    print(f"  trajectory points : {n_pts:,}")
    print(f"  point spacing     : {spacing_m:.1f} m between samples")
    print(f"  flight time       : {flight_s:.0f} s ({flight_s/60:.1f} min) @ {CRUISE_SPEED_MS:.0f} m/s")
    print()
    print(f"  {'rate':>6} | {'ticks':>12} | {'naive O(N)/tick ops':>22} | {'tracked O(1)/tick ops':>22}")
    print("  " + "-" * 70)
    for hz in CONTROL_RATES_HZ:
        ticks = int(flight_s * hz)
        naive = ticks * n_pts        # re-scan whole trajectory every tick
        tracked = ticks * 8          # local window of ~8 points around last index
        print(f"  {hz:>5.0f}H | {ticks:>12,} | {naive:>22,} | {tracked:>22,}")
    print()
    print("  Rule of thumb: ~1e8 simple ops/sec/core in Python.")
    print("  naive total = the top-right number; divide by 1e8 for seconds of CPU.")


if __name__ == "__main__":
    main()
