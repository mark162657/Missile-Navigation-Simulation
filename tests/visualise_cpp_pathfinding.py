"""
Standalone 2D Fastplotlib visualizer for C++ A* pathfinding on realistic shaded-relief DEM terrain.
"""

import sys
from pathlib import Path

import fastplotlib as fpl
import numpy as np

TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

from compare_pathfinding import (
    add_subplot_scene,
    build_metric_lookups,
    choose_dem,
    choose_scenario_gps,
    compute_viewport,
    render_realistic_terrain_rgb,
    run_cpp_single,
)
from missile.planning.pathfinding_backend import Pathfinding as CppPathfinding


def main() -> None:
    tif_path = choose_dem()
    cpp_pf = CppPathfinding(tif_path.name)
    dem_loader = cpp_pf.dem_loader
    mx_lookup, my_lookup = build_metric_lookups(dem_loader)

    start_gps, end_gps = choose_scenario_gps(dem_loader)
    raw_weights = input("\nHeuristic weights comma-separated (default 2.0): ").strip()
    weights = [float(w.strip()) for w in raw_weights.split(",") if w.strip()] if raw_weights else [2.0]

    sr, sc = dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1])
    er, ec = dem_loader.lat_lon_to_pixel(end_gps[0], end_gps[1])
    start_pixel = (int(sr), int(sc))
    end_pixel = (int(er), int(ec))

    results = []
    for w in weights:
        print(f"\n--- Running C++ A* (w={w}) ---")
        res = run_cpp_single(cpp_pf, start_pixel, end_pixel, w)
        if res:
            print(
                f"[OK] {res['points']} pts | {res['distance_km']:.2f} km | "
                f"climb {res['climb_m']:.0f} m | max |dz| {res['max_dz_m']:.1f} m | {res['time_s']:.3f}s"
            )
            results.append(res)
        else:
            print("[ERROR] No path found.")

    if not results:
        return

    min_row, max_row, min_col, max_col, downsample = compute_viewport(
        dem_loader.shape, start_pixel, end_pixel, [r["path_3d"] for r in results], dem_loader
    )
    dem_region = cpp_pf.dem[min_row:max_row:downsample, min_col:max_col:downsample].astype(np.float32)
    dem_region[dem_region <= -100.0] = np.nan

    mid_row = (min_row + max_row) // 2
    terrain_rgb = render_realistic_terrain_rgb(
        dem_region,
        dx_m=float(mx_lookup[mid_row]) * downsample,
        dy_m=float(my_lookup[mid_row]) * downsample,
    )

    colors = ["cyan", "red", "orange", "magenta", "white"]
    named_paths = [
        (f"w={r['weight']} ({r['distance_km']:.1f} km)", r["path_3d"], colors[i % len(colors)], 3.5)
        for i, r in enumerate(results)
    ]

    figure = fpl.Figure(size=(1400, 900))
    add_subplot_scene(
        figure[0, 0],
        terrain_rgb,
        f"C++ A* Trajectory (w={weights})",
        dem_loader,
        min_row,
        min_col,
        downsample,
        start_pixel,
        end_pixel,
        named_paths,
    )

    print("\n>> Starting viewer...")
    figure.show(maintain_aspect=True)
    import rendercanvas.auto

    rendercanvas.auto.loop.run()


if __name__ == "__main__":
    main()
