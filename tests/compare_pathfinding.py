"""
Side-by-side comparison of Rust A* vs. C++ A* pathfinding on DEM terrain.

Uses an absolute physical hypsometric color scale (0..4500 m MSL) combined with
metric-accurate Lambertian hillshading so flat plains look calm and flat while
real mountain ridges and river gorges show crisp 3D relief.
"""

import math
import sys
import time
from pathlib import Path

import fastplotlib as fpl
import numpy as np
import pathfinder

# Add src package root to path
SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from paths import PROJECT_ROOT
from terrain.coordinates import meter_per_deg_lat, meter_per_deg_lon_at
from terrain.dem_loader import DEMLoader
from missile.pathfinding.trajectory import TrajectoryGenerator as RustTrajectoryGenerator
from missile.planning.pathfinding_backend import Pathfinding as CppPathfinding
from missile.planning.trajectory import TrajectoryGenerator as CppTrajectoryGenerator


# Absolute cartographic hypsometric stops (MSL meters -> RGB 0..255)
_ELEV_STOPS_M = np.array([-50.0, 0.0, 300.0, 800.0, 1500.0, 2500.0, 3500.0, 4500.0], dtype=np.float32)
_ELEV_STOPS_RGB = np.array(
    [
        [65, 105, 135],   # -50 m: below-sea-level basin
        [88, 138, 96],    #   0 m: coastal lowland green
        [118, 158, 100],  # 300 m: plains green
        [175, 178, 118],  # 800 m: foothills olive-tan
        [198, 162, 112],  # 1500 m: plateau warm tan
        [164, 122, 86],   # 2500 m: mountain brown
        [145, 136, 132],  # 3500 m: alpine rock grey
        [242, 244, 247],  # 4500 m: high peaks snow
    ],
    dtype=np.float32,
)


def render_realistic_terrain_rgb(
    dem_region: np.ndarray,
    dx_m: float,
    dy_m: float,
    z_exaggeration: float = 2.0,
) -> np.ndarray:
    """
    Convert a 2D elevation array (meters MSL) into an (H, W, 3) uint8 RGB image
    using an absolute elevation ramp + physical slope hillshading.
    """
    nodata_mask = np.isnan(dem_region) | (dem_region <= -100.0)
    valid = dem_region[~nodata_mask]
    fill_val = float(np.median(valid)) if valid.size > 0 else 0.0
    elev = np.where(nodata_mask, fill_val, dem_region).astype(np.float32)

    # 1. Absolute hypsometric base color (interpolated per RGB channel)
    base_rgb = np.empty((*elev.shape, 3), dtype=np.float32)
    for ch in range(3):
        base_rgb[..., ch] = np.interp(elev, _ELEV_STOPS_M, _ELEV_STOPS_RGB[:, ch])

    # 2. Metric-accurate hillshade (sun from NW 315°, altitude 45°)
    if elev.shape[0] >= 2 and elev.shape[1] >= 2:
        gy, gx = np.gradient(elev, max(dy_m, 1e-3), max(dx_m, 1e-3))
        slope = np.arctan(z_exaggeration * np.hypot(gx, gy))
        aspect = np.arctan2(-gy, gx)

        az = math.radians(315.0)
        alt = math.radians(45.0)
        shade = np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)
        shade = np.clip(shade, 0.0, 1.0)
        # Flat ground (slope=0) -> shade = sin(45°) ≈ 0.7071 -> light_factor ≈ 1.00
        light_factor = 0.45 + 0.78 * shade
    else:
        light_factor = np.ones_like(elev)

    rgb = np.clip(base_rgb * light_factor[..., None], 0.0, 255.0).astype(np.uint8)
    rgb[nodata_mask] = np.array([28, 32, 38], dtype=np.uint8)
    return rgb


def calculate_geo_distance(p1, p2) -> float:
    """Euclidean ground distance in meters (with cos(lat) longitude scaling)."""
    mid_lat_rad = math.radians(0.5 * (p1[0] + p2[0]))
    dy = (p2[0] - p1[0]) * 111_320.0
    dx = (p2[1] - p1[1]) * 111_320.0 * math.cos(mid_lat_rad)
    return math.hypot(dx, dy)


def compute_trajectory_stats(path_3d: np.ndarray, elapsed_s: float) -> dict:
    """Compute distance, total climb, max step |dz|, and elevation stats."""
    total_dist = 0.0
    total_climb = 0.0
    max_dz = 0.0

    for i in range(len(path_3d) - 1):
        p1 = path_3d[i]
        p2 = path_3d[i + 1]
        total_dist += calculate_geo_distance(p1, p2)
        dz = float(p2[2] - p1[2])
        if dz > 0.0:
            total_climb += dz
        if abs(dz) > max_dz:
            max_dz = abs(dz)

    elevs = path_3d[:, 2] if len(path_3d) > 0 else np.array([0.0])
    avg_climb_deg = math.degrees(math.atan2(total_climb, max(total_dist, 1.0)))

    return {
        "path_3d": path_3d,
        "points": len(path_3d),
        "distance_km": total_dist / 1000.0,
        "climb_m": total_climb,
        "max_dz_m": max_dz,
        "min_elev_m": float(elevs.min()),
        "max_elev_m": float(elevs.max()),
        "avg_climb_deg": avg_climb_deg,
        "time_s": elapsed_s,
    }


def choose_dem() -> Path:
    """Interactive selector for any .tif DEM in data/dem/."""
    dem_dir = PROJECT_ROOT / "data" / "dem"
    tifs = sorted(dem_dir.glob("*.tif"))
    if not tifs:
        raise FileNotFoundError(f"No .tif DEM files found in {dem_dir}")

    print("\nAvailable DEM files:")
    for idx, tif in enumerate(tifs, start=1):
        size_mb = tif.stat().st_size / (1024 * 1024)
        print(f"  [{idx}] {tif.name} ({size_mb:.1f} MB)")

    while True:
        choice = input(f"\nChoose DEM [1-{len(tifs)}] (default 1): ").strip()
        if not choice:
            return tifs[0]
        if choice.isdigit() and 1 <= int(choice) <= len(tifs):
            return tifs[int(choice) - 1]
        print("Invalid choice, try again.")


def choose_scenario_gps(dem_loader: DEMLoader) -> tuple[tuple[float, float], tuple[float, float]]:
    """Interactive route scenario picker centered on the chosen DEM."""
    rows, cols = dem_loader.shape
    base_lat, base_lon = dem_loader.pixel_to_lat_lon(rows // 2, cols // 2)
    base_lat, base_lon = float(base_lat), float(base_lon)
    deg_per_km_lat = 1000.0 / meter_per_deg_lat(base_lat)
    deg_per_km_lon = 1000.0 / meter_per_deg_lon_at(base_lat)

    print(f"DEM center GPS: ({base_lat:.4f}, {base_lon:.4f}) | shape: {rows}x{cols}")
    print("\nSelect test scenario:")
    print("  1. Short path  (~10 km)")
    print("  2. Medium path (~50 km)")
    print("  3. Long path   (~100 km)")
    print("  4. Custom GPS coordinates")

    choice = input("\nEnter choice (1-4, default 2): ").strip() or "2"

    if choice == "1":
        return (base_lat, base_lon), (base_lat, base_lon + 10.0 * deg_per_km_lon)
    if choice == "2":
        return (base_lat, base_lon), (base_lat + 25.0 * deg_per_km_lat, base_lon + 43.3 * deg_per_km_lon)
    if choice == "3":
        return (base_lat, base_lon), (base_lat, base_lon + 100.0 * deg_per_km_lon)
    if choice == "4":
        print("\nEnter custom GPS coordinates:")
        s_lat = float(input("  Start Latitude:  "))
        s_lon = float(input("  Start Longitude: "))
        e_lat = float(input("  End Latitude:    "))
        e_lon = float(input("  End Longitude:   "))
        return (s_lat, s_lon), (e_lat, e_lon)

    print("Invalid choice. Using default 50 km test.")
    return (base_lat, base_lon), (base_lat + 25.0 * deg_per_km_lat, base_lon + 43.3 * deg_per_km_lon)


def build_metric_lookups(dem_loader: DEMLoader) -> tuple[np.ndarray, np.ndarray]:
    """Build per-row meters-per-pixel lookup tables (float64) for Rust A*."""
    rows, _ = dem_loader.shape
    start_lat = dem_loader.transform[5]
    pixel_height = dem_loader.transform[4]
    pixel_width = abs(dem_loader.transform[0])
    lats = start_lat + np.arange(rows, dtype=np.float64) * pixel_height

    mx_lookup = np.ascontiguousarray(
        [meter_per_deg_lon_at(float(lat)) * pixel_width for lat in lats],
        dtype=np.float64,
    )
    my_lookup = np.ascontiguousarray(
        [meter_per_deg_lat(float(lat)) * abs(pixel_height) for lat in lats],
        dtype=np.float64,
    )
    return mx_lookup, my_lookup


def compute_viewport(
    dem_shape: tuple[int, int],
    start_pixel: tuple[int, int],
    end_pixel: tuple[int, int],
    extra_paths_3d: list[np.ndarray],
    dem_loader: DEMLoader,
    max_pixels: int = 20_000_000,
) -> tuple[int, int, int, int, int]:
    """Compute a square, padded, downsampled viewport covering start, end, and all paths."""
    all_rows = [start_pixel[0], end_pixel[0]]
    all_cols = [start_pixel[1], end_pixel[1]]

    for path_3d in extra_paths_3d:
        if len(path_3d) > 0:
            rs, cs = dem_loader.lat_lon_to_pixel(path_3d[:, 0], path_3d[:, 1])
            all_rows.extend([int(np.min(rs)), int(np.max(rs))])
            all_cols.extend([int(np.min(cs)), int(np.max(cs))])

    min_row, max_row = min(all_rows), max(all_rows)
    min_col, max_col = min(all_cols), max(all_cols)

    row_pad = int((max_row - min_row) * 0.2) + 100
    col_pad = int((max_col - min_col) * 0.2) + 100

    min_row = max(0, min_row - row_pad)
    max_row = min(dem_shape[0] - 1, max_row + row_pad)
    min_col = max(0, min_col - col_pad)
    max_col = min(dem_shape[1] - 1, max_col + col_pad)

    height = max_row - min_row
    width = max_col - min_col
    target_dim = max(height, width)

    if height < target_dim:
        diff = target_dim - height
        min_row = max(0, min_row - diff // 2)
        max_row = min(dem_shape[0] - 1, max_row + diff // 2)
    if width < target_dim:
        diff = target_dim - width
        min_col = max(0, min_col - diff // 2)
        max_col = min(dem_shape[1] - 1, max_col + diff // 2)

    zoom_rows = max_row - min_row
    zoom_cols = max_col - min_col
    downsample = 1
    while (zoom_rows // downsample) * (zoom_cols // downsample) > max_pixels:
        downsample += 1

    return min_row, max_row, min_col, max_col, downsample


def add_subplot_scene(
    subplot,
    terrain_rgb: np.ndarray,
    title: str,
    dem_loader: DEMLoader,
    min_row: int,
    min_col: int,
    downsample: int,
    start_pixel: tuple[int, int],
    end_pixel: tuple[int, int],
    named_paths: list[tuple[str, np.ndarray, str, float]],
) -> None:
    """Render shaded terrain + path lines + start/end markers onto a Fastplotlib subplot."""
    subplot.add_image(data=terrain_rgb)
    subplot.name = title

    h, w, _ = terrain_rgb.shape

    print("\n--- Map Legend ---")
    print("  Lime dot ── Start")
    print("  Red dot  ── Target (End)")

    for label, path_3d, color, thickness in named_paths:
        if len(path_3d) == 0:
            continue
        rs, cs = dem_loader.lat_lon_to_pixel(path_3d[:, 0], path_3d[:, 1])
        r_local = (np.asarray(rs, dtype=np.float32) - min_row) / downsample
        c_local = (np.asarray(cs, dtype=np.float32) - min_col) / downsample
        mask = (r_local >= 0) & (r_local < h) & (c_local >= 0) & (c_local < w)
        pts = np.column_stack((c_local[mask], r_local[mask])).astype(np.float32)
        if len(pts) > 0:
            subplot.add_line(data=pts, thickness=thickness, colors=color, name=label)
            print(f"  {color.capitalize():<8} ── {label}")

    s_r = (start_pixel[0] - min_row) / downsample
    s_c = (start_pixel[1] - min_col) / downsample
    e_r = (end_pixel[0] - min_row) / downsample
    e_c = (end_pixel[1] - min_col) / downsample

    if 0 <= s_r < h and 0 <= s_c < w:
        subplot.add_scatter(
            np.array([[s_c, s_r]], dtype=np.float32),
            sizes=18,
            colors="lime",
            name="Start",
        )
    if 0 <= e_r < h and 0 <= e_c < w:
        subplot.add_scatter(
            np.array([[e_c, e_r]], dtype=np.float32),
            sizes=18,
            colors="#ff2222",
            name="End",
        )


def run_rust_single(
    dem_loader: DEMLoader,
    dem_f32: np.ndarray,
    mx_lookup: np.ndarray,
    my_lookup: np.ndarray,
    start_pixel: tuple[int, int],
    end_pixel: tuple[int, int],
    weight: float,
) -> dict | None:
    traj_gen = RustTrajectoryGenerator(None, dem_loader)
    t0 = time.perf_counter()
    raw_path = pathfinder.rs_pathfinder(
        dem_f32, mx_lookup, my_lookup, start_pixel, end_pixel, float(weight)
    )
    if not raw_path:
        return None
    path_3d = traj_gen.get_trajectory(raw_path)
    elapsed = time.perf_counter() - t0
    stats = compute_trajectory_stats(path_3d, elapsed)
    stats["weight"] = weight
    return stats


def run_cpp_single(
    cpp_pf: CppPathfinding,
    start_pixel: tuple[int, int],
    end_pixel: tuple[int, int],
    weight: float,
) -> dict | None:
    traj_gen = CppTrajectoryGenerator(cpp_pf.engine, cpp_pf.dem_loader)
    t0 = time.perf_counter()
    raw_path = cpp_pf.find_path(start_pixel, end_pixel, heuristic_weight=float(weight))
    if not raw_path:
        return None
    path_3d = traj_gen.get_trajectory(raw_path)
    elapsed = time.perf_counter() - t0
    stats = compute_trajectory_stats(path_3d, elapsed)
    stats["weight"] = weight
    return stats


def compare_rust_vs_cpp() -> None:
    tif_path = choose_dem()
    print(f"\nLoading {tif_path.name}...")
    cpp_pf = CppPathfinding(tif_path.name)
    dem_loader = cpp_pf.dem_loader
    dem_f32 = cpp_pf.dem
    mx_lookup, my_lookup = build_metric_lookups(dem_loader)

    start_gps, end_gps = choose_scenario_gps(dem_loader)

    rust_w_raw = input("\nRust heuristic weight (default 1.1): ").strip()
    rust_w = float(rust_w_raw) if rust_w_raw else 1.1

    cpp_w_raw = input("C++  heuristic weight (default 2.0): ").strip()
    cpp_w = float(cpp_w_raw) if cpp_w_raw else 2.0

    sr, sc = dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1])
    er, ec = dem_loader.lat_lon_to_pixel(end_gps[0], end_gps[1])
    start_pixel = (int(sr), int(sc))
    end_pixel = (int(er), int(ec))

    print("\n" + "=" * 68)
    print(f"Start GPS: {start_gps} -> Pixel {start_pixel}")
    print(f"End GPS:   {end_gps} -> Pixel {end_pixel}")
    print("=" * 68)

    print(f"\n[1/2] Running Rust A* (w={rust_w})...")
    rust_res = run_rust_single(
        dem_loader, dem_f32, mx_lookup, my_lookup, start_pixel, end_pixel, rust_w
    )

    print(f"\n[2/2] Running C++ A*  (w={cpp_w})...")
    if cpp_pf.engine is None:
        print("[SKIP] C++ engine not available for this Python version; showing Rust results only.")
        cpp_res = None
    else:
        cpp_res = run_cpp_single(cpp_pf, start_pixel, end_pixel, cpp_w)

    if not rust_res and not cpp_res:
        print("\n[ERROR] Neither engine found a path.")
        return

    # --- Side-by-side comparison table ---
    print("\n" + "=" * 68)
    print(f"{'METRIC':<20} | {'RUST (w=' + str(rust_w) + ')':<20} | {'C++ (w=' + str(cpp_w) + ')':<20}")
    print("-" * 68)

    def fmt(res, key, fmt_str, suffix=""):
        return f"{res[key]:{fmt_str}}{suffix}" if res else "N/A"

    print(f"{'Time':<20} | {fmt(rust_res, 'time_s', '.3f', ' s'):<20} | {fmt(cpp_res, 'time_s', '.3f', ' s'):<20}")
    print(f"{'Waypoints':<20} | {fmt(rust_res, 'points', 'd'):<20} | {fmt(cpp_res, 'points', 'd'):<20}")
    print(f"{'Distance':<20} | {fmt(rust_res, 'distance_km', '.2f', ' km'):<20} | {fmt(cpp_res, 'distance_km', '.2f', ' km'):<20}")
    print(f"{'Total Climb':<20} | {fmt(rust_res, 'climb_m', '.0f', ' m'):<20} | {fmt(cpp_res, 'climb_m', '.0f', ' m'):<20}")
    print(f"{'Max Step |dz|':<20} | {fmt(rust_res, 'max_dz_m', '.1f', ' m'):<20} | {fmt(cpp_res, 'max_dz_m', '.1f', ' m'):<20}")
    rust_elev_str = f"{rust_res['min_elev_m']:.0f}..{rust_res['max_elev_m']:.0f} m" if rust_res else "N/A"
    cpp_elev_str = f"{cpp_res['min_elev_m']:.0f}..{cpp_res['max_elev_m']:.0f} m" if cpp_res else "N/A"
    print(f"{'Elev Min..Max':<20} | {rust_elev_str:<20} | {cpp_elev_str:<20}")
    if rust_res and cpp_res and rust_res["time_s"] > 0:
        speedup = cpp_res["time_s"] / rust_res["time_s"]
        print("-" * 68)
        print(f"Rust Speedup: {speedup:.2f}x faster")
    print("=" * 68)

    # --- Build shared shaded-relief terrain region ---
    paths_for_bounds = [r["path_3d"] for r in (rust_res, cpp_res) if r is not None]
    min_row, max_row, min_col, max_col, downsample = compute_viewport(
        dem_loader.shape, start_pixel, end_pixel, paths_for_bounds, dem_loader
    )

    dem_region = dem_f32[min_row:max_row:downsample, min_col:max_col:downsample].astype(np.float32)
    dem_region[dem_region <= -100.0] = np.nan

    mid_row = (min_row + max_row) // 2
    dx_m = float(mx_lookup[mid_row]) * downsample
    dy_m = float(my_lookup[mid_row]) * downsample

    valid_elev = dem_region[~np.isnan(dem_region)]
    if valid_elev.size > 0:
        print(
            f"\nViewport elevation: min={np.min(valid_elev):.0f} m, "
            f"max={np.max(valid_elev):.0f} m (downsample={downsample}x)"
        )

    terrain_rgb = render_realistic_terrain_rgb(dem_region, dx_m=dx_m, dy_m=dy_m)

    figure = fpl.Figure(size=(1400, 900))

    overlaid_paths = []
    if cpp_res:
        overlaid_paths.append(
            (f"C++ A* (w={cpp_w}, {cpp_res['distance_km']:.1f} km)", cpp_res["path_3d"], "cyan", 3.5)
        )
    if rust_res:
        overlaid_paths.append(
            (f"Rust A* (w={rust_w}, {rust_res['distance_km']:.1f} km)", rust_res["path_3d"], "red", 3.5)
        )

    add_subplot_scene(
        figure[0, 0],
        terrain_rgb,
        f"Pathfinding Comparison — {tif_path.name}",
        dem_loader,
        min_row,
        min_col,
        downsample,
        start_pixel,
        end_pixel,
        overlaid_paths,
    )

    print("\n>> Starting viewer (Red = Rust, Cyan = C++)...")
    print("Press Ctrl+C or close the window to exit...")
    figure.show(maintain_aspect=True)
    import rendercanvas.auto

    rendercanvas.auto.loop.run()


if __name__ == "__main__":
    compare_rust_vs_cpp()
