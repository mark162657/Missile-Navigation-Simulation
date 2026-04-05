"""
2D Fastplotlib visualization of SMOOTHED pathfinding results on DEM terrain.
Uses TrajectoryGenerator to visualize the actual flyable curve.
"""

import sys
import time
import math
import numpy as np
from pathlib import Path
import fastplotlib as fpl

# Add project root to path (Parent of 'tests' is root)
sys.path.insert(0, str(Path(__file__).parent.parent))

# UPDATED IMPORTS
sys.path.insert(0, str(Path(__file__).parent.parent))
from src.pathfinder.pathfinding_backend import Pathfinding
from src.pathfinder.trajectory import TrajectoryGenerator
def calculate_geo_distance(p1, p2):
    """Simple Euclidean distance for stats (using lat/lon approx)"""
    # Note: This is a rough approx for visualization stats only.
    # Real pathfinder uses the C++ distance metric.
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) * 111000 # ~Meters

def visualize_path_fastplotlib(start_gps: tuple[float, float], end_gps: tuple[float, float],
                                weights: list[float] = [1.5]):
    """
    Run pathfinding -> Trajectory Generation -> Visualize smooth curve in 2D.
    """

    print("\n" + "="*60)
    print("2D TRAJECTORY VISUALIZATION (FASTPLOTLIB)")
    print("="*60)

    # 1. Initialize Pathfinding (Backend + Loader)
    # Ensure this matches your actual init signature (e.g. passing a filename or path)
    pf = Pathfinding("merged_dem_sib_N54_N59_E090_E100.tif")

    # 2. Initialize Trajectory Generator
    # This connects the C++ Engine (for height) with the Python Loader (for coords)
    traj_gen = TrajectoryGenerator(pf.engine, pf.dem_loader)

    # ALIAS for easy access to the numpy array
    dem_data = pf.dem_loader.data

    # Convert GPS to pixel coordinates for A* Start/End
    start_row, start_col = pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1])
    end_row, end_col = pf.dem_loader.lat_lon_to_pixel(end_gps[0], end_gps[1])

    start_pixel = (start_row, start_col)
    end_pixel = (end_row, end_col)

    print(f"\nStart GPS: {start_gps} -> Pixel: {start_pixel}")
    print(f"End GPS:   {end_gps} -> Pixel: {end_pixel}")

    # Compute paths for each weight
    paths_data = []

    for weight in weights:
        print(f"\n--- Testing Weight: {weight} ---")
        start_time = time.time()

        # A. Run A* (Returns Raw Grid Pixels)
        raw_path = pf.find_path(start_pixel, end_pixel, heuristic_weight=weight)

        if raw_path:
            # B. Generate Trajectory (Returns Smooth GPS + Altitude)
            # This handles Splines + C++ Height Check + Min AGL
            smooth_path_3d = traj_gen.get_trajectory(
                raw_path,
                smooth_factor=2.0,
                res_multi=5,
                min_alt=30.0
            )

            elapsed = time.time() - start_time
            path_length = len(smooth_path_3d)

            # Basic stats for the smoothed path
            total_dist = 0.0
            total_climb = 0.0

            # Simple stats loop
            for i in range(len(smooth_path_3d) - 1):
                p1 = smooth_path_3d[i] # (lat, lon, alt)
                p2 = smooth_path_3d[i+1]

                # Distance
                total_dist += calculate_geo_distance(p1, p2)

                # Climb (Target Altitude difference)
                climb = max(0, p2[2] - p1[2])
                total_climb += climb

            paths_data.append({
                'weight': weight,
                'path_3d': smooth_path_3d, # Storing the final GPS path
                'length': path_length,
                'distance': total_dist,
                'climb': total_climb,
                'time': elapsed
            })

            print(f"[OK] Trajectory generated: {path_length} points, {total_dist/1000:.2f}km")
            print(f"   Time: {elapsed:.3f}s")
        else:
            print(f"[ERROR] No path found")

    if not paths_data:
        print("\n[ERROR] No paths found. Exiting.")
        return

    # --- VISUALIZATION SETUP ---
    print("\n" + "="*60)
    print("GENERATING 2D VISUALIZATION...")
    print("="*60)

    # Determine bounds based on Start/End PIXELS (since path is now GPS)
    all_rows = [start_row, end_row]
    all_cols = [start_col, end_col]

    # If we have path data, we can also include the projected pixel bounds of the curves
    # (Optional optimization, sticking to start/end is usually safe enough for zoom)

    min_row, max_row = min(all_rows), max(all_rows)
    min_col, max_col = min(all_cols), max(all_cols)

    # Padding logic
    row_padding = int((max_row - min_row) * 0.2) + 100
    col_padding = int((max_col - min_col) * 0.2) + 100

    # Use dem_loader.shape to clamp bounds safely
    min_row = max(0, min_row - row_padding)
    max_row = min(pf.dem_loader.shape[0] - 1, max_row + row_padding)

    min_col = max(0, min_col - col_padding)
    max_col = min(pf.dem_loader.shape[1] - 1, max_col + col_padding)

    # Force square aspect ratio logic
    height = max_row - min_row
    width = max_col - min_col
    target_dim = max(height, width)

    if height < target_dim:
        diff = target_dim - height
        min_row = max(0, min_row - diff // 2)
        max_row = min(pf.dem_loader.shape[0] - 1, max_row + diff // 2)
    if width < target_dim:
        diff = target_dim - width
        min_col = max(0, min_col - diff // 2)
        max_col = min(pf.dem_loader.shape[1] - 1, max_col + diff // 2)

    print(f"Zoom area: rows [{min_row}:{max_row}], cols [{min_col}:{max_col}]")

    # Downsample logic
    max_pixels = 30_000_000
    zoom_rows = max_row - min_row
    zoom_cols = max_col - min_col

    optimal_downsample = 1
    while True:
        downsampled_pixels = (zoom_rows // optimal_downsample) * (zoom_cols // optimal_downsample)
        if downsampled_pixels <= max_pixels:
            break
        optimal_downsample += 1

    downsample = optimal_downsample
    print(f"Downsample factor: {downsample}")

    # Extract DEM region
    dem_region = dem_data[min_row:max_row:downsample, min_col:max_col:downsample].astype(float)
    dem_region[dem_region <= -100] = np.nan

    # Contrast stats
    valid_elev = dem_region[~np.isnan(dem_region)]
    if len(valid_elev) > 0:
        elev_min = np.percentile(valid_elev, 2)
        elev_max = np.percentile(valid_elev, 98)
    else:
        elev_min, elev_max = 0, 1000

    dem_display = np.nan_to_num(dem_region, nan=elev_min)

    # Create Figure
    figure = fpl.Figure(size=(1400, 900))

    heatmap = figure[0, 0].add_image(
        data=dem_display,
        cmap="gist_ncar",
        vmin=elev_min,
        vmax=elev_max,
    )
    figure[0, 0].name = f"Smoothed Trajectory (w={weights})"

    path_colors = ['red', 'blue', 'green', 'orange', 'purple']

    # --- PLOT PATHS ---
    for i, data in enumerate(paths_data):
        smooth_gps_path = data['path_3d'] # List of (Lat, Lon, Alt)
        weight = data['weight']

        # Convert GPS back to Viewport Pixels for visualization
        path_pixels = []
        for lat, lon, alt in smooth_gps_path:
            # 1. Get Absolute Pixel (Global Map)
            r_global, c_global = pf.dem_loader.lat_lon_to_pixel(lat, lon)

            # 2. Get Relative Pixel (Zoomed Viewport)
            # Subtract offset and divide by downsample
            r_local = (r_global - min_row) / downsample
            c_local = (c_global - min_col) / downsample

            # 3. Check bounds (for safety)
            if 0 <= r_local < dem_region.shape[0] and 0 <= c_local < dem_region.shape[1]:
                 # fpl uses (x, y) which corresponds to (col, row)
                path_pixels.append([c_local, r_local])

        path_pixels = np.array(path_pixels, dtype=np.float32)

        if len(path_pixels) > 0:
            figure[0, 0].add_line(
                data=path_pixels,
                thickness=3,
                colors=path_colors[i % len(path_colors)],
                name=f"w={weight} ({data['length']} pts)"
            )

    # Markers (Start/End) - Convert Global Pixels to Viewport Pixels
    start_ds_r = (start_row - min_row) // downsample
    start_ds_c = (start_col - min_col) // downsample

    end_ds_r = (end_row - min_row) // downsample
    end_ds_c = (end_col - min_col) // downsample

    # Safety check for markers
    if 0 <= start_ds_r < dem_region.shape[0] and 0 <= start_ds_c < dem_region.shape[1]:
        figure[0, 0].add_scatter(np.array([[start_ds_c, start_ds_r]], dtype=np.float32),
                                sizes=20, colors="lime", name="Start")

    if 0 <= end_ds_r < dem_region.shape[0] and 0 <= end_ds_c < dem_region.shape[1]:
        figure[0, 0].add_scatter(np.array([[end_ds_c, end_ds_r]], dtype=np.float32),
                                sizes=20, colors="red", name="End")

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    for data in paths_data:
        print(f"\nWeight {data['weight']}:")
        print(f"  Points: {data['length']}")
        print(f"  Distance: {data['distance']/1000:.2f} km")
        print(f"  Climb: {data['climb']:.0f} m")
        print(f"  Time: {data['time']:.3f} s")

    print("\n>> Starting viewer...")
    print("Press Ctrl+C or close the window to exit...")
    figure.show(maintain_aspect=True)
    import rendercanvas.auto
    rendercanvas.auto.loop.run()


if __name__ == "__main__":
    # Test Config
    print("\nSelect test scenario:")
    print("1. Short path (~10km)")
    print("2. Medium path (~50km)")
    print("3. Long path (~100km)")
    print("4. Custom coordinates")

    choice = input("\nEnter choice (1-4): ").strip()

    base_lat, base_lon = 56.0, 95.0
    deg_per_km_lon = 1 / (111.32 * 0.559193)
    deg_per_km_lat = 1 / 111.32

    if choice == "1":
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat, base_lon + 10 * deg_per_km_lon)

    elif choice == "2":
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat + 25 * deg_per_km_lat, base_lon + 43.3 * deg_per_km_lon)

    elif choice == "3":
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat, base_lon + 100 * deg_per_km_lon)

    elif choice == "4":
        print("\nEnter custom coordinates:")
        start_lat = float(input("Start Latitude: "))
        start_lon = float(input("Start Longitude: "))
        end_lat = float(input("End Latitude: "))
        end_lon = float(input("End Longitude: "))
        start_gps = (start_lat, start_lon)
        end_gps = (end_lat, end_lon)
    else:
        print("Invalid choice. Using default 10km test.")
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat, base_lon + 10 * deg_per_km_lon)

    # Run 2D visualization
    visualize_path_fastplotlib(start_gps, end_gps, weights=[2])