
import matplotlib.pyplot as plt
import numpy as np
import math

from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap, LightSource
from pathlib import Path

# Attempt to import DEMLoader
try:
    from terrain.dem_loader import DEMLoader
except ImportError:
    print("Warning: Could not import DEMLoader. Terrain background will be disabled.")
    DEMLoader = None


class MissionPlotter:
    """
    Handles visualization of missile trajectory, sensor data, and errors.
    Automatically zooms into the active flight area and overlays DEM terrain.
    """

    @staticmethod
    def plot_mission(history: dict, planned_path: list = None,
                     dem_file: str = None, origin: tuple = None,
                     title: str = "Mission Analysis", downsample: int = 2):
        """
        Generates mission summary plots with optional terrain background.

        Args:
            history (dict): Must contain 'time', 'true_pos' (Nx3), 'est_pos' (Nx3).
                            Assumes Z-Up Standard: [East, North, Altitude].
            planned_path (list): Optional list of waypoints [[x, y, z], ...].
            dem_file (str): Filename of the .tif file in 'data/dem/' (e.g., "srtm_43_02.tif").
            origin (tuple): (Latitude, Longitude) of the (0,0) point in the simulation.
                            Required if dem_file is provided.
            downsample (int): Factor to reduce DEM resolution for faster plotting (default: 2).
        """

        # 1. Data Preparation
        t = np.array(history['time'])
        true_pos = np.array(history['true_pos'])  # [x, y, z]
        est_pos = np.array(history['est_pos'])

        # Calculate Position Error
        pos_error = np.linalg.norm(true_pos - est_pos, axis=1)

        # 2. Calculate Bounds (Auto-Zoom) in METERS
        all_x = [true_pos[:, 0], est_pos[:, 0]]
        all_y = [true_pos[:, 1], est_pos[:, 1]]

        if planned_path is not None:
            pp = np.array(planned_path)
            all_x.append(pp[:, 0])
            all_y.append(pp[:, 1])

        flat_x = np.concatenate(all_x)
        flat_y = np.concatenate(all_y)

        min_x, max_x = np.min(flat_x), np.max(flat_x)
        min_y, max_y = np.min(flat_y), np.max(flat_y)

        # Padding (10%)
        span_x = max_x - min_x
        span_y = max_y - min_y
        padding = max(span_x, span_y) * 0.1 if max(span_x, span_y) > 0 else 100.0

        # Final View Limits (Meters)
        view_min_x, view_max_x = min_x - padding, max_x + padding
        view_min_y, view_max_y = min_y - padding, max_y + padding

        # 3. Create Figure
        fig = plt.figure(figsize=(16, 10))
        gs = GridSpec(3, 2, figure=fig)
        fig.suptitle(title, fontsize=16, fontweight='bold')

        # --- PLOT A: Ground Track (Top-Down) ---
        ax_top = fig.add_subplot(gs[0:2, 0])
        ax_top.set_title(f"Ground Track (Zoomed, Downsample={downsample})")
        ax_top.set_xlabel("East (X) [m]")
        ax_top.set_ylabel("North (Y) [m]")

        # === TERRAIN BACKGROUND LOGIC ===
        if dem_file and origin and DEMLoader:
            try:
                # Construct Path: PROJECT_ROOT/data/dem/FILENAME
                project_root = Path(__file__).resolve().parents[2]
                dem_path = project_root / "data" / "dem" / dem_file

                if not dem_path.exists():
                    dem_path = Path("data/dem") / dem_file

                if dem_path.exists():
                    dem = DEMLoader(dem_path)

                    # 1. Inverse Project: Meter Bounds -> Lat/Lon Bounds
                    origin_lat, origin_lon = origin
                    meters_per_lat = 111320.0
                    meters_per_lon = 111320.0 * math.cos(math.radians(origin_lat))

                    # Calculate Lat/Lon for the view corners
                    # Top-Left (North-West)
                    lat_max = origin_lat + (view_max_y / meters_per_lat)
                    lon_min = origin_lon + (view_min_x / meters_per_lon)

                    # Bottom-Right (South-East)
                    lat_min = origin_lat + (view_min_y / meters_per_lat)
                    lon_max = origin_lon + (view_max_x / meters_per_lon)

                    # 2. Get Pixel Indices
                    r_min, c_min = dem.lat_lon_to_pixel(lat_max, lon_min)  # Top-Left
                    r_max, c_max = dem.lat_lon_to_pixel(lat_min, lon_max)  # Bottom-Right

                    # Handle indices order
                    r_start, r_end = min(r_min, r_max), max(r_min, r_max)
                    c_start, c_end = min(c_min, c_max), max(c_min, c_max)

                    # 3. Slice DEM with Downsampling
                    buffer = 2 * downsample
                    r_start = max(0, r_start - buffer)
                    r_end = min(dem.shape[0], r_end + buffer)
                    c_start = max(0, c_start - buffer)
                    c_end = min(dem.shape[1], c_end + buffer)

                    if r_end > r_start and c_end > c_start:
                        # APPLY DOWNSAMPLING HERE
                        dem_patch = dem.data[r_start:r_end:downsample, c_start:c_end:downsample].astype(float)

                        # Handle NoData
                        dem_patch[dem_patch == dem.nodata] = np.nan

                        # 4. Calculate Exact Extent for imshow (in Meters)
                        # We use the original corner pixels to determine the geographic extent
                        patch_lat_top, patch_lon_left = dem.pixel_to_lat_lon(r_start, c_start)
                        patch_lat_bottom, patch_lon_right = dem.pixel_to_lat_lon(r_end, c_end)

                        img_min_x = (patch_lon_left - origin_lon) * meters_per_lon
                        img_max_x = (patch_lon_right - origin_lon) * meters_per_lon
                        img_min_y = (patch_lat_bottom - origin_lat) * meters_per_lat
                        img_max_y = (patch_lat_top - origin_lat) * meters_per_lat

                        # 5. Create Terrain Colormap
                        colors = ['#2d5016', '#5a8c2b', '#8fb359', '#d4c77e', '#a67c52', '#ffffff']
                        cmap_terrain = LinearSegmentedColormap.from_list('terrain', colors, N=100)

                        # Hillshade for 3D effect
                        ls = LightSource(azdeg=315, altdeg=45)
                        hillshade = ls.hillshade(dem_patch, vert_exag=1, dx=1, dy=1)

                        # Plot Map
                        ax_top.imshow(hillshade, cmap='gray',
                                      extent=[img_min_x, img_max_x, img_min_y, img_max_y],
                                      alpha=0.4, origin='upper')

                        ax_top.imshow(dem_patch, cmap=cmap_terrain,
                                      extent=[img_min_x, img_max_x, img_min_y, img_max_y],
                                      origin='upper', alpha=0.6)

                        print(f"      [Plotter] Terrain loaded (Downsample={downsample}). Shape: {dem_patch.shape}")

            except Exception as e:
                print(f"      [Plotter] Failed to load DEM background: {e}")
        # ================================

        # Plot Planned Path
        if planned_path is not None:
            ax_top.plot(pp[:, 0], pp[:, 1], 'g--', linewidth=1.5, label="Planned")
            ax_top.scatter(pp[:, 0], pp[:, 1], color='green', marker='x', s=30)

        # Plot Trajectories
        ax_top.plot(true_pos[:, 0], true_pos[:, 1], 'b-', linewidth=2, label="True")
        ax_top.plot(est_pos[:, 0], est_pos[:, 1], 'r:', linewidth=2, label="Est (KF)")

        # Markers
        ax_top.scatter(true_pos[0, 0], true_pos[0, 1], c='lime', s=100, edgecolors='k', zorder=5)
        ax_top.scatter(true_pos[-1, 0], true_pos[-1, 1], c='black', marker='X', s=100, edgecolors='w', zorder=5)

        # Set Bounds
        ax_top.set_xlim(view_min_x, view_max_x)
        ax_top.set_ylim(view_min_y, view_max_y)
        ax_top.set_aspect('equal')
        ax_top.legend(loc='upper right')

        # --- PLOT B: Altitude Profile ---
        ax_alt = fig.add_subplot(gs[0, 1])
        ax_alt.set_title("Altitude Profile")
        ax_alt.set_ylabel("Altitude (Z) [m]")
        ax_alt.set_xlabel("Time [s]")
        ax_alt.plot(t, true_pos[:, 2], 'b-', label="True")
        ax_alt.plot(t, est_pos[:, 2], 'r:', label="Est")
        ax_alt.axhline(0, color='k', linewidth=1)
        ax_alt.grid(True, linestyle='--', alpha=0.4)
        ax_alt.legend()

        # --- PLOT C: Error ---
        ax_err = fig.add_subplot(gs[1, 1])
        ax_err.set_title("Position Error")
        ax_err.set_ylabel("Error [m]")
        ax_err.set_xlabel("Time [s]")
        ax_err.plot(t, pos_error, 'k-', label="Distance Error")
        ax_err.axhline(10, color='red', linestyle='--', label="10m Limit")
        ax_err.grid(True, linestyle='--', alpha=0.4)
        ax_err.legend()

        # --- PLOT D: 3D View ---
        ax_3d = fig.add_subplot(gs[2, :], projection='3d')
        ax_3d.set_title("3D Trajectory")
        ax_3d.plot(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2], 'b-', label="True")
        ax_3d.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], 'r--', label="Est")
        # Ground plane reference (min Z)
        z_floor = min(true_pos[:, 2].min(), 0)
        ax_3d.plot(true_pos[:, 0], true_pos[:, 1], z_floor, 'gray', alpha=0.2)
        ax_3d.set_box_aspect((span_x, span_y, max(true_pos[:, 2]) - z_floor + 10))

        plt.tight_layout()
        plt.show()