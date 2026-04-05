import numpy as np
import rasterio
from ..terrain.dem_loader import DEMLoader
from itertools import groupby
from scipy.interpolate import splprep, splev

class TrajectoryGenerator:
    def __init__(self, engine_backend, dem_loader: DEMLoader):
        self.engine = engine_backend
        self.dem = dem_loader

    def get_trajectory(self, raw_path: list[tuple[int, int]], smooth_factor: float=2.0, res_multi: int=5, min_alt: float=30.0) \
            -> list[tuple[int, int]]:
        """
        Smooth the path by B Spline (C2 continuity), which is aerodynamic and close to what missile could fly.
        Balancing continuity and local control.
        We ignore altitude during 2D smoothing (z will be handled by missile itself/terrain lookup), we focus on x, y (lat/lon).
        """

        # check length
        if not raw_path or len(raw_path) < 3: return []

        # check again for cleaned path
        clean_path = self._remove_duplication(raw_path)
        if len(clean_path) < 3: return []

        # Smoothing row and col separately
        smoothed_rows, smoothed_cols = self._compute_b_spline(clean_path, smooth_factor, res_multi)
        if smoothed_rows is None or smoothed_cols is None: return []

        # Get elevation by get_terrain_profile function in CPP
        ground_elevation = self.engine.get_terrain_profile(smoothed_rows, smoothed_cols)

        # Convert (int, int) pixel coordinate to -> GPS coordinates (float: lat, float: lon)
        lat, lon = self.dem.pixel_to_lat_lon(smoothed_rows, smoothed_cols)

        # Get 3D trajectory with target_altitude to met
        final_3d_trajectory = []

        for lat, lon, height in zip(lat, lon, ground_elevation):
            # check if no data region
            if height < -100:
                continue

            target_alt = height + min_alt

            final_3d_trajectory.append([lat, lon, target_alt])

        return final_3d_trajectory

    def _remove_duplication(self, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Filter out any potential consecutive duplicated points that can cause error.
        We use groupby from itertools. Probably the fastest

        Args:
            - path: the list contains tuple (representing pixels)... yeah just path
        """
        pixels = np.array(path)
        diffs = np.diff(pixels)
        return [k for k, g in groupby(path)]


    def _compute_b_spline(self, clean_path: list[tuple[int, int]], smooth_factor: float, res_multi: int) -> list[tuple[int, int]]:
        """
        Smooth the path by b spline, using splev and splprep
        """
        try:
            # Numpy slicing
            clean_path = np.array(clean_path)

            row = clean_path[:, 0] # select everything but only keep the 0th index
            col = clean_path[:, 1] # same but 1st

            # Main smoothing part. splprep for patter analysing, linspace for point generating, splev for drawing
            tck, u = splprep([row, col], s=smooth_factor * len(clean_path), k=3) # default k = 3, cubic
            u_new = np.linspace(0, 1, int(len(clean_path) * res_multi)) # draw original nodes * resolution upsampling to create smoother path
            new_row, new_col = splev(u_new, tck)

            return new_row.astype(np.float32), new_col.astype(np.float32)

        except Exception as e:
            print(f"Trajectory Gen Error: {e}")
            return None, None