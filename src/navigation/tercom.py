import math
import numpy as np

from src.control.timer import InternalTimer
from src.terrain.dem_loader import DEMLoader
from numpy.lib.stride_tricks import sliding_window_view

class TERCOM:
    """
    TERCOM (Terrain Contour Matching): A navigation system that guides missiles by
    comparing ground elevation profiles measured by radar altimeter with pre-stored
    terrain maps to determine position and correct flight path. Implemented by Kalman Filter.

    The terrain check will be performed every 5 seconds. With following data:
        - db data: from TerrainDatabase.get_elevation_patch
        - sensor data:
    """
    def __init__(self, location: list[float, float], DEM_NAME: str):
        """
        Args:
            - location: receive the current location of the missile, not a true absolute location
            - update_freq: time interval for update of position (Hz)
        """
        self.location = location

        tif_path = Path(__file__).parent.parent.parent / 'data' / 'dem' / f'{DEM_NAME}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem

        # Get location patch
        self.location_pixel = self.dem_loader.lat_lon_to_pixel(self.location[0], self.location[1]) # first by turning lat/lon to pixel
        self.location_patch = self.dem_loader.get_elevation_patch(self.location_pixel[0], self.location_pixel[1]) # get a patch under the missile

        # Deal with accuracy
        self.lateral_accuracy = 12.0  # meters
        self.vertical_accuracy = 2.5  # meters

    def cross_correlation(self, a_greater_patch: np.ndarray, b_sensed_patch: np.ndarray) -> float:
        """
        Manual function implementation for normalized cross-correlation.

        Args:
            a_greater_patch: labelled patch a, the patch in database to compare
            b_sensed_patch: labelled patch b, the elevation patch sensed by missile

        """

        a_mean = np.mean(a_greater_patch)
        b_mean = np.mean(b_sensed_patch)

        a_diff = a_greater_patch - a_mean
        b_diff = b_sensed_patch - b_mean

        # Get numerator
        numerator = np.sum((a_diff) * (b_diff))

        # Square difference
        a_sqr_diff = np.sum((a_diff) ** 2)
        b_sqr_diff = np.sum((b_diff) ** 2)

        denominator = np.sqrt(a_sqr_diff * b_sqr_diff)
        correlation = numerator / denominator

        return correlation


    def process_update(self, sensed_patch: np.ndarray, est_lat: float, est_lon: float, search_size: int=125) \
            -> tuple[float, float, float]:
        """
        We already obtained the normalized sensed patch 7 * 7 grid underneath our missile, now we will
        search for the certain grid size from our tif for pattern and determine where we might have been.

        Complexity:
            - for the nested for loop iteration: O(N^2 * M^2)
        """
        if not self.is_ready():
            return None, None, None

        self.last_update_time = self.timer.get_time_elapsed()

        center_row, center_col = self.dem_loader.lat_lon_to_pixel(est_lat, est_lon)
        half_search = search_size // 2
        row_start = max(0, center_row - half_search)
        col_start = max(0, center_col - half_search)

        db_search_patch = self.dem_loader.get_elevation_patch(est_lat, est_lon, search_size, normalized=False)
        snsr_patch_height, snsr_patch_width = sensed_patch.shape

        best_correlation = -1.0 # -1.0 ~ 1.0
        best_offset = (0, 0)
        found_match = False

        db_row, db_col = db_search_patch.shape
        for r in range(db_row - snsr_patch_height + 1): # vertical movement
            for c in range(db_col - snsr_patch_width + 1): # horizontal
                sub_patch = db_search_patch[r: r + snsr_patch_height, c : c + snsr_patch_width] # extract 7 * 7 chunk

                # Get NCC
                correlation = self.cross_correlation(sub_patch, sensed_patch)

                if correlation > best_correlation:
                    best_correlation = correlation
                    found_match = True
                    best_offset = (r + snsr_patch_height // 2, c + snsr_patch_width // 2) # best matched middle pixel

        if found_match and best_correlation > 0.9999: # adjust this value to define how strict our matching algorithm is
            # Add best_offset (local patch) to the rest of the larger map, to get exact coordinate pixel
            matched_row = row_start + best_offset[0]
            matched_col = col_start + best_offset[1]

            # Turn row/col into lat/lon
            matched_lat, matched_lon = self.dem_loader.pixel_to_lat_lon(matched_row, matched_col)

            return matched_lat, matched_lon, self.get_noise_covariance() # get lat/lon coordinate and also the noise

        return None, None, None

    def get_noise_covariance(self) -> np.ndarray:
        """
        Calculates the noise covariance matrix.
        The noises are represented as a diagonal in the 3D matrix

        """
        return np.diag([
            self.lateral_accuracy ** 2,
            self.lateral_accuracy ** 2,
            self.vertical_accuracy ** 2
        ])

if __name__ == "__main__":
    import time
    from pathlib import Path
    from src.terrain.dem_loader import DEMLoader
    from src.control.timer import InternalTimer

    dem_path = Path(__file__).parents[2] / "data" / "dem" / "merged_dem_sib_N54_N59_E090_E100.tif"
    dem = DEMLoader(dem_path)
    tercom = TERCOM(location=(54.9, 98.7))
    tercom.dem_loader = dem

    tercom.timer.start()

    true_loc = (54.7, 98.6)
    ins_guess = (54.7005, 98.6007)

    sensed_patch = dem.get_elevation_patch(true_loc[0], true_loc[1], 7, True)

    start_bench = time.perf_counter()

    result = tercom.process_update(sensed_patch, ins_guess[0], ins_guess[1], 125)

    end_bench = time.perf_counter()
    duration = (end_bench - start_bench) * 1000  # Convert to milliseconds

    if result and result[0]:
        print(f"Match Found: {result[0]}, {result[1]}")
        print(f"TERCOM Execution Time: {duration:.2f} ms")
    else:
        print("Match Failed!")