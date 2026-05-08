from fastplotlib.graphics.features import _selection_features
import numpy as np

from pathlib import Path
from src.terrain.dem_loader import DEMLoader
from numpy.lib.stride_tricks import sliding_window_view
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
    def __init__(self, location: list[float, float], dem_name: str):
        """
        Args:
            - location: receive the current location of the missile, not a true absolute location
            - update_freq: time interval for update of position (Hz)
        """
        self.location = location

        tif_path = Path(__file__).parent.parent.parent / 'data' / 'dem' / f'{dem_name}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem

        # Get location patch
        self.location_pixel = self.dem_loader.lat_lon_to_pixel(self.location[0], self.location[1]) # first by turning lat/lon to pixel
        self.location_patch = self.dem_loader.get_elevation_patch(self.location_pixel[0], self.location_pixel[1]) # get a patch under the missile

        # Deal with accuracy
        self.lateral_accuracy = 12.0  # meters
        self.vertical_accuracy = 2.5  # meters

    def cross_correlation(self, a_window: np.ndarray, b_sensed_patch: np.ndarray) -> float:
        """
        Manual function implementation for normalized cross-correlation.
        Accepts all windows at once.
        Args:
            a_windows: shape (119, 119, 7, 7)
            b_sensed_patch: shape (7, 7)
        Return:
            ncc_map: shape (119, 119)

        """

        a_mean = a_window.mean(axis=(-2, -1), keepdims=True)
        b_mean = b_sensed_patch.mean()

        a_diff = a_window - a_mean
        b_diff = b_sensed_patch - b_mean

        # Get numerator
        numerator = np.sum((a_diff) * (b_diff), axis=(-2, -1))

        # Square difference
        a_sqr_diff = np.sum((a_diff) ** 2, axis=(-2, -1))
        b_sqr_diff = np.sum((b_diff) ** 2)

        denominator = np.sqrt(a_sqr_diff * b_sqr_diff + 1e-10)

        return numerator / denominator


    def process_update(self, sensed_patch: np.ndarray, est_lat: float, est_lon: float, search_size: int=125) \
            -> tuple[float, float, np.ndarray]:
        """
        We already obtained the normalized sensed patch 7 * 7 grid underneath our missile, now we will
        search for the certain grid size from our tif for pattern and determine where we might have been.

        Args:
            sensed_patch: the smaller patched sensed; default: 7 * 7
            est_lat: 
            est_lon

        """
        # Initialisation
        found_match = False
        best_correlation = 0
        best_offset = (0, 0)

        # Convert INS guess to DEM pixel, define search box boundaries
        center_row, center_col = self.dem_loader.lat_lon_to_pixel(est_lat, est_lon)
        half_search = search_size // 2
        row_start = max(0, center_row - half_search)
        col_start = max(0, center_col - half_search)

        # Load database terrain chunk from INS guess from DEM       
        db_search_patch = self.dem_loader.get_elevation_patch(est_lat, est_lon, search_size, normalized=False)
        snsr_patch_height, snsr_patch_width = sensed_patch.shape

        # Create sliding window by numpy.sliding_window_view and compute NCC
        # window = (199, 199, 7, 7), for example, last two 7 are dimensions 7 * 7
        window = sliding_window_view(db_search_patch, (snsr_patch_height, snsr_patch_width))
        ncc_map = self.cross_correlation(window, sensed_patch) # return score of all each window

        # Find best matching window position
        best_r, best_c = np.unravel_index(np.argmax(ncc_map), ncc_map.shape)
        best_correlation = ncc_map[best_r, best_c]
        found_match = best_correlation > 0.9999

        # Shift from top-left corner -> centre pixel of matched window
        best_offset = (best_r + snsr_patch_height // 2, best_c + snsr_patch_width // 2)
        
        # If matched found, returned the matching lat/lon coordinate and noise covariance
        if found_match:
            matched_row = row_start + best_offset[0]
            matched_col = col_start + best_offset[1]
            matched_lat, matched_lon = self.dem_loader.pixel_to_lat_lon(matched_row, matched_col)
            return matched_lat, matched_lon, self.get_noise_covariance()


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

    dem_path = Path(__file__).parents[2] / "data" / "dem" / "merged_dem_sib_N54_N59_E090_E100.tif"
    dem = DEMLoader(dem_path)
    tercom = TERCOM(location=(54.9, 98.7), dem_name="merged_dem_sib_N54_N59_E090_E100.tif")
    tercom.dem_loader = dem


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