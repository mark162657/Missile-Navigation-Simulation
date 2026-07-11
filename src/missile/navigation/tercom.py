"""
Statement:
    This is a simplified DSMAC. Real DSMAC correlates a downward optical/IR camera
    image of the scene against stored reference imagery. Satellite/optical imagery is not
    available in this simulator at this point, so we approximate the "scene" with a 2D 
    elevation patch from the DEM and use normalized cross-correlation. The matching 
    geometry and output (a horizontal [lat, lon] fix) are the same as real DSMAC; 
    only the sensed modality differs.
"""

import sys
from pathlib import Path

# parents[2] -> src/ (allows: python src/missile/navigation/tercom.py)
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from numpy.lib.stride_tricks import sliding_window_view
from paths import PROJECT_ROOT
from terrain.dem_loader import DEMLoader

class TERCOM:
    """
    TERCOM (Terrain Contour Matching): A navigation system that guides missiles by
    comparing ground elevation profiles measured by radar altimeter with pre-stored
    terrain maps to determine position and correct flight path.

    The terrain check is performed periodically (the rate is set by the
    navigation computer; intended ~every 2 seconds).
    """
    def __init__(self, location: list[float, float], dem_name: str):
        """
        Args:
            location: receive the current location of the missile, not a true absolute location
            dem_name: the name of the DEM file
        """
        self.location = location

        tif_path = PROJECT_ROOT / 'data' / 'dem' / f'{dem_name}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem

        # Deal with accuracy
        self.lateral_accuracy = 12.0  # meters
        self.vertical_accuracy = 2.5  # meters

        # Last-run telemetry (for UI / debugging). Populated by process_update().
        self.last_correlation = 0.0
        self.last_match = False
        self.last_matched_latlon = None  # (lat, lon) of the last accepted fix
        self.last_offset = (0, 0)        # (row, col) offset within the search box
        self.last_search_size = 0

    def cross_correlation(self, a_window: np.ndarray, b_sensed_patch: np.ndarray) -> float:
        """
        Manual function implementation for normalized cross-correlation.
        Accepts all windows at once.
        Args:
            a_window: all the patches segmented by np.sliding_window_view, shape (119, 119, 7, 7)
            b_sensed_patch: the patch that is sensed by radar altimeter of the cruise missile
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
        We already obtained the normalized sensed patch 7 * 7 grid underneath 
        our missile, now we will search for the certain grid size from our tif 
        for the pattern and determine where we might have been.
        Args:
            sensed_patch: the smaller patched sensed; default: 7 * 7
            est_lat, est_lon: the estimated latitude and longitude of the missile
            search_size: the size of the search area in the database; default: 125
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
        # window = (199, 199, 7, 7) - for example, last two 7 are dimensions 7 * 7
        window = sliding_window_view(db_search_patch, (snsr_patch_height, snsr_patch_width))
        ncc_map = self.cross_correlation(window, sensed_patch) # return score of all each window

        # Find best matching window position
        best_r, best_c = np.unravel_index(np.argmax(ncc_map), ncc_map.shape)
        best_correlation = ncc_map[best_r, best_c]
        found_match = best_correlation > 0.9999

        # Shift from top-left corner -> centre pixel of matched window
        best_offset = (best_r + snsr_patch_height // 2, best_c + snsr_patch_width // 2)

        # Record last-run telemetry for the UI / debugging.
        self.last_correlation = float(best_correlation)
        self.last_match = bool(found_match)
        self.last_offset = (int(best_offset[0]), int(best_offset[1]))
        self.last_search_size = int(search_size)

        # If matched found, returned the matching lat/lon coordinate and noise covariance
        if found_match:
            matched_row = row_start + best_offset[0]
            matched_col = col_start + best_offset[1]
            matched_lat, matched_lon = self.dem_loader.pixel_to_lat_lon(matched_row, matched_col)
            self.last_matched_latlon = (float(matched_lat), float(matched_lon))
            return matched_lat, matched_lon, self.get_noise_covariance()

        return None, None, None
        
    def get_noise_covariance(self) -> np.ndarray:
        """
        Calculates the noise covariance matrix.
        The noises are represented as a diagonal in the 3D matrix

        Return:
            The noise covariance matrix in the 3D numpy array.
        """
        return np.diag([
            self.lateral_accuracy ** 2,
            self.lateral_accuracy ** 2,
            self.vertical_accuracy ** 2
        ])

if __name__ == "__main__":
    """
    Test code by Claude to simply conduct mass test for verifying compile time.
    Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
    """
    import time

    dem_path = PROJECT_ROOT / "data" / "dem" / "merged_dem_sib_N54_N59_E090_E100.tif"
    dem = DEMLoader(dem_path)
    tercom = TERCOM(location=(54.9, 98.7), dem_name="merged_dem_sib_N54_N59_E090_E100.tif")
    tercom.dem_loader = dem

    # ── 1. Define DEM valid range ─────────────────────────────────────────
    LAT_MIN, LAT_MAX = 54.5, 58.5   # stay away from edges (avoid out-of-bounds)
    LON_MIN, LON_MAX = 90.5, 99.5

    # ── 2. Test parameters ────────────────────────────────────────────────
    NUM_LOCATIONS  = 20    # how many random true locations to test
    RUNS_PER_LOC   = 5     # how many times to run each location (for timing stability)
    INS_NOISE_DEG  = 0.001 # simulate INS drift (~100m offset)

    np.random.seed(42)     # reproducible random coordinates

    # ── 3. Generate random true locations ────────────────────────────────
    true_lats = np.random.uniform(LAT_MIN, LAT_MAX, NUM_LOCATIONS)
    true_lons = np.random.uniform(LON_MIN, LON_MAX, NUM_LOCATIONS)

    # ── 4. Run mass test ──────────────────────────────────────────────────
    results = []

    for i, (true_lat, true_lon) in enumerate(zip(true_lats, true_lons)):

        # Simulate INS guess with small noise
        ins_lat = true_lat + np.random.uniform(-INS_NOISE_DEG, INS_NOISE_DEG)
        ins_lon = true_lon + np.random.uniform(-INS_NOISE_DEG, INS_NOISE_DEG)

        # Get sensed patch at true location
        try:
            sensed_patch = dem.get_elevation_patch(true_lat, true_lon, 7, True)
        except Exception as e:
            print(f"[{i+1}] Skipped ({true_lat:.4f}, {true_lon:.4f}) — patch load failed: {e}")
            continue

        # Run multiple times for timing stability
        run_times = []
        match_found = False

        for _ in range(RUNS_PER_LOC):
            start = time.perf_counter()
            result = tercom.process_update(sensed_patch, ins_lat, ins_lon, 125)
            end   = time.perf_counter()
            run_times.append((end - start) * 1000)  # ms
            if result[0] is not None:
                match_found = True

        avg_time = np.mean(run_times)
        results.append(avg_time)

        status = "[Y] Match" if match_found else "[N] No match"
        print(f"[{i+1:02d}] ({true_lat:.4f}°N, {true_lon:.4f}°E) | "
              f"{status} | avg: {avg_time:.2f}ms | "
              f"min: {min(run_times):.2f}ms | max: {max(run_times):.2f}ms")

    # ── 5. Summary statistics ─────────────────────────────────────────────
    results = np.array(results)
    print(f"\n{'─'*55}")
    print(f"  Locations tested : {len(results)}")
    print(f"  Runs per location: {RUNS_PER_LOC}")
    print(f"  Mean runtime     : {np.mean(results):.2f} ms")
    print(f"  Std deviation    : {np.std(results):.2f} ms")
    print(f"  Min runtime      : {np.min(results):.2f} ms")
    print(f"  Max runtime      : {np.max(results):.2f} ms")
    print(f"{'─'*55}")