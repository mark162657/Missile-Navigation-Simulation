# dem_loader.py — Updated
# Changes from original:
#   [FIX-3c] load_window() added: reads only a pixel sub-region directly from
#            disk using rasterio windowed reads — never loads the full DEM.
#            Use this when you want true disk-level memory savings.
#   Everything else unchanged.

import numpy as np
import matplotlib.pyplot as plt
import rasterio

from pathlib import Path
from rasterio.transform import rowcol, xy
from rasterio.windows import Window               # [FIX-3c] new import
from matplotlib.colors import LinearSegmentedColormap, LightSource


class DEMLoader:
    """Loads and queries elevation data from a single SRTM/merged DEM file."""

    def __init__(self, dem_path: Path) -> None:
        """
        Initialise DEM loader. Reads full DEM into self.data.

        Args:
            dem_path: path to the .tif DEM file
        """
        self.path = Path(dem_path)
        if not self.path.exists():
            raise FileNotFoundError(f"DEM file not found: {self.path}. Check again.")

        with rasterio.open(self.path) as src:
            self.data      = src.read(1)
            self.transform = src.transform
            self.crs       = src.crs
            self.bounds    = src.bounds
            self.shape     = self.data.shape
            self.nodata    = src.nodata

    # -------------------------------------------------------------------------
    # [FIX-3c] Windowed read — reads only a rectangular sub-region from disk
    # -------------------------------------------------------------------------
    def load_window(self, min_row: int, max_row: int,
                    min_col: int, max_col: int) -> np.ndarray:
        """
        Read a sub-region of the DEM directly from disk without loading the
        full file into RAM. Uses rasterio's native Window API.

        This is the true memory-saving approach — self.data is NOT used here.
        Ideal for large DEMs like Iran where full load causes RAM exhaustion.

        Args:
            min_row, max_row: row pixel range (exclusive end)
            min_col, max_col: col pixel range (exclusive end)

        Returns:
            np.ndarray float32 of shape (max_row-min_row, max_col-min_col)
        """
        # rasterio Window: Window(col_offset, row_offset, width, height)
        window = Window(
            col_off = min_col,
            row_off = min_row,
            width   = max_col - min_col,
            height  = max_row - min_row
        )

        with rasterio.open(self.path) as src:
            data = src.read(1, window=window)

        return np.ascontiguousarray(data, dtype=np.float32)

    # -------------------------------------------------------------------------
    # All original methods below — unchanged
    # -------------------------------------------------------------------------

    def get_elevation(self, lat: float, lon: float) -> float | None:
        """Get elevation at GPS coordinates."""
        try:
            row, col = rowcol(self.transform, lon, lat)
            if 0 <= row < self.shape[0] and 0 <= col < self.shape[1]:
                elev = float(self.data[row, col])
                return elev if elev != self.nodata else None
            return None
        except Exception:
            return None

    def get_elevation_patch(self, lat: float, lon: float,
                             patch_size=7, normalized=True) -> np.ndarray:
        """
        Get a patch_size x patch_size elevation patch centred on (lat, lon).
        Used for TERCOM navigation.
        """
        pixel_row, pixel_col = rowcol(self.transform, lon, lat)
        half = patch_size // 2

        row_start = max(0, pixel_row - half)
        row_end   = min(self.shape[0], pixel_row + half + 1)
        col_start = max(0, pixel_col - half)
        col_end   = min(self.shape[1], pixel_col + half + 1)

        if row_start >= row_end or col_start >= col_end:
            return None

        patch = self.data[row_start:row_end, col_start:col_end]

        if normalized:
            patch = self._normalised_patch(patch)

        return patch

    def _normalised_patch(self, patch: np.ndarray) -> np.ndarray:
        """Z-score normalisation."""
        patch    = patch.astype(float)
        mean     = np.mean(patch)
        std_dev  = np.std(patch)
        return (patch - mean) / (std_dev + 1e-6)

    def lat_lon_to_pixel(self, lat: float, lon: float):
        """Convert GPS coordinates to pixel (row, col)."""
        return rowcol(self.transform, lon, lat)

    def pixel_to_lat_lon(self, row, col):
        """Convert pixel (row, col) to GPS (lat, lon)."""
        lon, lat = xy(self.transform, row, col)
        return lat, lon


# Quick test — unchanged
if __name__ == "__main__":
    script_dir   = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    siberia_dem  = "merged_dem_sib_N54_N59_E090_E100.tif"
    dem_path     = project_root / "data" / "dem" / siberia_dem

    dem = DEMLoader(dem_path)
    print(f"\n  DEM loaded: {dem.path.name}")
    print(f"  Shape: {dem.shape}")
    print(f"  Bounds: {dem.bounds}")

    lat, lon = 55.5, 95.0
    elev = dem.get_elevation(lat, lon)

    if elev is not None:
        print(f"  Elevation at ({lat}, {lon}): {elev:.2f}m")

        patch_size = 125
        patch = dem.get_elevation_patch(lat, lon, patch_size=patch_size)

        if patch is not None and patch.size > 0:
            print(f"\n  --- Patch Test ({patch_size}x{patch_size}) ---")
            print(f"  Patch shape: {patch.shape}")
            print(f"  Mean: {patch.mean():.4f} (expected ~0)")
            print(f"  Std Dev: {patch.std():.4f} (expected ~1)")
        else:
            print("  Patch is empty or out of bounds.")
    else:
        print(f"  Coordinate ({lat}, {lon}) outside tile bounds: {dem.bounds}")