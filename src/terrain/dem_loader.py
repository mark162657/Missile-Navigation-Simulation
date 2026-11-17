# dem_loader.py — memory-mapped rewrite
#
# Public API is unchanged:
#   DEMLoader(dem_path)
#   .path .data .transform .crs .bounds .shape .nodata
#   .load_window() .get_elevation() .get_elevation_patch()
#   ._normalised_patch() .lat_lon_to_pixel() .pixel_to_lat_lon()
#
# What changed (RAM / speed):
#   1. The full DEM is no longer read into RAM. On first use the GeoTIFF is
#      converted, in ~64 MB row strips, to a float32 .npy cache next to the .tif.
#      Every later run memory-maps that cache: start-up is instant and the OS
#      only loads the pages that are actually touched.
#   2. self.data is that float32 memmap, so pathfinding_backend.py's
#      np.ascontiguousarray(dem.data, dtype=np.float32) is now a zero-copy view
#      (no second copy of the DEM) and the C++ engine reads the file pages directly.
#   3. load_window() slices the memmap instead of reopening the GeoTIFF and
#      decompressing it on every call.
#   4. Unused matplotlib imports removed (they only slowed down import).
#   5. NEW: downsample(k) builds a coarse block map (RAM only) for coarse-to-fine
#      search, after Lee & Yu, "A Coarse-to-Fine Approach for Fast Path Finding
#      for Mobile Robots", IROS 2009.
#
# Notes:
#   - self.data is read-only. Code that writes into dem.data must make a copy.
#   - The cache costs 4 bytes per pixel of disk. It is rebuilt automatically if
#     the .tif is newer or a different size. Add *.npy to .gitignore.
#   - Pass use_cache=False to read the whole DEM into RAM once, as float32, e.g.
#     on a read-only disk.

from __future__ import annotations

import os
import warnings
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rasterio
from rasterio.transform import Affine, rowcol, xy
from rasterio.windows import Window

_CACHE_SUFFIX = ".f32.npy"


@dataclass(frozen=True)
class CoarseDEM:
    """Block map of a DEM (coarse level of coarse-to-fine search)."""
    data: np.ndarray        # float32, shape (ceil(rows/k), ceil(cols/k)); NaN = no data
    k: int                  # block size in fine pixels
    transform: Affine       # geotransform of the coarse grid
    fine_shape: tuple       # (rows, cols) of the original DEM

    @property
    def shape(self) -> tuple:
        return self.data.shape

    def fine_to_coarse(self, row: int, col: int) -> tuple[int, int]:
        """Fine pixel -> index of the block containing it."""
        return row // self.k, col // self.k

    def coarse_to_fine_window(self, brow: int, bcol: int) -> tuple[int, int, int, int]:
        """Block -> (min_row, max_row, min_col, max_col) in fine pixels, clipped.
        Feed straight into DEMLoader.load_window() for the fine search."""
        r0, c0 = brow * self.k, bcol * self.k
        return (r0, min(self.fine_shape[0], r0 + self.k),
                c0, min(self.fine_shape[1], c0 + self.k))


class DEMLoader:
    """Loads and queries elevation data from a single SRTM/merged DEM file."""

    def __init__(self, dem_path: Path | str, use_cache: bool = True) -> None:
        """
        Initialise DEM loader. self.data is a read-only float32 array of the whole
        DEM, memory-mapped from a .npy cache (built on first use), so it costs
        almost no RAM until pixels are read.

        Args:
            dem_path: path to the .tif DEM file, or a DEM filename located in data/dem
            use_cache: True (default) = memory-map a float32 .npy cache.
                       False = read the whole DEM into RAM as float32.
        """
        self.path = self._resolve_path(Path(dem_path))

        # Small GDAL block cache: we never re-read blocks, so a big cache only wastes RAM
        with rasterio.Env(GDAL_CACHEMAX=64), rasterio.open(self.path) as src:
            self.transform = src.transform
            self.crs = src.crs
            self.bounds = src.bounds
            self.nodata = src.nodata
            self.shape = (src.height, src.width)

            if use_cache:
                cache_path = self._cache_path()
                if not self._cache_is_fresh(cache_path):
                    self._build_cache(src, cache_path)
            else:
                # Read straight into float32: one copy, no int16 intermediate kept
                self.data = src.read(1, out_dtype=np.float32)

        if use_cache:
            self.data = np.load(cache_path, mmap_mode="r")

        self._nodata_f32 = (
            np.float32(self.nodata) if self.nodata is not None else None
        )

    # -------------------------------------------------------------------------
    # Setup helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def _resolve_path(requested_path: Path) -> Path:
        if requested_path.exists():
            return requested_path

        project_root = Path(__file__).resolve().parents[2]
        dem_dir_path = project_root / "data" / "dem" / requested_path.name

        if dem_dir_path.exists():
            return dem_dir_path

        raise FileNotFoundError(
            "DEM file not found.\n"
            f"Checked direct path: {requested_path.resolve()}\n"
            f"Checked data/dem path: {dem_dir_path}\n"
            "Place the DEM .tif file in data/dem, or provide the full path."
        )

    def _cache_path(self) -> Path:
        project_root = Path(__file__).resolve().parents[2]  # same root _resolve_path uses
        cache_dir = project_root / "data" / "dem" / "cache"
        cache_dir.mkdir(parents=True, exist_ok=True)  # create data/cache on first run
        return cache_dir / (self.path.stem + _CACHE_SUFFIX)

    def _cache_is_fresh(self, cache_path: Path) -> bool:
        """Cache is valid if it exists, is newer than the .tif, and has the right shape."""
        if not cache_path.exists():
            return False
        if cache_path.stat().st_mtime < self.path.stat().st_mtime:
            return False
        try:
            arr = np.load(cache_path, mmap_mode="r")
            return arr.shape == self.shape and arr.dtype == np.float32
        except Exception:
            return False

    def _build_cache(self, src, cache_path: Path) -> None:
        """
        Convert the GeoTIFF to a float32 .npy in full-width row strips of about
        64 MB, written straight to disk, so the full DEM is never held in RAM.
        Written to a temp file first and renamed, so an interrupted run never
        leaves a half-written cache.
        """
        tmp_path = cache_path.with_name(cache_path.name + ".tmp")
        print(f"  [DEMLoader] Building float32 cache: {cache_path.name} (one-time)")

        rows, cols = self.shape
        block_h = src.block_shapes[0][0] or 1
        strip_h = max(1, (64 * 1024 * 1024) // (cols * 4))
        strip_h = max(block_h, (strip_h // block_h) * block_h)  # whole internal blocks

        try:
            with open(tmp_path, "wb") as f:
                np.lib.format.write_array_header_2_0(
                    f, {"descr": "<f4", "fortran_order": False, "shape": (rows, cols)}
                )
                for r0 in range(0, rows, strip_h):
                    h = min(strip_h, rows - r0)
                    strip = src.read(1, window=Window(0, r0, cols, h),
                                     out_dtype=np.float32)
                    f.write(np.ascontiguousarray(strip, dtype="<f4").tobytes())
            os.replace(tmp_path, cache_path)
        except BaseException:
            tmp_path.unlink(missing_ok=True)
            raise

    # -------------------------------------------------------------------------
    # Windowed read — reads only a rectangular sub-region
    # -------------------------------------------------------------------------
    def load_window(
            self,
            min_row: int,
            max_row: int,
            min_col: int,
            max_col: int
    ) -> np.ndarray:
        """
        Return a sub-region of the DEM as a new C-contiguous float32 array.
        Only the pages of this window are read from disk.

        Args:
            min_row, max_row: row pixel range (exclusive end)
            min_col, max_col: col pixel range (exclusive end)
            Ranges are clipped to the DEM edges.

        Returns:
            np.ndarray float32 of shape (max_row-min_row, max_col-min_col)
            (smaller if the window was clipped)
        """
        rows, cols = self.shape
        r0, r1 = max(0, min_row), min(rows, max_row)
        c0, c1 = max(0, min_col), min(cols, max_col)
        # Copy, so the result is writable, contiguous and independent of the file
        return np.array(self.data[r0:r1, c0:c1], dtype=np.float32, order="C")

    # -------------------------------------------------------------------------
    # Coarse-to-fine downsampling (Lee & Yu, IROS 2009, Sec. II)
    # -------------------------------------------------------------------------
    @staticmethod
    def max_block_size(r: int) -> int:
        """Largest safe k for a robot radius / safety margin of r pixels
        (Lee & Yu 2009, Eq. 2: k <= r + 1)."""
        return r + 1

    def downsample(self, k: int | None = None, reduce: str = "min") -> CoarseDEM | None:
        """
        Build a coarse block map by merging each disjoint k x k block of pixels
        into one cell (ceil(rows/k) x ceil(cols/k)), as in Lee & Yu (2009), Sec. II.
        Edge blocks may be smaller than k x k.

        The result lives in RAM only (it is ~1/k^2 of the DEM) and is memoised
        per (k, reduce). The DEM is read in row strips from the memmap, so the
        full-resolution array is never copied into RAM.

        Args:
            k: block size in pixels. None (or 1) = no downsampling: returns None,
               so callers fall back to the full-resolution dem.data. Otherwise
               k >= 2. Hard limit is min(rows, cols); for the paper's guarantee
               that obstacle structure is preserved keep k <= max_block_size(r).
            reduce: how a block's elevation is chosen.
                "min"  = paper's rule: a block is blocked only if ALL cells are
                         blocked, i.e. it stays "free" if any cell is low/free.
                "max"  = conservative for terrain clearance (keeps peaks).
                "mean" = smooth average.
            nodata pixels are ignored; a block that is entirely nodata is NaN.
        """
        if k is None or k == 1:
            return None  # downsampling switched off
        if not isinstance(k, (int, np.integer)) or k < 2:
            raise ValueError("k must be None (off) or an integer >= 2")
        if k > min(self.shape):
            raise ValueError(f"k={k} exceeds the DEM size {self.shape}")
        fns = {"min": np.nanmin, "max": np.nanmax, "mean": np.nanmean}
        if reduce not in fns:
            raise ValueError(f"reduce must be one of {sorted(fns)}")
        k = int(k)

        cache = self.__dict__.setdefault("_coarse_cache", {})
        if (k, reduce) in cache:
            return cache[(k, reduce)]

        rows, cols = self.shape
        out_r, out_c = -(-rows // k), -(-cols // k)
        out = np.empty((out_r, out_c), dtype=np.float32)
        fn = fns[reduce]

        # whole blocks per strip, ~64 MB of fine pixels at a time
        strip_blocks = max(1, (64 * 1024 * 1024) // (cols * 4 * k))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN blocks
            for b0 in range(0, out_r, strip_blocks):
                b1 = min(out_r, b0 + strip_blocks)
                strip = np.array(self.data[b0 * k:min(rows, b1 * k)], dtype=np.float32)
                if self._nodata_f32 is not None:
                    strip[strip == self._nodata_f32] = np.nan
                # pad edges with NaN so every block is full k x k, then pool
                h, w = strip.shape
                ph, pw = (-h) % k, (-w) % k
                if ph or pw:
                    strip = np.pad(strip, ((0, ph), (0, pw)),
                                   constant_values=np.nan)
                blocks = strip.reshape(strip.shape[0] // k, k,
                                       strip.shape[1] // k, k)
                out[b0:b1] = fn(blocks, axis=(1, 3))

        coarse = CoarseDEM(out, k, self.transform * Affine.scale(k), (rows, cols))
        cache[(k, reduce)] = coarse
        return coarse

    # -------------------------------------------------------------------------
    # Queries
    # -------------------------------------------------------------------------
    def _to_pixel(self, lat: float, lon: float) -> tuple[int, int]:
        """(row, col) for a GPS point, identical to rasterio's rowcol()."""
        row, col = rowcol(self.transform, lon, lat)
        return int(row), int(col)

    def get_elevation(self, lat: float, lon: float) -> float | None:
        """Get elevation at GPS coordinates."""
        try:
            row, col = self._to_pixel(lat, lon)
            if 0 <= row < self.shape[0] and 0 <= col < self.shape[1]:
                elev = self.data[row, col]
                if self._nodata_f32 is not None and elev == self._nodata_f32:
                    return None
                return float(elev)
            return None
        except Exception:
            return None

    def get_elevation_patch(self, lat: float, lon: float,
                            patch_size=7, normalized=True) -> np.ndarray:
        """
        Get a patch_size x patch_size elevation patch centred on (lat, lon).
        Used for TERCOM navigation.
        """
        pixel_row, pixel_col = self._to_pixel(lat, lon)
        half = patch_size // 2

        row_start = max(0, pixel_row - half)
        row_end = min(self.shape[0], pixel_row + half + 1)
        col_start = max(0, pixel_col - half)
        col_end = min(self.shape[1], pixel_col + half + 1)

        if row_start >= row_end or col_start >= col_end:
            return None

        patch = self.data[row_start:row_end, col_start:col_end]

        if normalized:
            return self._normalised_patch(patch)

        # Copy so callers get a normal writable array, not a view of the file
        return np.array(patch)

    def _normalised_patch(self, patch: np.ndarray) -> np.ndarray:
        """Z-score normalisation."""
        patch = np.asarray(patch, dtype=np.float64)
        mean = patch.mean()
        std_dev = patch.std()
        return (patch - mean) / (std_dev + 1e-6)

    def lat_lon_to_pixel(self, lat: float, lon: float):
        """Convert GPS coordinates to pixel (row, col)."""
        return rowcol(self.transform, lon, lat)

    def pixel_to_lat_lon(self, row, col):
        """Convert pixel (row, col) to GPS (lat, lon)."""
        lon, lat = xy(self.transform, row, col)
        return lat, lon


if __name__ == "__main__":
    # parents[2] -> project root
    project_root = Path(__file__).resolve().parents[2]
    siberia_dem = "merged_dem_sib_N54_N59_E090_E100.tif"
    iran_dem = "merged_dem_iran_N31_N41_E44_E54_filled.tif"
    sample_dem = "srtm_43_02.tif"

    dem_path = project_root / "data" / "dem" / sample_dem

    dem = DEMLoader(dem_path)
    print(f"\n  DEM loaded: {dem.path.name}")
    print(f"  Shape: {dem.shape}")
    print(f"  Bounds: {dem.bounds}")

    coarse = dem.downsample(4)
    print(f"  Coarse (k=4) shape: {coarse.shape}")

    lat, lon = 38.57670, 50.99988
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
        # Distinguish "outside the tile" from "inside the tile but nodata"
        # (e.g. an unfilled sea pixel) — these are very different failures.
        b = dem.bounds
        in_bounds = (b.left <= lon <= b.right) and (b.bottom <= lat <= b.top)
        if in_bounds:
            print(f"  Coordinate ({lat}, {lon}) is IN bounds but has no data "
                  f"(nodata={dem.nodata}). If this is sea, run fill_nodata.py.")
        else:
            print(f"  Coordinate ({lat}, {lon}) outside tile bounds: {dem.bounds}")
