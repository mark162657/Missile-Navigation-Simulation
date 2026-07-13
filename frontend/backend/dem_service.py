"""DEM elevation grids for the map underlay and the 3D terrain viewer.

Reads a GeoTIFF via rasterio and returns a coarse elevation grid plus geographic
bounds. The grid is deliberately downsampled (default max 180x180) so it streams
to the browser as a compact JSON payload and renders as light wireframe / hillshade
without touching the GPU-heavy full-resolution data.
"""
from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling

from .bootstrap import DATA_ROOT

DEM_DIR = DATA_ROOT / "dem"


def list_dems() -> list[dict]:
    """Return metadata for every DEM tile in data/dem/ (no pixel data)."""
    out: list[dict] = []
    for tif in sorted(DEM_DIR.glob("*.tif")):
        try:
            out.append(_dem_meta(tif))
        except Exception as exc:  # noqa: BLE001 - surface a broken tile, keep the rest
            out.append({"name": tif.name, "error": str(exc)})
    return out


def _dem_meta(path: Path) -> dict:
    with rasterio.open(path) as src:
        b = src.bounds
        return {
            "name": path.name,
            "width": src.width,
            "height": src.height,
            "crs": str(src.crs),
            "bounds": {"west": b.left, "south": b.bottom, "east": b.right, "north": b.top},
            "resolution_deg": abs(src.transform[0]),
        }


@lru_cache(maxsize=8)
def elevation_grid(name: str, max_size: int = 180) -> dict:
    """Downsampled elevation grid for a whole DEM tile.

    Returns row-major elevations (north->south, west->east), the geographic
    bounds, and min/max for colour/height scaling. Cached because the browser
    re-requests the same tile across screens.
    """
    path = _resolve(name)
    with rasterio.open(path) as src:
        scale = max(1, math.ceil(max(src.width, src.height) / max_size))
        out_h = max(2, src.height // scale)
        out_w = max(2, src.width // scale)
        data = src.read(
            1,
            out_shape=(out_h, out_w),
            resampling=Resampling.average,
        ).astype("float32")
        b = src.bounds
        nodata = src.nodata

    if nodata is not None:
        data = np.where(data == nodata, np.nan, data)
    # Fill voids with the tile mean so the mesh has no holes.
    if np.isnan(data).any():
        data = np.where(np.isnan(data), np.nanmean(data), data)

    finite = data[np.isfinite(data)]
    z_min = float(finite.min()) if finite.size else 0.0
    z_max = float(finite.max()) if finite.size else 0.0

    return {
        "name": name,
        "rows": out_h,
        "cols": out_w,
        "bounds": {"west": b.left, "south": b.bottom, "east": b.right, "north": b.top},
        "z_min": z_min,
        "z_max": z_max,
        # Round to whole metres: elevation precision the viewer never needs beyond 1 m.
        "elev": np.round(data).astype("int16").flatten().tolist(),
    }


def elevation_at(name: str, lat: float, lon: float) -> float | None:
    """Single-point elevation lookup (used when the UI drops a start/target pin)."""
    path = _resolve(name)
    with rasterio.open(path) as src:
        b = src.bounds
        if not (b.left <= lon <= b.right and b.bottom <= lat <= b.top):
            return None
        row, col = src.index(lon, lat)
        if not (0 <= row < src.height and 0 <= col < src.width):
            return None
        val = float(src.read(1, window=((row, row + 1), (col, col + 1)))[0, 0])
        if src.nodata is not None and val == src.nodata:
            return None
        return val


def _resolve(name: str) -> Path:
    path = DEM_DIR / Path(name).name
    if not path.exists():
        raise FileNotFoundError(f"DEM not found: {name}")
    return path
