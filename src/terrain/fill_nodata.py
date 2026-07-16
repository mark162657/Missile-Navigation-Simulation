# fill_nodata.py — Fill nodata (sea / SRTM void) pixels in a DEM.
#
# Why this exists
# ---------------
# SRTM leaves oceans and large water bodies (e.g. the Caspian Sea) as nodata
# (-32768). The C++ pathfinder in cpp/pathfinder.cpp treats ANY elevation
# <= -100 m as impassable (returns infinite movement cost). So a launch or
# target placed on the Caspian sits on an impassable "wall" and A* returns no
# path — the missile can neither leave nor enter a sea pixel.
#
# This script replaces nodata with a sea-surface elevation (default -29 m, the
# uniform Caspian shoreline value in this tile) so those pixels become passable
# and attractive to the terrain-follower. It writes a NEW tiled + compressed
# GeoTIFF; the original file is never modified.
#
# Memory
# ------
# Processes the DEM in horizontal row-chunks via rasterio windows, so it never
# holds the full 39601x39601 array in RAM (important on a 16 GB machine). The
# output is internally tiled (512x512) + DEFLATE-compressed, which also speeds
# up the windowed reads used elsewhere (DEMLoader.load_window).
#
# Usage
# -----
#   python3 src/terrain/fill_nodata.py                       # default Iran tile
#   python3 src/terrain/fill_nodata.py <name.tif> --sea -29  # explicit
#   python3 src/terrain/fill_nodata.py <name.tif> --out <out_name.tif>
#
# Note on non-sea voids
# ---------------------
# Any nodata pixel is filled with --sea. In this tile 94.8% of nodata is the
# Caspian (border uniformly -29 m); a small ~5% void near (40.5N, 54.6E) in
# Turkmenistan is also set to -29. That region is far from any Caspian-launch
# Iran mission, so a flat fill there is harmless. If you ever need that void
# filled with local terrain instead, handle it as a separate localized pass.

import argparse
import sys
from pathlib import Path

import numpy as np
import rasterio
from rasterio.windows import Window

# Project root: src/terrain/fill_nodata.py -> parents[2]
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEM_DIR = PROJECT_ROOT / "data" / "dem"

DEFAULT_DEM = "merged_dem_iran_N31_N41_E44_E54.tif"
DEFAULT_SEA_VALUE = -29          # metres; Caspian shoreline elevation in this tile
ROW_CHUNK = 1024                 # rows read/written per window (~80 MB per chunk)


def fill_dem(in_path: Path, out_path: Path, sea_value: int = DEFAULT_SEA_VALUE) -> dict:
    """
    Stream-copy `in_path` to `out_path`, replacing every nodata pixel with
    `sea_value`. Returns a small stats dict for logging/verification.
    """
    with rasterio.open(in_path) as src:
        nodata = src.nodata
        if nodata is None:
            raise ValueError(
                f"{in_path.name} declares no nodata value — nothing to fill. "
                "If sea is coded as some sentinel, pass it explicitly."
            )

        dtype = src.dtypes[0]
        sea_cast = np.array(sea_value).astype(dtype)  # ensure it fits the band dtype

        profile = src.profile.copy()
        profile.update(
            tiled=True,
            blockxsize=512,
            blockysize=512,
            compress="deflate",
            predictor=2,            # horizontal differencing — great for elevation
            bigtiff="if_safer",
            # Keep the original nodata tag for metadata; no pixels will use it
            # after the fill, so downstream code stops seeing walls either way.
            nodata=nodata,
        )

        filled_pixels = 0
        total_pixels = src.width * src.height

        with rasterio.open(out_path, "w", **profile) as dst:
            for row0 in range(0, src.height, ROW_CHUNK):
                h = min(ROW_CHUNK, src.height - row0)
                win = Window(0, row0, src.width, h)

                arr = src.read(1, window=win)
                mask = arr == nodata
                filled_pixels += int(mask.sum())
                arr[mask] = sea_cast

                dst.write(arr, 1, window=win)

                done = min(row0 + h, src.height)
                pct = 100.0 * done / src.height
                print(f"\r  filling... {pct:5.1f}%  ({done}/{src.height} rows)",
                      end="", flush=True)

        print()  # newline after progress line

    return {
        "nodata": nodata,
        "dtype": dtype,
        "sea_value": int(sea_cast),
        "filled_pixels": filled_pixels,
        "total_pixels": total_pixels,
        "filled_pct": 100.0 * filled_pixels / total_pixels,
    }


def verify(out_path: Path, nodata) -> None:
    """Sanity-check the output: confirm no nodata remains (sampled) and report range."""
    with rasterio.open(out_path) as dst:
        # Sample a coarse overview to keep memory low.
        sample = dst.read(1, out_shape=(dst.height // 20, dst.width // 20))
        remaining = int(np.sum(sample == nodata))
        print(f"  verify: sampled residual nodata pixels = {remaining} (expect 0)")
        print(f"  verify: elevation range min {sample.min()} / max {sample.max()} m")


def _resolve_in_path(name_or_path: str) -> Path:
    p = Path(name_or_path)
    if p.exists():
        return p
    candidate = DEM_DIR / p.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(
        f"DEM not found. Checked '{p}' and '{candidate}'."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Fill nodata (sea) pixels in a DEM.")
    parser.add_argument("dem", nargs="?", default=DEFAULT_DEM,
                        help="DEM filename in data/dem or a full path "
                             f"(default: {DEFAULT_DEM})")
    parser.add_argument("--sea", type=int, default=DEFAULT_SEA_VALUE,
                        help=f"elevation to write for nodata (default {DEFAULT_SEA_VALUE} m)")
    parser.add_argument("--out", default=None,
                        help="output filename (default: <name>_filled.tif in data/dem)")
    args = parser.parse_args()

    in_path = _resolve_in_path(args.dem)
    if args.out:
        out_path = Path(args.out)
        if not out_path.is_absolute():
            out_path = DEM_DIR / out_path.name
    else:
        out_path = in_path.with_name(in_path.stem + "_filled.tif")

    if out_path.resolve() == in_path.resolve():
        print("Refusing to overwrite the source DEM. Choose a different --out.")
        return 1

    print(f"Input : {in_path}")
    print(f"Output: {out_path}")
    print(f"Sea   : {args.sea} m")

    stats = fill_dem(in_path, out_path, sea_value=args.sea)
    print(f"  filled {stats['filled_pixels']:,} / {stats['total_pixels']:,} "
          f"pixels ({stats['filled_pct']:.2f}%) with {stats['sea_value']} m")

    verify(out_path, stats["nodata"])

    size_gb = out_path.stat().st_size / 1e9
    print(f"  wrote {out_path.name} ({size_gb:.2f} GB, tiled + deflate)")
    print("\nDone. Point your mission's dem_name at the filled file to unblock "
          "pathfinding over the sea.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
