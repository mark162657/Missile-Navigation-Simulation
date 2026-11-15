"""
Baseline benchmark for the CURRENT (unoptimised) A* pathfinder.

Runs Pathfinding.find_path from src/missile/planning/pathfinding_backend.py,
unchanged, on each DEM and several straight-line route lengths, and records:

    - DEM size, load time, RAM after loading
    - find_path time, peak RAM during the search
    - nodes explored (parsed from the C++ "Target acquired" line)
    - path length in pixels and in km (summed along the path)
    - straight-line distance and detour ratio (path km / straight km)

Each (DEM, length) case runs in its own Python process, so "peak RAM" is the
true peak for that one search and one case cannot inflate the next.

Output:
    - a table in the console
    - tests/results/pathfinding_baseline_<timestamp>.csv   (one row per run)
    - tests/results/pathfinding_baseline_<timestamp>.txt   (readable report)

Usage (from the project root):
    python tests/benchmark_pathfinding_baseline.py
    python tests/benchmark_pathfinding_baseline.py --lengths 5 10 20 --dems srtm_43_02.tif
    python tests/benchmark_pathfinding_baseline.py --weight 2 --timeout 300

Warning: the current code allocates 8 bytes per DEM pixel on EVERY find_path
call. On the Iran tile (~1.57 billion pixels) that is ~12.6 GB per search,
which may push the machine into swap. Close other apps before running it.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_ROOT = PROJECT_ROOT / "src"
RESULTS_DIR = Path(__file__).resolve().parent / "results"

# The three DEMs listed in src/terrain/dem_loader.py
DEFAULT_DEMS = [
    "srtm_43_02.tif",
    "merged_dem_sib_N54_N59_E090_E100.tif",
    "merged_dem_iran_N31_N41_E44_E54_filled.tif",
]
DEFAULT_LENGTHS_KM = [5, 10, 20, 40, 80]
NODATA_LIMIT = -100.0          # same rule as pathfinder.cpp
ROUTE_BEARING_DEG = 30.0       # route direction: 30 deg north of east

CSV_FIELDS = [
    "dem", "rows", "cols", "pixels_million", "target_km", "status",
    "start_row", "start_col", "end_row", "end_col",
    "straight_km", "path_px", "path_km", "detour_ratio",
    "nodes_explored", "load_s", "find_path_s",
    "rss_after_load_mb", "peak_rss_mb", "heuristic_weight", "error",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def peak_rss_mb() -> float:
    """Peak resident memory of this process so far, in MB."""
    import resource
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # macOS reports bytes, Linux reports kilobytes
    return peak / (1024 * 1024) if sys.platform == "darwin" else peak / 1024


def total_ram_gb() -> float | None:
    try:
        return os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 1024 ** 3
    except (ValueError, OSError, AttributeError):
        return None


def pick_route(pf, target_km: float) -> tuple[tuple[int, int], tuple[int, int]] | None:
    """
    Pick a straight route of about target_km centred on the DEM, at a fixed
    bearing. If an end lands on no-data, shift the whole route and try again.
    """
    rows, cols = pf.rows, pf.cols
    mid = rows // 2
    mx = float(pf.meters_per_x_lookup[mid])
    my = float(pf.meter_per_y)

    bearing = math.radians(ROUTE_BEARING_DEG)
    d_col = round(target_km * 1000 * math.cos(bearing) / mx)
    d_row = -round(target_km * 1000 * math.sin(bearing) / my)   # north = smaller row

    # deterministic offsets: centre first, then a spiral of shifts
    step = max(50, int(0.05 * min(rows, cols)))
    offsets = [(0, 0)] + [(dr * step, dc * step)
                          for ring in range(1, 6)
                          for dr in range(-ring, ring + 1)
                          for dc in range(-ring, ring + 1)
                          if max(abs(dr), abs(dc)) == ring]

    for orow, ocol in offsets:
        r0 = mid + orow - d_row // 2
        c0 = cols // 2 + ocol - d_col // 2
        r1, c1 = r0 + d_row, c0 + d_col
        if not (0 <= min(r0, r1) and max(r0, r1) < rows and 0 <= min(c0, c1) and max(c0, c1) < cols):
            continue
        if pf.dem[r0, c0] > NODATA_LIMIT and pf.dem[r1, c1] > NODATA_LIMIT:
            return (int(r0), int(c0)), (int(r1), int(c1))
    return None


def straight_km(pf, a, b) -> float:
    mid_row = (a[0] + b[0]) // 2
    dx = (b[1] - a[1]) * float(pf.meters_per_x_lookup[mid_row])
    dy = (b[0] - a[0]) * float(pf.meter_per_y)
    return math.hypot(dx, dy) / 1000


def path_km(pf, path) -> float:
    total = 0.0
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dx = (c2 - c1) * float(pf.meters_per_x_lookup[r1])
        dy = (r2 - r1) * float(pf.meter_per_y)
        total += math.hypot(dx, dy)
    return total / 1000


# ---------------------------------------------------------------------------
# Worker: one DEM, one route, one fresh process
# ---------------------------------------------------------------------------
def run_worker(dem_name: str, target_km: float, weight: float) -> None:
    sys.path.insert(0, str(SRC_ROOT))
    row = {"dem": dem_name, "target_km": target_km, "heuristic_weight": weight}

    t = time.perf_counter()
    from missile.planning.pathfinding_backend import Pathfinding
    pf = Pathfinding(dem_name)
    row["load_s"] = round(time.perf_counter() - t, 3)
    row["rss_after_load_mb"] = round(peak_rss_mb(), 1)
    row.update(rows=pf.rows, cols=pf.cols,
               pixels_million=round(pf.rows * pf.cols / 1e6, 1))

    route = pick_route(pf, target_km)
    if route is None:
        row.update(status="no_route", error="no valid start/end found inside the DEM")
        print("RESULT " + json.dumps(row), flush=True)
        return
    start, end = route
    row.update(start_row=start[0], start_col=start[1], end_row=end[0], end_col=end[1],
               straight_km=round(straight_km(pf, start, end), 3))
    print("ROUTE " + json.dumps(row), flush=True)   # in case the search is killed

    t = time.perf_counter()
    path = pf.find_path(start, end, heuristic_weight=weight)
    row["find_path_s"] = round(time.perf_counter() - t, 3)
    row["peak_rss_mb"] = round(peak_rss_mb(), 1)

    if path:
        path = [tuple(p) for p in path]
        km = path_km(pf, path)
        row.update(status="ok", path_px=len(path), path_km=round(km, 3),
                   detour_ratio=round(km / row["straight_km"], 3) if row["straight_km"] else None)
    else:
        row.update(status="no_path")
    sys.stdout.flush()
    print("RESULT " + json.dumps(row), flush=True)


# ---------------------------------------------------------------------------
# Parent: run every case, collect, report
# ---------------------------------------------------------------------------
NODES_RE = re.compile(r"Total of (\d+) nodes explored")


def run_case(dem_name: str, target_km: float, weight: float, timeout: int) -> dict:
    cmd = [sys.executable, str(Path(__file__).resolve()), "--worker",
           "--dem", dem_name, "--km", str(target_km), "--weight", str(weight)]
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    env["PYTHONUNBUFFERED"] = "1"

    base = {"dem": dem_name, "target_km": target_km, "heuristic_weight": weight}
    try:
        proc = subprocess.run(cmd, cwd=PROJECT_ROOT, env=env, capture_output=True,
                              text=True, timeout=timeout)
        out, err, code = proc.stdout, proc.stderr, proc.returncode
        timed_out = False
    except subprocess.TimeoutExpired as e:
        out = e.stdout.decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
        err = e.stderr.decode() if isinstance(e.stderr, bytes) else (e.stderr or "")
        code, timed_out = None, True

    row = dict(base)
    for line in out.splitlines():
        if line.startswith("ROUTE ") or line.startswith("RESULT "):
            row.update(json.loads(line.split(" ", 1)[1]))
    m = NODES_RE.search(out)
    if m:
        row["nodes_explored"] = int(m.group(1))

    if timed_out:
        row.update(status="timeout", error=f"over {timeout}s")
    elif code != 0:
        last = (err.strip().splitlines() or ["(no stderr)"])[-1]
        row.update(status="crashed" if code and code < 0 else "error",
                   error=f"exit {code}: {last[:200]}")
    return row


def fmt(v, nd=2):
    if v is None or v == "":
        return "-"
    if isinstance(v, float):
        return f"{v:,.{nd}f}"
    if isinstance(v, int):
        return f"{v:,}"
    return str(v)


def table(rows: list[dict]) -> str:
    cols = [("DEM", "dem"), ("Mpx", "pixels_million"), ("Target km", "target_km"),
            ("Status", "status"), ("Straight km", "straight_km"), ("Path px", "path_px"),
            ("Path km", "path_km"), ("Detour", "detour_ratio"), ("Nodes", "nodes_explored"),
            ("find_path s", "find_path_s"), ("Peak RAM MB", "peak_rss_mb")]
    data = []
    for r in rows:
        dem = r.get("dem", "")
        data.append([dem if len(dem) <= 28 else dem[:25] + "..."] +
                    [fmt(r.get(k)) for _, k in cols[1:]])
    widths = [max(len(h), *(len(d[i]) for d in data)) if data else len(h)
              for i, (h, _) in enumerate(cols)]
    line = "  ".join(h.ljust(w) if i == 0 else h.rjust(w) for i, ((h, _), w) in enumerate(zip(cols, widths)))
    sep = "-" * len(line)
    body = ["  ".join(c.ljust(w) if i == 0 else c.rjust(w) for i, (c, w) in enumerate(zip(d, widths)))
            for d in data]
    return "\n".join([line, sep, *body])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dems", nargs="+", default=DEFAULT_DEMS, help="DEM file names in data/dem")
    ap.add_argument("--lengths", nargs="+", type=float, default=DEFAULT_LENGTHS_KM,
                    help="straight-line route lengths in km")
    ap.add_argument("--weight", type=float, default=1.0,
                    help="heuristic_weight passed to find_path (1.0 = optimal route)")
    ap.add_argument("--timeout", type=int, default=900, help="seconds per case before giving up")
    ap.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--dem", help=argparse.SUPPRESS)
    ap.add_argument("--km", type=float, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.worker:
        run_worker(args.dem, args.km, args.weight)
        return

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = RESULTS_DIR / f"pathfinding_baseline_{stamp}.csv"
    txt_path = RESULTS_DIR / f"pathfinding_baseline_{stamp}.txt"

    ram = total_ram_gb()
    header = [
        "A* pathfinding baseline (current code, no optimisation)",
        f"Date:     {datetime.now():%Y-%m-%d %H:%M:%S}",
        f"Machine:  {platform.platform()}, {platform.machine()}, "
        f"{os.cpu_count()} CPUs, {fmt(ram, 1)} GB RAM",
        f"Python:   {platform.python_version()}",
        f"Weight:   {args.weight}   Bearing: {ROUTE_BEARING_DEG} deg   Timeout: {args.timeout}s per case",
        f"DEMs:     {', '.join(args.dems)}",
        f"Lengths:  {', '.join(fmt(x, 0) for x in args.lengths)} km",
    ]
    print("\n".join(header), "\n", flush=True)

    rows: list[dict] = []
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for dem in args.dems:
            if not (PROJECT_ROOT / "data" / "dem" / dem).exists():
                r = {"dem": dem, "status": "missing", "error": "file not in data/dem"}
                rows.append(r); writer.writerow(r); f.flush()
                print(f"[skip] {dem}: not found in data/dem", flush=True)
                continue
            for km in args.lengths:
                print(f"[run ] {dem}  {fmt(km, 0)} km ...", end=" ", flush=True)
                t = time.perf_counter()
                r = run_case(dem, km, args.weight, args.timeout)
                print(f"{r.get('status')}  ({time.perf_counter() - t:.1f}s wall, "
                      f"peak {fmt(r.get('peak_rss_mb'), 0)} MB)", flush=True)
                rows.append(r)
                writer.writerow(r)
                f.flush()          # keep partial results if a later case is killed

    report = "\n".join(header) + "\n\n" + table(rows) + "\n"

    # Short per-DEM summary
    report += "\nSummary per DEM\n---------------\n"
    for dem in args.dems:
        ok = [r for r in rows if r.get("dem") == dem and r.get("status") == "ok"]
        if not ok:
            report += f"{dem}: no successful runs\n"
            continue
        r0 = ok[0]
        arrays_gb = r0["rows"] * r0["cols"] * 8 / 1024 ** 3
        report += (f"{dem}: {fmt(r0['pixels_million'], 1)} M pixels, "
                   f"g_score+came_from = {arrays_gb:.2f} GB per call; "
                   f"peak RAM {fmt(min(r['peak_rss_mb'] for r in ok), 0)}"
                   f"-{fmt(max(r['peak_rss_mb'] for r in ok), 0)} MB; "
                   f"find_path {fmt(min(r['find_path_s'] for r in ok))}"
                   f"-{fmt(max(r['find_path_s'] for r in ok))} s\n")
    failed = [r for r in rows if r.get("status") != "ok"]
    if failed:
        report += "\nNot completed\n-------------\n"
        for r in failed:
            report += f"{r.get('dem')} {fmt(r.get('target_km'), 0)} km: {r.get('status')} {r.get('error', '')}\n"

    report += ("\nNotes\n-----\n"
               "- Peak RAM is the peak resident memory of a fresh process doing one load + one search.\n"
               "- Nodes = heap pops reported by pathfinder.cpp, including stale entries.\n"
               "- Path km sums the step lengths with the same metres-per-pixel table the C++ uses.\n"
               "- Detour = path km / straight-line km.\n")

    txt_path.write_text(report)
    print("\n" + report)
    print(f"CSV:    {csv_path}\nReport: {txt_path}")


if __name__ == "__main__":
    main()
