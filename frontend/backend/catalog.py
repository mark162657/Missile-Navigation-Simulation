"""Read-only catalog: missile profiles, recorded flights, and mission results.

A "mission" for the UI is one recorded flight-log CSV (the full time series in
data/logs/). The matching end-of-run verdict JSON in data/results/ is paired by
nearest timestamp, since the two files are stamped at different moments (log at
launch, result at impact) and share no explicit id.
"""
from __future__ import annotations

import csv
import json
import re
from datetime import datetime
from pathlib import Path

from .bootstrap import DATA_ROOT
from .frames import frame_from_csv_row

MISSILE_DIR = DATA_ROOT / "missiles"
LOG_DIR = DATA_ROOT / "logs"
RESULTS_DIR = DATA_ROOT / "results"

_TS = re.compile(r"(\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2})")


def list_profiles() -> list[dict]:
    """Every missile JSON in data/missiles/, returned verbatim."""
    out = []
    for p in sorted(MISSILE_DIR.glob("*.json")):
        try:
            out.append(json.loads(p.read_text()))
        except Exception:  # noqa: BLE001
            continue
    return out


def _stamp(path: Path) -> datetime | None:
    m = _TS.search(path.name)
    if not m:
        return None
    try:
        return datetime.strptime(m.group(1), "%Y-%m-%dT%H-%M-%S")
    except ValueError:
        return None


def _load_results() -> list[tuple[datetime | None, dict, Path]]:
    out = []
    for p in sorted(RESULTS_DIR.glob("*.json")):
        try:
            out.append((_stamp(p), json.loads(p.read_text()), p))
        except Exception:  # noqa: BLE001
            continue
    return out


def _nearest_result(log_stamp: datetime | None, results, max_gap_s: float = 1800.0) -> dict | None:
    """Pair a flight log with the result JSON whose timestamp is closest."""
    if log_stamp is None:
        return None
    best, best_gap = None, max_gap_s
    for stamp, data, _ in results:
        if stamp is None:
            continue
        gap = abs((stamp - log_stamp).total_seconds())
        if gap <= best_gap:
            best, best_gap = data, gap
    return best


def list_missions() -> list[dict]:
    """Summaries of every recorded flight, newest first, each with paired verdict."""
    results = _load_results()
    out = []
    for csv_path in sorted(LOG_DIR.glob("*.csv"), reverse=True):
        summary = _summarise_log(csv_path)
        if summary is None:
            continue
        summary["result"] = _nearest_result(_stamp(csv_path), results)
        out.append(summary)
    return out


def _summarise_log(path: Path) -> dict | None:
    try:
        with path.open(newline="", encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
    except Exception:  # noqa: BLE001
        return None
    if not rows:
        return None
    first, last = rows[0], rows[-1]

    def f(row, key, default=0.0):
        try:
            return float(row.get(key) or default)
        except (TypeError, ValueError):
            return default

    stages = {r.get("stage") for r in rows}
    return {
        "id": path.stem,
        "samples": len(rows),
        "duration_s": f(last, "time_s"),
        "final_stage": last.get("stage"),
        "stages": [s for s in ("BOOST", "CRUISE", "TERMINAL", "IMPACT") if s in stages],
        "start_gps": [f(first, "true_lat"), f(first, "true_lon"), f(first, "true_alt_m")],
        "end_gps": [f(last, "true_lat"), f(last, "true_lon"), f(last, "true_alt_m")],
        "max_alt_m": max(f(r, "true_alt_m") for r in rows),
        "max_speed_ms": max(f(r, "ground_speed_ms") for r in rows),
        "max_pos_error_m": max(f(r, "pos_error_m") for r in rows),
        "distance_flown_m": f(last, "distance_traveled_m"),
    }


def load_flightlog(mission_id: str) -> dict | None:
    """Full telemetry time series for one recorded flight, as frames."""
    path = LOG_DIR / f"{Path(mission_id).name}.csv"
    if not path.exists():
        return None
    with path.open(newline="", encoding="utf-8") as fh:
        frames = [frame_from_csv_row(r) for r in csv.DictReader(fh)]
    if not frames:
        return None
    results = _load_results()
    return {
        "id": path.stem,
        "frames": frames,
        "result": _nearest_result(_stamp(path), results),
    }


def list_results() -> list[dict]:
    return [data for _, data, _ in _load_results()]
