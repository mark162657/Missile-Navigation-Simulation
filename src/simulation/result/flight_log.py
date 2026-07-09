"""
flight_log.py -- per-flight telemetry recorder (one CSV per run).

Where MissionResult is the single end-of-run verdict, FlightLogger captures the
TIME SERIES of a flight: it samples the major state every `interval_s` seconds
and writes one labelled CSV row per sample. CSV so it drops straight into pandas
/ Excel / any plotter, with the header row as the column labels.

Sampling rate: 0.1 s (10 Hz) by default -- fine enough to plot the trajectory and
the INS drift, coarse enough that even a long flight stays a few MB. Pass
interval_s=0.5 for lighter 2 Hz logs.

Rows are written incrementally (the file stays open across the flight), so a run
that crashes mid-flight still leaves everything logged up to that point.

Usage from the sim loop:

    with FlightLogger(interval_s=0.1, missile_id=cfg.missile_id) as log:
        while sim.alive():
            sim.step()
            log.record(sim.state, sim.sim_time,
                       distance_to_target_m=sim.target.direct_ground_distance(sim.state))
    # -> data/logs/<missile_id>_<timestamp>.csv
"""
from __future__ import annotations

import csv
import math
from datetime import datetime, timezone
from pathlib import Path

from paths import PROJECT_ROOT
from missile.state import MissileState
from terrain import coordinates

# Default directory for flight-log CSVs (separate from the outcome JSONs).
DEFAULT_LOG_DIR = PROJECT_ROOT / "data" / "logs"

# Column order == the CSV header labels.
_FIELDS = [
    "time_s",
    "stage",
    # truth (ground truth from the physics plant)
    "true_lat", "true_lon", "true_alt_m",
    # terrain under the missile + height above ground
    "ground_alt_m", "agl_m",
    # navigation estimate (INS + KF, corrected by GPS/TERCOM)
    "est_lat", "est_lon", "est_alt_m",
    # navigation error: estimate vs truth
    "pos_error_m", "alt_error_m",
    # velocity
    "vel_east_ms", "vel_north_ms", "vel_up_ms", "ground_speed_ms",
    # attitude + flight path
    "roll_deg", "pitch_deg", "yaw_deg", "flight_path_angle_deg",
    # progress
    "distance_traveled_m", "distance_to_target_m",
    # nav flags
    "gps_valid", "tercom_active",
]


class FlightLogger:
    """Samples MissileState every `interval_s` and writes one CSV row per sample."""

    def __init__(
        self,
        interval_s: float = 0.1,
        directory: str | Path | None = None,
        filename: str | None = None,
        missile_id: str = "",
    ) -> None:
        """
        Args:
            interval_s: seconds between samples (0.1 = 10 Hz, 0.5 = 2 Hz).
            directory:  target folder (default: data/logs/). Created on open().
            filename:   override the file name. Default is
                        "<missile_id or 'flight'>_<timestamp>.csv".
            missile_id: stamped into the default filename.
        """
        if interval_s <= 0.0:
            raise ValueError("interval_s must be > 0")

        self.interval_s = float(interval_s)
        self.missile_id = missile_id
        self._dir = Path(directory) if directory is not None else DEFAULT_LOG_DIR

        stamp = datetime.now(timezone.utc).isoformat().replace(":", "-")
        stem = missile_id or "flight"
        self._filename = filename or f"{stem}_{stamp}.csv"

        self.path: Path | None = None
        self._file = None
        self._writer: csv.DictWriter | None = None
        self._next_sample = 0.0     # sim_time of the next row to write
        self._last_time: float | None = None   # sim_time of the last row written
        self._rows_written = 0

    # ------------------------------------------------------------------
    # Lifecycle (also usable as a context manager)
    # ------------------------------------------------------------------
    def open(self) -> "FlightLogger":
        """Create the file and write the header row."""
        self._dir.mkdir(parents=True, exist_ok=True)
        self.path = self._dir / self._filename
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=_FIELDS)
        self._writer.writeheader()
        return self

    def close(self) -> Path | None:
        """Flush and close the file; returns the path written."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
        return self.path

    def __enter__(self) -> "FlightLogger":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def record(
        self,
        state: MissileState,
        sim_time: float,
        distance_to_target_m: float | None = None,
        ground_alt_m: float | None = None,
        force: bool = False,
    ) -> None:
        """
        Write a row if `interval_s` has elapsed since the last sample.

        Call this every tick; the rate-limiting is handled here. `sim_time` is the
        loop clock. Pass `force=True` to write regardless of the interval -- used
        for the FINAL (impact) tick so it is never dropped just because it did not
        land on a sample boundary; a forced write that repeats the last row's
        timestamp is skipped so the final row is never duplicated.

        `distance_to_target_m` / `ground_alt_m` are optional passthroughs (the
        logger stays decoupled from TargetGeometry / the DEM).
        """
        if self._writer is None:
            raise RuntimeError("FlightLogger.open() must be called before record().")
        if not force and sim_time + 1e-9 < self._next_sample:
            return  # not time for the next sample yet
        if self._last_time is not None and abs(sim_time - self._last_time) < 1e-9:
            return  # this exact tick is already logged (no duplicate final row)

        self._writer.writerow(self._row(state, sim_time, distance_to_target_m, ground_alt_m))
        self._rows_written += 1
        self._last_time = sim_time
        # Advance to the next boundary, skipping any we jumped past (dt > interval).
        while self._next_sample <= sim_time + 1e-9:
            self._next_sample += self.interval_s

    @property
    def rows_written(self) -> int:
        return self._rows_written

    # ------------------------------------------------------------------
    def _row(self, s: MissileState, t: float, dist_to_target: float | None,
             ground_alt: float | None = None) -> dict:
        # Navigation error: estimate vs truth, in meters (local ENU scaling).
        m_lat = coordinates.meter_per_deg_lat(s.true_lat)
        m_lon = coordinates.meter_per_deg_lon_at(s.true_lat)
        north_err = (s.est_lat - s.true_lat) * m_lat
        east_err = (s.est_lon - s.true_lon) * m_lon
        pos_error_m = math.hypot(north_err, east_err)

        ground_speed = math.hypot(s.vel_east, s.vel_north)
        flight_path_angle = math.degrees(math.atan2(s.vel_up, ground_speed)) if ground_speed else 0.0

        # Terrain under the missile + height above ground (blank if no DEM reading).
        if ground_alt is None or not math.isfinite(ground_alt):
            ground_alt_m = ""
            agl_m = ""
        else:
            ground_alt_m = round(float(ground_alt), 2)
            agl_m = round(s.true_alt - float(ground_alt), 2)

        return {
            "time_s": round(t, 3),
            "stage": s.missile_stage.name,
            "true_lat": round(s.true_lat, 7),
            "true_lon": round(s.true_lon, 7),
            "true_alt_m": round(s.true_alt, 2),
            "ground_alt_m": ground_alt_m,
            "agl_m": agl_m,
            "est_lat": round(s.est_lat, 7),
            "est_lon": round(s.est_lon, 7),
            "est_alt_m": round(s.est_alt, 2),
            "pos_error_m": round(pos_error_m, 2),
            "alt_error_m": round(s.est_alt - s.true_alt, 2),
            "vel_east_ms": round(s.vel_east, 3),
            "vel_north_ms": round(s.vel_north, 3),
            "vel_up_ms": round(s.vel_up, 3),
            "ground_speed_ms": round(s.get_ground_speed(), 3),
            "roll_deg": round(math.degrees(s.roll), 2),
            "pitch_deg": round(math.degrees(s.pitch), 2),
            "yaw_deg": round(math.degrees(s.yaw), 2),
            "flight_path_angle_deg": round(flight_path_angle, 2),
            "distance_traveled_m": round(s.distance_traveled, 2),
            "distance_to_target_m": (round(dist_to_target, 2)
                                     if dist_to_target is not None else ""),
            "gps_valid": int(bool(s.gps_valid)),
            "tercom_active": int(bool(s.tercom_active)),
        }
