"""The unified telemetry frame.

Both the recorded-log replayer and the live simulation runner emit this exact
shape, so the browser has a single schema to render. It mirrors the columns of
`data/logs/*.csv` (see simulation.result.flight_log) plus a couple of derived
convenience fields.
"""
from __future__ import annotations

import math


def _f(v, default=0.0) -> float:
    try:
        if v is None or v == "":
            return default
        return float(v)
    except (TypeError, ValueError):
        return default


def _wrap180(deg: float) -> float:
    """Normalise an angle to (-180, 180].

    Attitude (pitch / roll) is stored wrapped to [0, 360); a slight nose-down of
    -1° is recorded as 359°. Displaying that raw makes a PFD horizon fly off the
    panel, so pitch/roll are normalised here (yaw stays a 0-360 compass heading).
    """
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def frame_from_csv_row(row: dict) -> dict:
    """Turn one FlightLogger CSV row (all strings) into a telemetry frame."""
    true_alt = _f(row.get("true_alt_m"))
    ground = row.get("ground_alt_m")
    agl = row.get("agl_m")
    return {
        "t": _f(row.get("time_s")),
        "stage": row.get("stage") or "PRE_LAUNCHED",
        "true": {
            "lat": _f(row.get("true_lat")),
            "lon": _f(row.get("true_lon")),
            "alt": true_alt,
            "ground_alt": _f(ground) if ground not in (None, "") else None,
            "agl": _f(agl) if agl not in (None, "") else None,
        },
        "est": {
            "lat": _f(row.get("est_lat")),
            "lon": _f(row.get("est_lon")),
            "alt": _f(row.get("est_alt_m")),
        },
        "err": {
            "pos_m": _f(row.get("pos_error_m")),
            "alt_m": _f(row.get("alt_error_m")),
        },
        "vel": {
            "east": _f(row.get("vel_east_ms")),
            "north": _f(row.get("vel_north_ms")),
            "up": _f(row.get("vel_up_ms")),
            "ground_speed": _f(row.get("ground_speed_ms")),
        },
        "att": {
            "roll": _wrap180(_f(row.get("roll_deg"))),
            "pitch": _wrap180(_f(row.get("pitch_deg"))),
            "yaw": _f(row.get("yaw_deg")),
            "fpa": _f(row.get("flight_path_angle_deg")),
        },
        "progress": {
            "traveled_m": _f(row.get("distance_traveled_m")),
            "to_target_m": _f(row.get("distance_to_target_m")) if row.get("distance_to_target_m") else None,
        },
        "flags": {
            "gps": bool(int(_f(row.get("gps_valid")))),
            "tercom": bool(int(_f(row.get("tercom_active")))),
        },
    }


def frame_from_state(state, sim_time: float, ground_alt: float | None,
                     to_target_m: float | None) -> dict:
    """Build a telemetry frame from a live MissileState (live_runner path)."""
    ground_speed = math.hypot(state.vel_east, state.vel_north)
    # atan2 handles a vertical velocity vector correctly (±90 degrees). The
    # previous zero-ground-speed fallback incorrectly reported vertical boost
    # as level flight.
    fpa = math.degrees(math.atan2(state.vel_up, ground_speed))

    m_lat = 111_320.0
    m_lon = 111_320.0 * math.cos(math.radians(state.true_lat))
    north_err = (state.est_lat - state.true_lat) * m_lat
    east_err = (state.est_lon - state.true_lon) * m_lon
    pos_err = math.hypot(north_err, east_err)

    agl = (state.true_alt - ground_alt) if ground_alt is not None else None
    return {
        "t": round(sim_time, 3),
        "stage": state.missile_stage.name,
        "true": {
            "lat": state.true_lat, "lon": state.true_lon, "alt": state.true_alt,
            "ground_alt": ground_alt, "agl": agl,
        },
        "est": {"lat": state.est_lat, "lon": state.est_lon, "alt": state.est_alt},
        "err": {"pos_m": pos_err, "alt_m": state.est_alt - state.true_alt},
        "vel": {
            "east": state.vel_east, "north": state.vel_north, "up": state.vel_up,
            "ground_speed": state.get_ground_speed(),
        },
        "att": {
            "roll": _wrap180(math.degrees(state.roll)), "pitch": _wrap180(math.degrees(state.pitch)),
            "yaw": math.degrees(state.yaw), "fpa": fpa,
        },
        "progress": {"traveled_m": state.distance_traveled, "to_target_m": to_target_m},
        "flags": {"gps": bool(state.gps_valid), "tercom": bool(state.tercom_active)},
    }
