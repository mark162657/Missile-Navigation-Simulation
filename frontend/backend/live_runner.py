"""Drive the real Simulation and yield telemetry frames.

This is the live counterpart to catalog.load_flightlog: instead of replaying a
recorded CSV it steps the actual guidance/physics/navigation loop. It bypasses
Simulation.run()'s interactive `input()` prompts by calling the setup + step
methods directly, and downsamples the 500 Hz tick to a browser-friendly rate.

Live simulation depends on the C++ pathfinder being built and the full loop being
numerically healthy; the WebSocket handler treats any failure here as an error
frame rather than a crash.
"""
from __future__ import annotations

import time
from typing import Callable, Iterator

from . import bootstrap  # noqa: F401 - puts src/ on sys.path


MAX_FLIGHT_TIME_S = 7200.0  # 2 hr hard ceiling (matches SimulationConfig default)


def _pid_dict(pid) -> dict:
    """Snapshot a PIDController's gains + last-tick components for the UI."""
    return {
        "kp": pid.Kp, "ki": pid.Ki, "kd": pid.Kd,
        "p": pid.last_p, "i": pid.last_i, "d": pid.last_d,
        "out": pid.last_output, "error": pid.last_error,
        "integral": pid.integral,
        "out_min": pid.out_min, "out_max": pid.out_max,
    }


def _live_telemetry(sim, state) -> dict:
    """Build the live-only control / TERCOM / navigation block for a frame.

    Everything here is read from the running Simulation's sub-systems, so it is
    only available on the live stream (recorded CSV replays omit it and the UI
    falls back to a note).
    """
    out: dict = {}

    fc = getattr(sim, "flight_computer", None)
    ctrl = getattr(sim, "_last_control", None)

    control: dict = {}
    if ctrl is not None:
        control["throttle"] = float(ctrl.throttle)
        control["accel_turn"] = float(ctrl.accel_turn)
        control["accel_climb"] = float(ctrl.accel_climb)
    if fc is not None:
        control["terminal"] = bool(fc.terminal_latched)
        ap = getattr(fc, "autopilot", None)
        if ap is not None:
            control["target_alt"] = float(ap.last_target_alt)
            control["target_spd"] = float(ap.last_target_spd)
            control["vs_cmd"] = float(ap.last_vs_cmd)
            control["alt_pid"] = _pid_dict(ap.vs_pid)
            control["spd_pid"] = _pid_dict(ap.spd_pid)
    if control:
        out["ctrl"] = control

    nav = getattr(sim, "nav", None)
    if nav is not None:
        tercom = getattr(nav, "tercom", None)
        block = {
            "active": bool(state.tercom_active),
            "roughness_m": float(nav.last_tercom_roughness),
            "threshold_m": float(nav.tercom_roughness_threshold_m),
            "suitable": bool(nav.last_tercom_suitable),
            "fixes": int(nav.tercom_fix_count),
            "period_s": float(nav.tercom_period),
        }
        if tercom is not None:
            block["correlation"] = float(tercom.last_correlation)
            block["match"] = bool(tercom.last_match)
            block["search_size"] = int(tercom.last_search_size)
            block["lateral_acc_m"] = float(tercom.lateral_accuracy)
            if tercom.last_matched_latlon is not None:
                block["matched_lat"] = tercom.last_matched_latlon[0]
                block["matched_lon"] = tercom.last_matched_latlon[1]
        out["tercom"] = block
        out["nav"] = {
            "gps_period_s": float(nav.gps_period),
            "ins_period_s": float(nav.ins_period),
            "gps_fixes": int(nav.gps_fix_count),
        }

    return out


def _opt_float(value, default: float | None = None) -> float | None:
    """Parse a possibly-blank UI value into a float (or None for 'auto')."""
    if value is None or (isinstance(value, str) and value.strip() == ""):
        return default
    return float(value)


def build_config(cfg: dict):
    """Translate a UI config dict into a SimulationConfig."""
    from main import SimulationConfig

    # Operator-chosen flight ceiling, clamped to [1, MAX_FLIGHT_TIME_S]; default max.
    max_flight = _opt_float(cfg.get("max_flight_time_s"), MAX_FLIGHT_TIME_S)
    max_flight = max(1.0, min(float(max_flight), MAX_FLIGHT_TIME_S))

    return SimulationConfig(
        dem_name=cfg["dem_name"],
        start_gps=tuple(cfg["start_gps"]),
        target_gps=tuple(cfg["target_gps"]),
        heuristic_weight=_opt_float(cfg.get("heuristic_weight"), 2.0),
        impact_angle_deg=_opt_float(cfg.get("impact_angle_deg"), -30.0),
        detonation_radius_m=_opt_float(cfg.get("detonation_radius_m"), 25.0),
        wind_speed_ref_ms=_opt_float(cfg.get("wind_speed_ref_ms"), 8.0),
        wind_from_deg=_opt_float(cfg.get("wind_from_deg"), None),  # blank => auto
        max_flight_time_s=max_flight,
        realtime_factor=0.0,  # run as fast as possible; the emitter paces output
        missile_id=cfg.get("missile_id", ""),
    )


def iter_frames(
    profile_name: str,
    cfg: dict,
    emit_dt: float = 0.1,
    on_log: Callable[[str], None] | None = None,
    should_stop: Callable[[], bool] | None = None,
) -> Iterator[dict]:
    """Yield a telemetry frame roughly every `emit_dt` sim-seconds.

    Also yields a leading {'type': 'planning'} log frame while the route is being
    computed, then {'type': 'frame'} telemetry until impact / timeout.

    The core simulation runs as fast as possible (realtime_factor=0), but the
    emitter here paces frame *output* to wall-clock so the browser sees the
    mission unfold at real time (`view_factor` sim-seconds per real second). If
    the sim is too heavy to keep up, no delay is added and it simply runs slower.
    """
    from missile.config_store import get_profile
    from missile.profile import MissileProfile
    from missile.state import FlightStage

    from .frames import frame_from_state

    def log(msg: str) -> None:
        if on_log:
            on_log(msg)

    profile: MissileProfile | None = get_profile(profile_name)
    if profile is None:
        raise RuntimeError(f"Unknown missile profile: {profile_name!r}")

    from main import Simulation
    from simulation.result import FlightLogger

    config = build_config(cfg)
    sim = Simulation(profile, config)

    # Reuse the route the planning screen already computed, if the UI sent it —
    # re-running A* here would just reproduce the same path and stall the launch.
    precomputed = cfg.get("trajectory")
    if precomputed:
        import numpy as np

        sim.trajectory = np.asarray(precomputed, dtype=float)
        log(f"[plan] using route from planner: {len(sim.trajectory):,} waypoints")
    else:
        log(f"[plan] pathfinding over {config.dem_name} ...")
        sim.plan_mission()
        log(f"[plan] trajectory ready: {len(sim.trajectory):,} waypoints")

    sim.state = sim._build_initial_state(config.start_gps)
    sim._pre_ignition_setup()
    sim._ignite()
    log("[launch] boost ignition")

    # Record the same per-flight CSV that Simulation.run() writes (data/logs/*.csv),
    # sampled at 10 Hz, so a live mission also leaves a replayable flight log behind.
    sim.logger = FlightLogger(interval_s=0.1, missile_id=config.missile_id).open()

    dem = sim.pathfinding.dem_loader
    next_emit = 0.0
    csv_path = None

    # Wall-clock pacing: hold each emitted frame until real time catches up to
    # sim_time / view_factor, so the mission plays at (roughly) real time.
    view_factor = _opt_float(cfg.get("view_factor"), 1.0) or 1.0
    view_factor = max(0.05, min(float(view_factor), 50.0))
    wall_start = time.perf_counter()

    def pace_to(sim_t: float) -> None:
        if view_factor <= 0:
            return
        delay = (sim_t / view_factor) - (time.perf_counter() - wall_start)
        if delay > 0:
            time.sleep(min(delay, 1.0))  # cap a single sleep so aborts stay responsive

    def build_frame() -> dict:
        ground = dem.get_elevation(sim.state.true_lat, sim.state.true_lon)
        to_target = sim.target.direct_ground_distance(sim.state)
        frame = frame_from_state(sim.state, sim.sim_time, ground, to_target)
        frame.update(_live_telemetry(sim, sim.state))
        return frame

    try:
        while sim._alive():
            if should_stop and should_stop():
                log("[abort] stopped by operator")
                sim._log_flight(force=True)  # capture the last tick before bailing
                return
            sim.step()
            sim._log_flight()  # rate-limited internally to interval_s
            if sim.sim_time + 1e-9 >= next_emit:
                pace_to(sim.sim_time)  # real-time throttle before releasing the frame
                yield {"type": "frame", "frame": build_frame()}
                next_emit += emit_dt

        # Always emit + log the final (impact) tick, ignoring the sample interval.
        sim._log_flight(force=True)
        yield {"type": "frame", "frame": build_frame()}
    finally:
        csv_path = sim.logger.close()

    if csv_path is not None:
        log(f"[log] flight log saved: {csv_path.name} ({sim.logger.rows_written:,} rows)")

    result = sim._finalise_result()
    log(f"[done] {result.summary()}")
    yield {"type": "result", "result": result.to_dict(),
           "flightlog": (csv_path.name if csv_path is not None else None),
           "trajectory": [[float(p[0]), float(p[1]), float(p[2])] for p in sim.trajectory]}
