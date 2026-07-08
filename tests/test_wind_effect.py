"""
test_wind_effect.py -- wind actually perturbs the trajectory, and the sensor
chain carries it (the payoff of wiring weather.py into dynamics.py).

PLAIN SCRIPT (not pytest). Run:

    python tests/test_wind_effect.py

What it demonstrates (the wind -> trajectory -> IMU -> nav chain):
    1. Two flights with IDENTICAL launch state and IDENTICAL controls:
         A. calm air        (WindField.calm())
         B. steady crosswind + Dryden turbulence from the west (~25 m/s)
       The only difference is the air mass. The windy flight's TRUE ground
       track is pushed sideways -- proof the wind is coupled into the physics.
    2. The windy flight's IMU stream is fed into a CLEAN INS. Because the
       accelerometer senses the wind-induced aero accelerations, the INS
       follows the perturbed truth (small residual) -- proof the disturbance
       reaches the navigation stack only through the sensors (the information
       barrier), exactly as a real airframe would experience it.

If B did NOT diverge from A, wind would not be affecting flight. If the INS
did NOT track B's perturbed truth, the IMU would not be carrying the wind.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np

from missile import config_store
from simulation.physics import MissileDynamics, ControlInput
from simulation.physics.weather import WindField, W20_MODERATE
from missile.profile import BasicSpec, DetailedSpec, MissileProfile
from missile.navigation.ins import INS
from missile.state import MissileState
from terrain import coordinates


def load_profile() -> MissileProfile:
    """Load the default profile via config_store; hand-build one if absent."""
    try:
        profile = config_store.get_default_profile()
        if profile is not None:
            return profile
    except Exception as exc:  # noqa: BLE001 -- script-level robustness
        print(f"[warn] config_store unavailable ({exc}); using fallback profile")
    basic = BasicSpec(
        cruise_speed=880.0, min_speed=400.0, max_speed=920.0,
        max_acceleration=9.8, min_altitude=30.0, max_altitude=1500.0,
        max_g_force=6.0, sustained_turn_rate=8.0, sustained_g_force=2.0,
        evasive_turn_rate=25.0, max_range=1600.0,
        cruise_agl_min=20.0, cruise_agl_max=70.0,
    )
    return MissileProfile(name="Fallback", basic=basic, detailed=DetailedSpec())


def fresh_state(profile: MissileProfile) -> MissileState:
    """Level cruise, heading due north, 100 m MSL (same as the nav test)."""
    cruise_v = profile.basic.cruise_speed_ms
    return MissileState(
        true_lat=36.0, true_lon=-115.0, true_alt=100.0,
        est_lat=36.0, est_lon=-115.0, est_alt=100.0,
        vel_east=0.0, vel_north=cruise_v, vel_up=0.0,
        roll=0.0, pitch=0.0, yaw=0.0,
        time=0.0, distance_traveled=0.0, distance_to_target=0.0,
        gps_valid=True, tercom_active=False, ins_calibrated=True,
    )


def enu_offset_m(ref_pos, pos) -> tuple[float, float]:
    """(east_m, north_m) of `pos` relative to `ref_pos` ([lat,lon,alt])."""
    lat_ref = float(ref_pos[0])
    m_lat = coordinates.meter_per_deg_lat(lat_ref)
    m_lon = coordinates.meter_per_deg_lon_at(lat_ref)
    east = (pos[1] - ref_pos[1]) * m_lon
    north = (pos[0] - ref_pos[0]) * m_lat
    return east, north


def fly(profile, wind: WindField, duration_s: float, dt: float,
        with_ins: bool = False):
    """Integrate one flight; optionally dead-reckon a clean INS alongside."""
    state = fresh_state(profile)
    dynamics = MissileDynamics(profile, wind=wind)
    control = ControlInput(throttle=0.50, accel_climb=9.80665)

    ins = None
    if with_ins:
        ins = INS(
            init_pos=[state.true_lat, state.true_lon, state.true_alt],
            init_vel=[state.vel_east, state.vel_north, state.vel_up],
            init_att=[state.roll, state.pitch, state.yaw],
        )

    for _ in range(int(round(duration_s / dt))):
        state, imu = dynamics.step(state, control, dt)
        if ins is not None:
            ins.predict(imu.accel_enu, dt, imu.angular_velocity)
    return state, ins


def main() -> None:
    profile = load_profile()
    duration, dt = 60.0, 0.01
    print(f"Profile: {profile.name}  |  {duration:.0f} s flight, dt={dt}")
    print("Track: due north from (36.0, -115.0).  Wind B: westerly ~25 m/s "
          "(blows east).\n")

    # A. calm reference.
    calm_state, _ = fly(profile, WindField.calm(), duration, dt)

    # B. steady crosswind from the west + Dryden turbulence (seeded).
    wind = WindField.preset(
        speed_ref=25.0, direction_from_deg=270.0,
        w20=W20_MODERATE, ref_height_m=10.0, seed=1,
    )
    windy_state, windy_ins = fly(profile, wind, duration, dt, with_ins=True)

    # --- trajectory divergence (truth vs truth) ---
    calm_pos = calm_state.true_position()
    windy_pos = windy_state.true_position()
    east_sep, north_sep = enu_offset_m(calm_pos, windy_pos)
    lateral = abs(east_sep)
    print("== 1. wind perturbs the TRUE trajectory ==")
    print(f"  calm  end : east  0.0 m   north {0.0:8.1f} m (ref)")
    print(f"  windy end : east {east_sep:+8.1f} m   north {north_sep:+8.1f} m")
    print(f"  => crosswind pushed the missile {lateral:.0f} m east "
          f"({lateral/1000:.2f} km) off the calm track.\n")

    # --- the INS follows the perturbed truth (sensor barrier) ---
    ins_pos, ins_vel, _ = windy_ins.get_state()
    e_err, n_err = enu_offset_m(windy_pos, ins_pos)
    horiz_err = math.hypot(e_err, n_err)
    print("== 2. the IMU carries the wind to the navigation stack ==")
    print(f"  windy TRUTH pos : lat {windy_pos[0]:.5f}  lon {windy_pos[1]:.5f}")
    print(f"  windy INS   pos : lat {ins_pos[0]:.5f}  lon {ins_pos[1]:.5f}")
    print(f"  clean-INS vs windy-truth horizontal error: {horiz_err:.2f} m")
    print("  => the INS tracked the wind-bent path; the disturbance reached")
    print("     nav ONLY through the IMU (no truth leak).\n")

    assert lateral > 50.0, (
        f"wind should visibly deflect the trajectory; got {lateral:.1f} m")
    assert horiz_err < 50.0, (
        f"clean INS should track the perturbed truth; got {horiz_err:.1f} m")
    print("PASS: wind deflects the path AND the sensor chain carries it.")


if __name__ == "__main__":
    main()
