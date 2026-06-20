"""
test_physics_nav_integration.py -- physics <-> INS format-compatibility check.

This is a PLAIN SCRIPT (not pytest). Run it directly:

    PYTHONPATH=src python tests/test_physics_nav_integration.py
        (or just `python tests/test_physics_nav_integration.py` -- it inserts
         src/ on sys.path itself, mirroring the other tests in this folder.)

What it does (requirement A):
    1. Builds a MissileState at a known launch position and cruise velocity.
    2. Loads a MissileProfile via config_store.py (falls back to a hand-built
       profile if no JSON store is present).
    3. Runs MissileDynamics.step() for 10 s of sim time at dt = 0.01 s.
    4. Feeds each IMUMeasurement straight into INS.predict() -- no adapter.
    5. Compares the INS dead-reckoned position/velocity/attitude against the
       true state at t = 10 s and prints the error.

The INS here is built CLEAN (zero bias / zero noise) on purpose: that isolates
IMU-format compatibility. If the format matches, the only residual error is the
tiny difference between the truth integrator (RK4) and the INS integrator
(Euler) -- metres, not kilometres. Swap in `profile.create_ins(...)` to instead
watch realistic tactical-grade drift.
"""

import math
import sys
from pathlib import Path

# Add src package root to path (same pattern as tests/test_get_path.py).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np

from missile import config_store
from simulation.physics import MissileDynamics, ControlInput
from missile.profile import BasicSpec, DetailedSpec, MissileProfile
from missile.navigation.ins import INS
from missile.state import MissileState
from terrain import coordinates


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
def load_profile() -> MissileProfile:
    """Load the default profile via config_store; hand-build one if absent."""
    try:
        profile = config_store.get_default_profile()
        if profile is not None:
            return profile
    except Exception as exc:  # noqa: BLE001 -- script-level robustness
        print(f"[warn] config_store unavailable ({exc}); using fallback profile")

    # Fallback Tomahawk-like profile so the script always runs.
    basic = BasicSpec(
        cruise_speed=880.0, min_speed=400.0, max_speed=920.0,
        max_acceleration=9.8, min_altitude=30.0, max_altitude=1500.0,
        max_g_force=6.0, sustained_turn_rate=8.0, sustained_g_force=2.0,
        evasive_turn_rate=25.0, max_range=1600.0,
        cruise_agl_min=20.0, cruise_agl_max=70.0,
    )
    return MissileProfile(name="Fallback", basic=basic, detailed=DetailedSpec())


def horizontal_position_error_m(true_pos, est_pos) -> tuple[float, float, float]:
    """Return (east_err_m, north_err_m, alt_err_m) between [lat,lon,alt] pairs."""
    lat_ref = float(true_pos[0])
    m_lat = coordinates.meter_per_deg_lat(lat_ref)
    m_lon = coordinates.meter_per_deg_lon_at(lat_ref)
    north_err = (est_pos[0] - true_pos[0]) * m_lat
    east_err = (est_pos[1] - true_pos[1]) * m_lon
    alt_err = est_pos[2] - true_pos[2]
    return east_err, north_err, alt_err


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main() -> None:
    profile = load_profile()
    print(f"Profile: {profile.name}  "
          f"(mass {profile.detailed.mass_kg:.0f} kg, "
          f"fuel {profile.detailed.fuel_capacity_kg:.0f} kg)")

    # --- 1. launch state: level cruise, heading due north, 100 m MSL ---
    launch_lat, launch_lon, launch_alt = 36.0, -115.0, 100.0
    cruise_v = profile.basic.cruise_speed_ms  # m/s

    # Trim incidence the airframe holds for level flight (informational; the
    # format check does not depend on flying a perfect trim).
    trim_elevator = 0.064          # rad  (~3.7 deg) -> alpha ~ 0.11 rad
    trim_pitch = 0.11              # rad, matches the trim AoA at level flight

    state = MissileState(
        true_lat=launch_lat, true_lon=launch_lon, true_alt=launch_alt,
        est_lat=launch_lat, est_lon=launch_lon, est_alt=launch_alt,
        vel_east=0.0, vel_north=cruise_v, vel_up=0.0,
        roll=0.0, pitch=trim_pitch, yaw=0.0,
        time=0.0, distance_traveled=0.0, distance_to_target=0.0,
        gps_valid=True, tercom_active=False, ins_calibrated=True,
    )

    # --- 2. dynamics + a CLEAN INS seeded at the true launch state ---
    dynamics = MissileDynamics(profile)
    ins = INS(
        init_pos=[launch_lat, launch_lon, launch_alt],
        init_vel=[state.vel_east, state.vel_north, state.vel_up],
        init_att=[state.roll, state.pitch, state.yaw],
    )

    # Throttle chosen so thrust ~ cruise drag (keeps it roughly level); the
    # format check itself does not depend on a perfect trim.
    control = ControlInput(throttle=0.50, elevator=trim_elevator,
                           rudder=0.0, aileron=0.0)

    # --- 3 & 4. integrate 10 s and feed every IMU sample into the INS ---
    dt = 0.01
    steps = int(round(10.0 / dt))
    for _ in range(steps):
        state, imu = dynamics.step(state, control, dt)
        # Direct hand-off, no adapter (rule 2):
        ins.predict(imu.accel_enu, dt, imu.angular_velocity)

    # --- 5. compare INS estimate vs truth at t = 10 s ---
    ins_pos, ins_vel, ins_att = ins.get_state()
    true_pos = state.true_position()
    true_vel = state.get_velocity()
    true_att = state.get_attitude()

    east_e, north_e, alt_e = horizontal_position_error_m(true_pos, ins_pos)
    horiz_err = math.hypot(east_e, north_e)
    vel_err = float(np.linalg.norm(ins_vel - true_vel))

    print("\n=== Truth vs INS at t = {:.1f} s ===".format(state.time))
    print(f"  true  pos : lat {true_pos[0]:.6f}  lon {true_pos[1]:.6f}  "
          f"alt {true_pos[2]:8.2f} m")
    print(f"  INS   pos : lat {ins_pos[0]:.6f}  lon {ins_pos[1]:.6f}  "
          f"alt {ins_pos[2]:8.2f} m")
    print(f"  pos error : east {east_e:+.3f} m  north {north_e:+.3f} m  "
          f"alt {alt_e:+.3f} m  | horizontal {horiz_err:.3f} m")
    print(f"  true  vel : {true_vel}  (|v| {state.get_speed():.2f} m/s)")
    print(f"  INS   vel : {ins_vel}")
    print(f"  vel error : {vel_err:.4f} m/s")
    # INS normalizes Euler angles to [0, 2pi); MissileState does not. Wrap the
    # difference to [-pi, pi] so the report shows the true (tiny) mismatch.
    def att_err_deg(a: float, b: float) -> float:
        return math.degrees((a - b + math.pi) % (2.0 * math.pi) - math.pi)

    print(f"  att error : roll {att_err_deg(true_att[0], ins_att[0]):+.3f} deg  "
          f"pitch {att_err_deg(true_att[1], ins_att[1]):+.3f} deg  "
          f"yaw {att_err_deg(true_att[2], ins_att[2]):+.3f} deg")

    print(f"\n  distance flown : {state.distance_traveled:.1f} m")
    print(f"  fuel remaining : {dynamics.engine.fuel_remaining_kg:.2f} kg  "
          f"(mass now {dynamics.current_mass_kg:.1f} kg)")

    ok = horiz_err < 50.0 and vel_err < 5.0
    verdict = "COMPATIBLE (small integration-only error)" if ok else \
        "CHECK FORMAT (error too large -- see notes)"
    print(f"\n  IMU<->INS format: {verdict}")


if __name__ == "__main__":
    main()
