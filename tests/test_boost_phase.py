"""
test_boost_phase.py -- boost -> cruise staging + INS-through-boost check.

PLAIN SCRIPT (not pytest). Run:

    python tests/test_boost_phase.py

What it shows:
    1. A vertical (surface-VLS) launch: booster fires, the missile climbs and
       pitches over via the programmed schedule.
    2. The FlightSequencer separating the booster at burnout (mass step-drop)
       and handing off to the cruise turbofan dynamics.
    3. A CLEAN INS fed the boost IMU stream directly -- demonstrating the nav
       stack survives the high-acceleration boost transient (the boost is a
       useful stress test for the INS/KF, which is the point of the project).

Switch LAUNCH to LaunchMode.GROUND or LaunchMode.SUBMARINE to compare.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np

from missile import config_store
from simulation.physics import (
    MissileDynamics, ControlInput, FlightSequencer, LaunchMode,
)
from missile.profile import BasicSpec, DetailedSpec, MissileProfile
from missile.navigation.ins import INS
from missile.state import FlightStage, MissileState
from terrain import coordinates


LAUNCH = LaunchMode.SURFACE_VLS   # try GROUND or SUBMARINE too


def load_profile() -> MissileProfile:
    try:
        p = config_store.get_default_profile()
        if p is not None:
            return p
    except Exception:
        pass
    basic = BasicSpec(
        cruise_speed=880.0, min_speed=400.0, max_speed=920.0,
        max_acceleration=9.8, min_altitude=30.0, max_altitude=1500.0,
        max_g_force=6.0, sustained_turn_rate=8.0, sustained_g_force=2.0,
        evasive_turn_rate=25.0, max_range=1600.0,
        cruise_agl_min=20.0, cruise_agl_max=70.0,
    )
    return MissileProfile(name="Fallback", basic=basic, detailed=DetailedSpec())


def pos_err_m(true_pos, est_pos) -> float:
    lat_ref = float(true_pos[0])
    m_lat = coordinates.meter_per_deg_lat(lat_ref)
    m_lon = coordinates.meter_per_deg_lon_at(lat_ref)
    dn = (est_pos[0] - true_pos[0]) * m_lat
    de = (est_pos[1] - true_pos[1]) * m_lon
    da = est_pos[2] - true_pos[2]
    return math.sqrt(dn * dn + de * de + da * da)


def main() -> None:
    profile = load_profile()

    # Vertical modes launch near 88 deg pitch; GROUND launches shallow.
    launch_pitch = math.radians(12.0 if LAUNCH is LaunchMode.GROUND else 88.0)
    launch_lat, launch_lon, launch_alt = 36.0, -115.0, 5.0  # ~deck height

    state = MissileState(
        true_lat=launch_lat, true_lon=launch_lon, true_alt=launch_alt,
        est_lat=launch_lat, est_lon=launch_lon, est_alt=launch_alt,
        vel_east=0.0, vel_north=0.0, vel_up=0.0,          # starts at rest
        roll=0.0, pitch=launch_pitch, yaw=0.0,            # points (near) up, north
        time=0.0, distance_traveled=0.0, distance_to_target=0.0,
        gps_valid=True, tercom_active=False, ins_calibrated=True,
    )

    # Booster spec + default launch mode now come from the profile (profile=).
    sequencer = FlightSequencer(launch_mode=LAUNCH, cruise_heading_rad=0.0,
                                profile=profile)
    dynamics = MissileDynamics(profile, sequencer=sequencer)

    ins = INS(
        init_pos=[launch_lat, launch_lon, launch_alt],
        init_vel=[0.0, 0.0, 0.0],
        init_att=[state.roll, state.pitch, state.yaw],
    )

    # Cruise command applied once the turbofan takes over (ignored during boost).
    # No control surfaces: ~g of vertical accel holds level once cruising.
    cruise_control = ControlInput(throttle=0.55, accel_climb=9.80665)

    dt = 0.01
    total_t = 25.0
    steps = int(round(total_t / dt))

    print(f"Launch mode: {LAUNCH.name}   profile: {profile.name}")
    print(f"Booster: {sequencer.booster.spec.booster_thrust_N/1000:.0f} kN for "
          f"{sequencer.booster.spec.burn_time_s:.0f} s, "
          f"{sequencer.booster.total_mass:.0f} kg total\n")
    header = (f"{'t(s)':>5} {'stage':>7} {'alt(m)':>8} {'V(m/s)':>7} "
              f"{'pitch':>6} {'mass(kg)':>8} {'INSerr(m)':>9}")
    print(header)
    print("-" * len(header))

    separated_reported = False
    prev_stage = sequencer.stage
    for i in range(steps):
        control = cruise_control if sequencer.stage != FlightStage.BOOST \
            else ControlInput()
        state, imu = dynamics.step(state, control, dt)
        ins.predict(imu.accel_enu, dt, imu.angular_velocity)

        # Announce the separation event the first time it happens.
        if (not separated_reported and prev_stage == FlightStage.BOOST
                and sequencer.stage == FlightStage.CRUISE):
            print(f"  >> booster separation at t={state.time:.2f}s  "
                  f"(mass {dynamics.current_mass_kg:.0f} kg, "
                  f"V={state.get_speed():.1f} m/s, alt={state.true_alt:.0f} m)")
            separated_reported = True
        prev_stage = sequencer.stage

        if i % 100 == 0 or i == steps - 1:   # every 1.0 s
            err = pos_err_m(state.true_position(), ins.get_state()[0])
            mass = dynamics.current_mass_kg + sequencer.attached_booster_mass()
            print(f"{state.time:5.1f} {sequencer.stage.name:>7} "
                  f"{state.true_alt:8.1f} {state.get_speed():7.1f} "
                  f"{math.degrees(state.pitch):6.1f} {mass:8.1f} {err:9.3f}")

    print(f"\nFinal: stage={sequencer.stage.name}  "
          f"alt={state.true_alt:.0f} m  V={state.get_speed():.1f} m/s  "
          f"dist={state.distance_traveled/1000:.2f} km")
    print(f"Booster separated: {sequencer.booster.separated}  | "
          f"cruise fuel left: {dynamics.engine.fuel_remaining_kg:.1f} kg")
    final_err = pos_err_m(state.true_position(), ins.get_state()[0])
    print(f"INS position error vs truth at t={state.time:.0f}s: {final_err:.2f} m "
          f"(clean INS -> integration-only residual through the boost transient)")


if __name__ == "__main__":
    main()
