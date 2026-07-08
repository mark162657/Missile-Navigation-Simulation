"""Unit tests for missile.state.MissileState."""
import math

import numpy as np
import pytest

from missile.state import MissileState
from missile.navigation.ins import INS
from terrain import coordinates

LAT, LON, ALT = 55.0, 99.0, 1000.0


def make_state(**overrides):
    base = dict(
        true_lat=LAT, true_lon=LON, true_alt=ALT,
        est_lat=LAT, est_lon=LON, est_alt=ALT,
        vel_east=0.0, vel_north=0.0, vel_up=0.0,
        roll=0.0, pitch=0.0, yaw=0.0,
        time=0.0, distance_traveled=0.0, distance_to_target=0.0,
        gps_valid=True, tercom_active=False, ins_calibrated=True,
    )
    base.update(overrides)
    return MissileState(**base)


def test_get_speed():
    s = make_state(vel_east=3.0, vel_north=4.0, vel_up=0.0)
    assert s.get_ground_speed() == pytest.approx(5.0)


def test_position_helpers():
    s = make_state()
    np.testing.assert_allclose(s.est_position(), [LAT, LON, ALT])
    np.testing.assert_allclose(s.true_position(), [LAT, LON, ALT])
    np.testing.assert_allclose(s.get_velocity(), [0.0, 0.0, 0.0])
    np.testing.assert_allclose(s.get_attitude(), [0.0, 0.0, 0.0])


def test_apply_kf_position_only_touches_estimate():
    s = make_state()
    s.apply_kf_position([10.0, 20.0, 30.0])
    np.testing.assert_allclose(s.est_position(), [10.0, 20.0, 30.0])
    # truth untouched
    np.testing.assert_allclose(s.true_position(), [LAT, LON, ALT])


def test_apply_ins_estimate_copies_ins_state():
    s = make_state()
    ins = INS(init_pos=[1.0, 2.0, 3.0], init_vel=[4.0, 5.0, 6.0],
              init_att=[0.1, 0.2, 0.3])
    ins.time = 12.0
    ins.distance_traveled = 99.0
    s.apply_ins_estimate(ins)
    np.testing.assert_allclose(s.est_position(), [1.0, 2.0, 3.0])
    np.testing.assert_allclose(s.get_velocity(), [4.0, 5.0, 6.0])
    np.testing.assert_allclose(s.get_attitude(), [0.1, 0.2, 0.3])
    assert s.time == 12.0
    assert s.distance_traveled == 99.0
    # apply_ins_estimate must NOT change ground truth
    np.testing.assert_allclose(s.true_position(), [LAT, LON, ALT])


# --------------------------------------------------------------------------
# update_physics kinematics vs analytic expectation
# --------------------------------------------------------------------------
def test_update_physics_velocity_integration():
    s = make_state()
    dt = 0.5
    acc = [1.0, 2.0, 3.0]
    s.update_physics(dt, acc, yaw_rate=0.0)
    assert s.vel_east == pytest.approx(1.0 * dt)
    assert s.vel_north == pytest.approx(2.0 * dt)
    assert s.vel_up == pytest.approx(3.0 * dt)


def test_update_physics_position_integration():
    s = make_state(vel_east=10.0, vel_north=20.0, vel_up=5.0)
    dt = 0.5
    acc = [1.0, 2.0, 3.0]
    m_lat = coordinates.meter_per_deg_lat(LAT)
    m_lon = coordinates.meter_per_deg_lon_at(LAT)
    s.update_physics(dt, acc, yaw_rate=0.0)
    exp_lat = LAT + (20.0 * dt + 0.5 * 2.0 * dt ** 2) / m_lat
    exp_lon = LON + (10.0 * dt + 0.5 * 1.0 * dt ** 2) / m_lon
    exp_alt = ALT + 5.0 * dt + 0.5 * 3.0 * dt ** 2
    assert s.true_lat == pytest.approx(exp_lat)
    assert s.true_lon == pytest.approx(exp_lon)
    assert s.true_alt == pytest.approx(exp_alt)


def test_update_physics_does_not_change_estimate():
    s = make_state()
    s.update_physics(1.0, [5.0, 5.0, 5.0], yaw_rate=0.1)
    np.testing.assert_allclose(s.est_position(), [LAT, LON, ALT])


def test_update_physics_yaw_wraps():
    s = make_state(yaw=0.0)
    s.update_physics(1.0, [0.0, 0.0, 0.0], yaw_rate=3 * math.pi)
    assert 0.0 <= s.yaw < 2 * math.pi
    assert s.yaw == pytest.approx(math.pi)


def test_update_physics_advances_time_and_distance():
    s = make_state(vel_east=3.0, vel_north=4.0, vel_up=0.0)
    s.update_physics(1.0, [0.0, 0.0, 0.0], yaw_rate=0.0)
    assert s.time == pytest.approx(1.0)
    # distance uses the post-update speed (5 m/s) * dt
    assert s.distance_traveled == pytest.approx(5.0)


def test_update_physics_matches_ins_predict_for_same_inputs():
    """MissileState truth integration and INS dead-reckoning should agree
    for identical initial conditions and inputs (both are the same model)."""
    v0 = [10.0, 20.0, 5.0]
    s = make_state(vel_east=v0[0], vel_north=v0[1], vel_up=v0[2])
    ins = INS(init_pos=[LAT, LON, ALT], init_vel=v0)
    dt = 0.1
    acc = [0.5, -0.3, 0.2]
    s.update_physics(dt, acc, yaw_rate=0.0)
    ins.predict(acc, dt)
    assert s.true_lat == pytest.approx(ins.pos[0])
    assert s.true_lon == pytest.approx(ins.pos[1])
    assert s.true_alt == pytest.approx(ins.pos[2])
