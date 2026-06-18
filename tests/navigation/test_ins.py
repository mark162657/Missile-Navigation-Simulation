"""Unit tests for missile.navigation.ins.INS."""
import math

import numpy as np
import pytest

from missile.navigation.ins import INS
from terrain import coordinates

LAT, LON, ALT = 55.0, 99.0, 1000.0


def make_ins(vel=None, **kwargs):
    return INS(
        init_pos=[LAT, LON, ALT],
        init_vel=vel if vel is not None else [0.0, 0.0, 0.0],
        **kwargs,
    )


# --------------------------------------------------------------------------
# Construction / state accessors
# --------------------------------------------------------------------------
def test_init_copies_inputs_and_defaults_attitude():
    ins = make_ins(vel=[1.0, 2.0, 3.0])
    np.testing.assert_allclose(ins.pos, [LAT, LON, ALT])
    np.testing.assert_allclose(ins.vel, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(ins.att, [0.0, 0.0, 0.0])
    assert ins.time == 0.0
    assert ins.distance_traveled == 0.0


def test_init_does_not_alias_input_arrays():
    pos = np.array([LAT, LON, ALT])
    ins = INS(init_pos=pos, init_vel=[0.0, 0.0, 0.0])
    ins.pos[0] += 1.0
    assert pos[0] == LAT  # original must be untouched


def test_get_state_vector_roundtrip():
    ins = make_ins(vel=[4.0, 5.0, 6.0])
    sv = ins.get_state_vector()
    assert sv.shape == (6,)
    np.testing.assert_allclose(sv, [LAT, LON, ALT, 4.0, 5.0, 6.0])

    ins.set_state_vector([1.0, 2.0, 3.0, 7.0, 8.0, 9.0])
    np.testing.assert_allclose(ins.pos, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(ins.vel, [7.0, 8.0, 9.0])


def test_get_speed():
    ins = make_ins(vel=[3.0, 4.0, 0.0])
    assert ins.get_speed() == pytest.approx(5.0)


def test_correct_state_replaces_pos_vel():
    ins = make_ins(vel=[1.0, 1.0, 1.0])
    ins.correct_state([10.0, 20.0, 30.0], [2.0, 3.0, 4.0])
    np.testing.assert_allclose(ins.pos, [10.0, 20.0, 30.0])
    np.testing.assert_allclose(ins.vel, [2.0, 3.0, 4.0])


# --------------------------------------------------------------------------
# predict() integration math (noise-free, deterministic)
# --------------------------------------------------------------------------
def test_predict_noise_free_velocity_integration():
    ins = make_ins(vel=[0.0, 0.0, 0.0])
    dt = 0.5
    acc = np.array([1.0, 2.0, 3.0])
    ins.predict(acc, dt)
    np.testing.assert_allclose(ins.vel, acc * dt)


def test_predict_noise_free_position_integration():
    v0 = np.array([10.0, 20.0, 5.0])  # east, north, up
    ins = make_ins(vel=v0.tolist())
    dt = 0.5
    acc = np.array([1.0, 2.0, 3.0])
    m_lat = coordinates.meter_per_deg_lat(LAT)
    m_lon = coordinates.meter_per_deg_lon_at(LAT)
    ins.predict(acc, dt)
    # lat uses north component (index 1), lon uses east component (index 0)
    exp_lat = LAT + (v0[1] * dt + 0.5 * acc[1] * dt ** 2) / m_lat
    exp_lon = LON + (v0[0] * dt + 0.5 * acc[0] * dt ** 2) / m_lon
    exp_alt = ALT + v0[2] * dt + 0.5 * acc[2] * dt ** 2
    assert ins.pos[0] == pytest.approx(exp_lat)
    assert ins.pos[1] == pytest.approx(exp_lon)
    assert ins.pos[2] == pytest.approx(exp_alt)


def test_predict_updates_time_and_distance():
    ins = make_ins(vel=[3.0, 4.0, 0.0])
    ins.predict([0.0, 0.0, 0.0], 2.0)
    assert ins.time == pytest.approx(2.0)
    # speed = 5 m/s, dt = 2 -> 10 m
    assert ins.distance_traveled == pytest.approx(10.0)


def test_predict_integrates_attitude_from_angular_velocity():
    ins = make_ins()
    ins.predict([0.0, 0.0, 0.0], 1.0, angular_velocity=[0.1, 0.2, 0.3])
    np.testing.assert_allclose(ins.att, [0.1, 0.2, 0.3])


def test_predict_is_deterministic_without_noise():
    a = make_ins(vel=[5.0, 5.0, 1.0])
    b = make_ins(vel=[5.0, 5.0, 1.0])
    for _ in range(50):
        a.predict([0.3, -0.2, 0.1], 0.1)
        b.predict([0.3, -0.2, 0.1], 0.1)
    np.testing.assert_allclose(a.get_state_vector(), b.get_state_vector())


def test_attitude_is_wrapped_to_two_pi():
    ins = make_ins()
    ins.predict([0.0, 0.0, 0.0], 1.0, angular_velocity=[0.0, 0.0, 3 * math.pi])
    assert 0.0 <= ins.att[2] < 2 * math.pi
    assert ins.att[2] == pytest.approx(math.pi)


# --------------------------------------------------------------------------
# IMU error model (seeded RNG)
# --------------------------------------------------------------------------
def test_constant_bias_offsets_acceleration():
    rng = np.random.default_rng(0)
    ins = INS(
        init_pos=[LAT, LON, ALT],
        init_vel=[0.0, 0.0, 0.0],
        accel_bias=[0.5, 0.0, 0.0],
        rng=rng,
    )
    dt = 1.0
    ins.predict([0.0, 0.0, 0.0], dt)
    # true accel 0, but bias 0.5 -> east velocity grows by 0.5 m/s
    assert ins.vel[0] == pytest.approx(0.5)


def test_white_noise_is_reproducible_with_seed():
    a = INS(init_pos=[LAT, LON, ALT], init_vel=[0.0, 0.0, 0.0],
            accel_noise_std=0.1, rng=np.random.default_rng(42))
    b = INS(init_pos=[LAT, LON, ALT], init_vel=[0.0, 0.0, 0.0],
            accel_noise_std=0.1, rng=np.random.default_rng(42))
    a.predict([1.0, 0.0, 0.0], 0.1)
    b.predict([1.0, 0.0, 0.0], 0.1)
    np.testing.assert_allclose(a.get_state_vector(), b.get_state_vector())


def test_noise_makes_estimate_differ_from_clean():
    clean = make_ins()
    noisy = INS(init_pos=[LAT, LON, ALT], init_vel=[0.0, 0.0, 0.0],
                accel_noise_std=0.5, rng=np.random.default_rng(1))
    clean.predict([1.0, 0.0, 0.0], 0.1)
    noisy.predict([1.0, 0.0, 0.0], 0.1)
    assert not np.allclose(clean.vel, noisy.vel)


def test_tactical_grade_factory_sets_error_terms():
    ins = INS.tactical_grade([LAT, LON, ALT], [0.0, 0.0, 0.0],
                             rng=np.random.default_rng(7))
    assert ins.accel_noise_std > 0.0
    assert ins.gyro_noise_std > 0.0
    assert np.any(ins.accel_bias != 0.0)


# --------------------------------------------------------------------------
# Static motion-model matrices (shared with the Kalman filter)
# --------------------------------------------------------------------------
def test_transition_matrix_values():
    dt = 0.2
    A = INS.get_transition_matrix(dt, reference_lat=LAT)
    m_lat = 111_320.0
    m_lon = 111_320.0 * math.cos(math.radians(LAT))
    assert A.shape == (6, 6)
    np.testing.assert_allclose(np.diag(A), np.ones(6))
    assert A[0, 4] == pytest.approx(dt / m_lat)
    assert A[1, 3] == pytest.approx(dt / m_lon)
    assert A[2, 5] == pytest.approx(dt)


def test_control_matrix_values():
    dt = 0.2
    B = INS.get_control_matrix(dt, reference_lat=LAT)
    m_lat = 111_320.0
    m_lon = 111_320.0 * math.cos(math.radians(LAT))
    assert B.shape == (6, 3)
    assert B[0, 1] == pytest.approx(0.5 * dt ** 2 / m_lat)
    assert B[1, 0] == pytest.approx(0.5 * dt ** 2 / m_lon)
    assert B[2, 2] == pytest.approx(0.5 * dt ** 2)
    assert B[3, 0] == pytest.approx(dt)
    assert B[4, 1] == pytest.approx(dt)
    assert B[5, 2] == pytest.approx(dt)
