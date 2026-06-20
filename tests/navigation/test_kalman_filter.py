"""Unit tests for missile.navigation.kalman_filter.KalmanFilter.

The filter keeps an internal ENU state vector
[east_m, north_m, alt_m, vx, vy, vz] relative to the launch origin.
Geographic [lat, lon, alt] is exposed only through get_state() and update().
"""
import numpy as np
import pytest

from missile.navigation.kalman_filter import KalmanFilter

LAT, LON, ALT = 55.0, 99.0, 1000.0


def make_kf(dt=0.1, vel=None, process_noise_std=0.05):
    return KalmanFilter(
        dt=dt,
        init_position=[LAT, LON, ALT],
        init_velocity=vel if vel is not None else [0.0, 0.0, 0.0],
        process_noise_std=process_noise_std,
    )


# --------------------------------------------------------------------------
# Construction / matrix structure
# --------------------------------------------------------------------------
def test_initial_state_vector_is_enu_at_origin():
    kf = make_kf(vel=[1.0, 2.0, 3.0])
    assert kf.x.shape == (6,)
    # Launch point is the ENU origin; altitude and velocity are stored directly.
    np.testing.assert_allclose(kf.x, [0.0, 0.0, ALT, 1.0, 2.0, 3.0])


def test_matrix_dimensions():
    kf = make_kf()
    assert kf.A.shape == (6, 6)
    assert kf.B.shape == (6, 3)
    assert kf.H.shape == (3, 6)
    assert kf.Q.shape == (6, 6)
    assert kf.P.shape == (6, 6)
    assert kf.R_GPS.shape == (3, 3)
    assert kf.R_TERCOM.shape == (3, 3)


def test_observation_matrix_selects_position():
    kf = make_kf()
    expected = np.hstack([np.eye(3), np.zeros((3, 3))])
    np.testing.assert_array_equal(kf.H, expected)


def test_transition_matrix_values():
    dt = 0.2
    A = KalmanFilter._build_transition_matrix(dt)
    assert A.shape == (6, 6)
    np.testing.assert_allclose(np.diag(A), np.ones(6))
    assert A[0, 3] == pytest.approx(dt)
    assert A[1, 4] == pytest.approx(dt)
    assert A[2, 5] == pytest.approx(dt)


def test_control_matrix_values():
    dt = 0.2
    B = KalmanFilter._build_control_matrix(dt)
    assert B.shape == (6, 3)
    assert B[0, 0] == pytest.approx(0.5 * dt ** 2)
    assert B[1, 1] == pytest.approx(0.5 * dt ** 2)
    assert B[2, 2] == pytest.approx(0.5 * dt ** 2)
    assert B[3, 0] == pytest.approx(dt)
    assert B[4, 1] == pytest.approx(dt)
    assert B[5, 2] == pytest.approx(dt)


def test_gps_and_tercom_use_distinct_R_matrices():
    kf = make_kf()
    np.testing.assert_allclose(np.diag(kf.R_GPS), [1.0, 1.0, 9.0])
    np.testing.assert_allclose(np.diag(kf.R_TERCOM), [169.0, 64.0, 1.0])
    assert kf.R_TERCOM[0, 0] > kf.R_GPS[0, 0]
    assert kf.R_TERCOM[2, 2] < kf.R_GPS[2, 2]


def test_process_noise_is_symmetric_psd():
    kf = make_kf()
    np.testing.assert_allclose(kf.Q, kf.Q.T)
    eigvals = np.linalg.eigvalsh(kf.Q)
    assert np.all(eigvals >= -1e-12)


# --------------------------------------------------------------------------
# predict()
# --------------------------------------------------------------------------
def test_predict_applies_control_input_to_velocity():
    dt = 0.1
    kf = make_kf(dt=dt)
    kf.predict([1.0, 2.0, 3.0])
    np.testing.assert_allclose(kf.x[3:], np.array([1.0, 2.0, 3.0]) * dt)


def test_predict_advances_position_from_velocity_in_enu():
    dt = 0.5
    kf = make_kf(dt=dt, vel=[10.0, 20.0, 5.0])
    east0, north0, alt0 = kf.x[0], kf.x[1], kf.x[2]
    kf.predict([0.0, 0.0, 0.0])
    assert kf.x[0] == pytest.approx(east0 + dt * 10.0)
    assert kf.x[1] == pytest.approx(north0 + dt * 20.0)
    assert kf.x[2] == pytest.approx(alt0 + dt * 5.0)


def test_predict_increases_covariance():
    kf = make_kf()
    trace_before = np.trace(kf.P)
    kf.predict([0.0, 0.0, 0.0])
    assert np.trace(kf.P) >= trace_before


# --------------------------------------------------------------------------
# update()
# --------------------------------------------------------------------------
def test_update_reduces_covariance():
    kf = make_kf()
    trace_before = np.trace(kf.P)
    kf.update([LAT, LON, ALT], sensor_type="GPS")
    assert np.trace(kf.P) < trace_before


def test_update_keeps_covariance_symmetric():
    kf = make_kf()
    kf.predict([1.0, 1.0, 1.0])
    kf.update([LAT + 0.001, LON + 0.001, ALT + 1.0], sensor_type="GPS")
    np.testing.assert_allclose(kf.P, kf.P.T, atol=1e-9)


def test_update_pulls_estimate_toward_measurement():
    kf = make_kf()
    measurement = [LAT + 0.01, LON - 0.01, ALT + 50.0]
    before_pos, _ = kf.get_state()
    kf.update(measurement, sensor_type="GPS")
    after_pos, _ = kf.get_state()
    for i in range(3):
        assert abs(after_pos[i] - measurement[i]) < abs(before_pos[i] - measurement[i])


def test_repeated_updates_converge_to_measurement():
    kf = make_kf()
    measurement = [LAT + 0.02, LON + 0.02, ALT + 100.0]
    for _ in range(200):
        kf.update(measurement, sensor_type="GPS")
    pos, _ = kf.get_state()
    np.testing.assert_allclose(pos, measurement, rtol=0, atol=0.1)


def test_tercom_update_trusts_vertical_more_than_gps():
    kf_gps = make_kf()
    kf_ter = make_kf()
    meas = [LAT, LON, ALT + 100.0]
    kf_gps.update(meas, sensor_type="GPS")
    kf_ter.update(meas, sensor_type="TERCOM")
    err_gps = abs(kf_gps.x[2] - (ALT + 100.0))
    err_ter = abs(kf_ter.x[2] - (ALT + 100.0))
    assert err_ter < err_gps


def test_default_sensor_type_is_gps_path():
    kf1 = make_kf()
    kf2 = make_kf()
    kf1.update([LAT, LON, ALT + 10.0])
    kf2.update([LAT, LON, ALT + 10.0], sensor_type="GPS")
    np.testing.assert_allclose(kf1.x, kf2.x)
    np.testing.assert_allclose(kf1.P, kf2.P)


def test_get_state_splits_position_and_velocity():
    kf = make_kf(vel=[7.0, 8.0, 9.0])
    pos, vel = kf.get_state()
    np.testing.assert_allclose(pos, [LAT, LON, ALT])
    np.testing.assert_allclose(vel, [7.0, 8.0, 9.0])
