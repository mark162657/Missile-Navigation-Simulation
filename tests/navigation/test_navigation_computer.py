"""Tests for missile.navigation.navigation_computer.NavigationComputer.

NavigationComputer is now a pure per-tick ESTIMATION service: it does not own a
MissileState and does not advance truth. The estimation helpers take the shared
state as an argument, and the per-tick entry point is step(imu, state, sim_time,
dt). (The old state-owning run_navigation_loop / update_physics scaffolding was
moved into the main simulation loop.)
"""
from unittest.mock import MagicMock

import numpy as np
import pytest

from missile.navigation.navigation_computer import NavigationComputer
from missile.navigation.kalman_filter import KalmanFilter
from missile.navigation.ins import INS
from missile.state import MissileState

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


def bare_nav(**attrs):
    """NavigationComputer with __init__ bypassed and collaborators injected.

    No nav.state: the estimation methods operate on a MissileState passed in.
    """
    nav = object.__new__(NavigationComputer)
    nav.ins_period = 0.002
    nav.gps_period = 0.2
    nav.tercom_period = 1.0
    nav.tercom_roughness_threshold_m = 5.0
    nav.next_gps = nav.gps_period
    nav.next_tercom = nav.tercom_period
    nav.ins = INS(init_pos=[LAT, LON, ALT], init_vel=[0.0, 0.0, 0.0])
    nav.KF = KalmanFilter(
        dt=nav.ins_period,
        init_position=[LAT, LON, ALT],
        init_velocity=[0.0, 0.0, 0.0],
        process_noise_std=0.05,
    )
    nav.imu = MagicMock()
    nav.gps = MagicMock(is_jammed=False)
    nav.tercom = MagicMock()
    nav.dem_loader = MagicMock()
    nav.baro_alt = MagicMock()
    for k, v in attrs.items():
        setattr(nav, k, v)
    return nav


class _FakeIMU:
    """Minimal stand-in for dynamics.IMUMeasurement (only the fields step() reads)."""
    def __init__(self, accel_enu=None, angular_velocity=None):
        self.accel_enu = np.zeros(3) if accel_enu is None else np.asarray(accel_enu, dtype=float)
        self.angular_velocity = np.zeros(3) if angular_velocity is None else np.asarray(angular_velocity, dtype=float)


# ==========================================================================
# Construction
# ==========================================================================
def test_init_constructs_estimators_without_owning_state(monkeypatch):
    import missile.navigation.navigation_computer as nc

    monkeypatch.setattr(nc, "DEMLoader", lambda *a, **k: MagicMock())
    monkeypatch.setattr(nc, "BaroAltimeter", lambda *a, **k: MagicMock())
    monkeypatch.setattr(nc, "TERCOM", lambda *a, **k: MagicMock())
    nav = NavigationComputer(true_start_gps=(LAT, LON, ALT), dem_name="fake.tif")

    assert isinstance(nav.ins, INS)
    assert isinstance(nav.KF, KalmanFilter)
    assert nav.gps is not None
    assert nav.tercom is not None
    # The nav computer no longer owns a MissileState (main.Simulation does).
    assert not hasattr(nav, "state")


# ==========================================================================
# KF -> INS -> state synchronisation (position only)
# ==========================================================================
def test_sync_estimate_writes_position_only():
    nav = bare_nav()
    state = make_state()
    nav.KF.x = np.array([100.0, 200.0, ALT, 1.0, 2.0, 3.0])
    expected_pos, expected_vel = nav.KF.get_state()

    nav._sync_estimate_to_state(state)

    # INS is fully corrected internally (position + velocity)...
    np.testing.assert_allclose(nav.ins.pos, expected_pos)
    np.testing.assert_allclose(nav.ins.vel, expected_vel)
    # ...but only POSITION is mirrored onto the shared state (the plant owns
    # velocity/attitude), so est_* moves and vel_* is left untouched.
    np.testing.assert_allclose(state.est_position(), expected_pos)
    np.testing.assert_allclose(state.get_velocity(), [0.0, 0.0, 0.0])


def test_apply_gps_fix_pulls_estimate_toward_measurement():
    nav = bare_nav()
    state = make_state()
    measurement = [LAT + 0.01, LON - 0.01, ALT + 20.0]
    nav._apply_gps_fix(measurement, state)
    est = state.est_position()
    assert abs(est[0] - measurement[0]) < abs(LAT - measurement[0])
    assert abs(est[1] - measurement[1]) < abs(LON - measurement[1])
    assert abs(est[2] - measurement[2]) < abs(ALT - measurement[2])


def test_apply_tercom_fix_updates_state():
    nav = bare_nav()
    state = make_state()
    nav._apply_tercom_fix(LAT + 0.02, LON + 0.02, ALT + 5.0, state)
    est = state.est_position()
    assert abs(est[0] - (LAT + 0.02)) < 0.02
    assert abs(est[2] - (ALT + 5.0)) < 5.0


# ==========================================================================
# _is_terrain_suitable
# ==========================================================================
def test_terrain_suitable_for_rough_patch():
    nav = bare_nav()
    rough = np.random.default_rng(0).normal(0.0, 50.0, size=(25, 25))
    assert nav._is_terrain_suitable(rough, LAT, LON) is True


def test_terrain_not_suitable_for_flat_patch():
    nav = bare_nav()
    flat = np.full((25, 25), 100.0)
    assert nav._is_terrain_suitable(flat, LAT, LON) is False


def test_terrain_not_suitable_when_no_patch_and_no_loader():
    nav = bare_nav(dem_loader=None)
    assert nav._is_terrain_suitable(None, LAT, LON) is False


def test_terrain_suitable_filters_non_finite():
    nav = bare_nav()
    patch = np.full((25, 25), np.inf)
    assert nav._is_terrain_suitable(patch, LAT, LON) is False


# ==========================================================================
# _tercom_update
# ==========================================================================
def _rough_patch_loader():
    loader = MagicMock()

    def fake(lat, lon, patch_size=7, normalized=True, **kw):
        return np.random.default_rng(1).normal(0.0, 30.0, size=(patch_size, patch_size))

    loader.get_elevation_patch.side_effect = fake
    return loader


def test_tercom_update_accepts_normalized_keyword():
    nav = bare_nav(dem_loader=_rough_patch_loader())
    nav.tercom.process_update.return_value = (None, None, None)
    nav._tercom_update(make_state())  # must not raise TypeError


def test_tercom_update_sets_active_flag_on_match():
    nav = bare_nav(dem_loader=_rough_patch_loader())
    nav.tercom.process_update.return_value = (LAT + 0.01, LON + 0.01, np.eye(3))
    nav.baro_alt.get_baro_msl.return_value = ALT
    state = make_state(tercom_active=False)

    nav._tercom_update(state)

    assert state.tercom_active is True


def test_tercom_update_clears_active_when_sensed_patch_unavailable():
    loader = MagicMock()

    def fake(lat, lon, patch_size=7, normalized=True, **kw):
        if patch_size == 25:
            return np.random.default_rng(2).normal(0.0, 30.0, size=(25, 25))
        return None

    loader.get_elevation_patch.side_effect = fake
    nav = bare_nav(dem_loader=loader)
    state = make_state(tercom_active=True)

    nav._tercom_update(state)

    assert state.tercom_active is False
    nav.tercom.process_update.assert_not_called()


def test_tercom_update_skips_when_terrain_flat():
    loader = MagicMock()
    loader.get_elevation_patch.side_effect = (
        lambda lat, lon, patch_size=7, normalized=True, **kw:
        np.full((patch_size, patch_size), 100.0)
    )
    nav = bare_nav(dem_loader=loader)
    state = make_state(tercom_active=True)
    nav._tercom_update(state)
    assert state.tercom_active is False
    nav.tercom.process_update.assert_not_called()


# ==========================================================================
# step() -- per-tick estimation (replaces run_navigation_loop)
# ==========================================================================
def test_step_predicts_ins_kf_and_writes_position_only():
    nav = bare_nav()
    nav.imu.imu_error.return_value = (np.zeros(3), np.zeros(3))
    nav.gps.get_gps_location.return_value = None
    nav.dem_loader.get_elevation_patch.return_value = np.full((25, 25), 100.0)  # flat -> TERCOM skips
    state = make_state()

    nav.step(_FakeIMU(), state, sim_time=0.0, dt=nav.ins_period)

    # IMU truth was corrupted once, INS dead-reckoned, and a position estimate written.
    nav.imu.imu_error.assert_called_once()
    assert np.all(np.isfinite(state.est_position()))
    # Truth velocity/attitude is the plant's; step() must not touch it.
    np.testing.assert_allclose(state.get_velocity(), [0.0, 0.0, 0.0])


def test_step_does_not_advance_truth():
    """step() must never call the old MissileState.update_physics truth integrator."""
    nav = bare_nav()
    nav.imu.imu_error.return_value = (np.zeros(3), np.zeros(3))
    nav.gps.get_gps_location.return_value = None
    nav.dem_loader.get_elevation_patch.return_value = np.full((25, 25), 100.0)
    state = MagicMock()
    state.true_position.return_value = np.array([LAT, LON, ALT])
    state.est_position.return_value = np.array([LAT, LON, ALT])

    nav.step(_FakeIMU(), state, sim_time=0.0, dt=nav.ins_period)

    state.update_physics.assert_not_called()


def test_step_takes_a_gps_fix_at_the_gps_rate():
    nav = bare_nav()
    nav.imu.imu_error.return_value = (np.zeros(3), np.zeros(3))
    nav.gps.get_gps_location.return_value = np.array([LAT, LON, ALT])
    nav.dem_loader.get_elevation_patch.return_value = np.full((25, 25), 100.0)
    state = make_state()

    # sim_time past the first GPS checkpoint -> a fix is pulled.
    nav.step(_FakeIMU(), state, sim_time=nav.gps_period, dt=nav.ins_period)

    assert nav.gps.get_gps_location.call_count >= 1
    assert state.gps_valid is True
