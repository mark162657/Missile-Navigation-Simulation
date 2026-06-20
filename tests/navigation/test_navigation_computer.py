"""Tests for missile.navigation.navigation_computer.NavigationComputer."""
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
    """NavigationComputer with __init__ bypassed and collaborators injected."""
    nav = object.__new__(NavigationComputer)
    nav.ins_period = 0.002
    nav.gps_period = 0.2
    nav.tercom_period = 1.0
    nav.tercom_roughness_threshold_m = 5.0
    nav.state = make_state()
    nav.ins = INS(init_pos=[LAT, LON, ALT], init_vel=[0.0, 0.0, 0.0])
    nav.KF = KalmanFilter(
        dt=nav.ins_period,
        init_position=[LAT, LON, ALT],
        init_velocity=[0.0, 0.0, 0.0],
        process_noise_std=0.05,
    )
    nav.gps = MagicMock(is_jammed=False)
    nav.tercom = MagicMock()
    nav.dem_loader = MagicMock()
    nav.baro_alt = MagicMock()
    for k, v in attrs.items():
        setattr(nav, k, v)
    return nav


# ==========================================================================
# Construction
# ==========================================================================
def test_init_constructs_with_stub_dependencies(monkeypatch):
    import missile.navigation.navigation_computer as nc

    monkeypatch.setattr(nc, "DEMLoader", lambda *a, **k: MagicMock())
    monkeypatch.setattr(nc, "BaroAltimeter", lambda *a, **k: MagicMock())
    nav = NavigationComputer(start_gps=(LAT, LON, ALT), dem_name="fake.tif")
    assert isinstance(nav.state, MissileState)
    assert nav.state.gps_valid is True
    assert isinstance(nav.ins, INS)
    assert isinstance(nav.KF, KalmanFilter)
    assert nav.gps is not None
    assert nav.tercom is not None


# ==========================================================================
# KF -> INS -> state synchronisation
# ==========================================================================
def test_sync_kf_to_ins_and_state():
    nav = bare_nav()
    nav.KF.x = np.array([100.0, 200.0, ALT, 1.0, 2.0, 3.0])
    expected_pos, expected_vel = nav.KF.get_state()

    nav._sync_kf_to_ins_and_state()

    np.testing.assert_allclose(nav.ins.pos, expected_pos)
    np.testing.assert_allclose(nav.ins.vel, expected_vel)
    np.testing.assert_allclose(nav.state.est_position(), expected_pos)
    np.testing.assert_allclose(nav.state.get_velocity(), expected_vel)


def test_apply_gps_fix_pulls_estimate_toward_measurement():
    nav = bare_nav()
    measurement = [LAT + 0.01, LON - 0.01, ALT + 20.0]
    nav._apply_gps_fix(measurement)
    est = nav.state.est_position()
    assert abs(est[0] - measurement[0]) < abs(LAT - measurement[0])
    assert abs(est[1] - measurement[1]) < abs(LON - measurement[1])
    assert abs(est[2] - measurement[2]) < abs(ALT - measurement[2])


def test_apply_tercom_fix_updates_state():
    nav = bare_nav()
    nav._apply_tercom_fix(LAT + 0.02, LON + 0.02, ALT + 5.0)
    est = nav.state.est_position()
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
    nav._tercom_update()  # must not raise TypeError


def test_tercom_update_sets_active_flag_on_match():
    nav = bare_nav(dem_loader=_rough_patch_loader())
    nav.tercom.process_update.return_value = (LAT + 0.01, LON + 0.01, np.eye(3))
    nav.baro_alt.get_baro_msl.return_value = ALT
    nav.state.tercom_active = False

    nav._tercom_update()

    assert nav.state.tercom_active is True


def test_tercom_update_clears_active_when_sensed_patch_unavailable():
    loader = MagicMock()

    def fake(lat, lon, patch_size=7, normalized=True, **kw):
        if patch_size == 25:
            return np.random.default_rng(2).normal(0.0, 30.0, size=(25, 25))
        return None

    loader.get_elevation_patch.side_effect = fake
    nav = bare_nav(dem_loader=loader)
    nav.state.tercom_active = True

    nav._tercom_update()

    assert nav.state.tercom_active is False
    nav.tercom.process_update.assert_not_called()


def test_tercom_update_skips_when_terrain_flat():
    loader = MagicMock()
    loader.get_elevation_patch.side_effect = (
        lambda lat, lon, patch_size=7, normalized=True, **kw:
        np.full((patch_size, patch_size), 100.0)
    )
    nav = bare_nav(dem_loader=loader)
    nav.state.tercom_active = True
    nav._tercom_update()
    assert nav.state.tercom_active is False
    nav.tercom.process_update.assert_not_called()


# ==========================================================================
# run_navigation_loop scheduling
# ==========================================================================
class _CountdownLimit:
    """Bounds the navigation loop without relying on wall-clock time."""

    def __init__(self, limit):
        self.limit = limit
        self.calls = 0

    def __gt__(self, other):
        self.calls += 1
        return self.calls <= self.limit


def test_run_loop_respects_mission_terminated():
    nav = bare_nav()
    nav.state = MagicMock()
    nav.run_navigation_loop([0.0, 0.0, 0.0], mission_terminated=True)
    nav.state.update_physics.assert_not_called()


def test_run_loop_steps_ins_many_times():
    nav = bare_nav()
    nav.state = MagicMock()
    nav.state.est_position.return_value = np.array([LAT, LON, ALT])
    nav.state.true_position.return_value = np.array([LAT, LON, ALT])
    nav.ins = MagicMock()
    nav.KF = MagicMock()
    nav.gps.get_gps_location.return_value = None
    nav.dem_loader.get_elevation_patch.return_value = np.full((25, 25), 100.0)

    nav.run_navigation_loop([1.0, 0.0, 0.0], run_seconds=_CountdownLimit(2000))

    assert nav.state.update_physics.call_count > 100
    assert nav.ins.predict.call_count > 100
    assert nav.KF.predict.call_count > 100


def test_run_loop_reaches_gps_schedule():
    nav = bare_nav()
    nav.state = MagicMock()
    nav.state.est_position.return_value = np.array([LAT, LON, ALT])
    nav.state.true_position.return_value = np.array([LAT, LON, ALT])
    nav.ins = MagicMock()
    nav.KF = MagicMock()
    nav.KF.get_state.return_value = (np.array([LAT, LON, ALT]), np.array([0.0, 0.0, 0.0]))
    nav.gps.get_gps_location.return_value = np.array([LAT, LON, ALT])
    nav.dem_loader.get_elevation_patch.return_value = np.full((25, 25), 100.0)

    nav.run_navigation_loop([0.0, 0.0, 0.0], run_seconds=_CountdownLimit(2000))

    assert nav.gps.get_gps_location.call_count >= 1
