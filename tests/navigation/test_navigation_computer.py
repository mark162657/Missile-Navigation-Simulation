"""Tests for missile.navigation.navigation_computer.NavigationComputer.

NavigationComputer.__init__ currently cannot run to completion (several
independent defects, see test_init_is_broken + the BUG REPORT). To still
exercise the orchestration logic (run_navigation_loop scheduling and the
private fusion helpers), we build a *bare* instance with object.__new__ and
wire the collaborators by hand -- real INS / KalmanFilter / MissileState plus
lightweight mocks for the DEM loader, GPS and baro altimeter.
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
    """A NavigationComputer with __init__ bypassed and collaborators injected."""
    nav = object.__new__(NavigationComputer)
    nav.ins_period = 0.002
    nav.gps_period = 0.2
    nav.tercom_period = 1.0
    nav.tercom_roughness_threshold_m = 5.0
    nav.state = make_state()
    nav.ins = INS(init_pos=[LAT, LON, ALT], init_vel=[0.0, 0.0, 0.0])
    nav.KF = KalmanFilter(dt=nav.ins_period, init_position=[LAT, LON, ALT],
                          init_velocity=[0.0, 0.0, 0.0], process_noise_std=0.05)
    nav.gps = MagicMock(is_jammed=False)
    nav.tercom = MagicMock()
    nav.dem_loader = MagicMock()
    nav.baro_alt = MagicMock()
    for k, v in attrs.items():
        setattr(nav, k, v)
    return nav


# ==========================================================================
# Construction is broken (documented bugs)
# ==========================================================================
def test_init_is_broken():
    """NavigationComputer(...) raises before finishing __init__.

    Multiple independent defects make construction impossible:
      * BaroAltimeter() raises TypeError (MissileState() needs args)
      * _build_initial_state is missing `self`
      * MissileState(gps_valud=...) is a keyword typo (should be gps_valid)
      * TERCOM(est_lat, est_lon) does not match TERCOM(location, dem_name)
    """
    with pytest.raises(TypeError):
        NavigationComputer(start_gps=(LAT, LON, ALT), dem_name="fake.tif")


@pytest.mark.xfail(
    reason="BUG: NavigationComputer.__init__ has several fatal defects "
    "(BaroAltimeter()/MissileState() construction, _build_initial_state "
    "missing self, gps_valud= typo, TERCOM signature mismatch). It should "
    "construct cleanly and expose .state/.ins/.KF/.gps/.tercom.",
    strict=True,
)
def test_init_should_succeed(monkeypatch):
    import missile.navigation.navigation_computer as nc

    monkeypatch.setattr(nc, "DEMLoader", lambda *a, **k: MagicMock())
    monkeypatch.setattr(nc, "BaroAltimeter", lambda *a, **k: MagicMock())
    nav = NavigationComputer(start_gps=(LAT, LON, ALT), dem_name="fake.tif")
    assert isinstance(nav.state, MissileState)
    assert nav.state.gps_valid is True


# ==========================================================================
# KF -> INS -> state synchronisation
# ==========================================================================
def test_sync_kf_to_ins_and_state():
    nav = bare_nav()
    nav.KF.x = np.array([10.0, 20.0, 30.0, 1.0, 2.0, 3.0])
    nav._sync_kf_to_ins_and_state()
    # INS receives the KF position/velocity
    np.testing.assert_allclose(nav.ins.pos, [10.0, 20.0, 30.0])
    np.testing.assert_allclose(nav.ins.vel, [1.0, 2.0, 3.0])
    # state mirrors INS
    np.testing.assert_allclose(nav.state.est_position(), [10.0, 20.0, 30.0])
    np.testing.assert_allclose(nav.state.get_velocity(), [1.0, 2.0, 3.0])


def test_apply_gps_fix_pulls_estimate_toward_measurement():
    nav = bare_nav()
    measurement = [LAT + 0.01, LON - 0.01, ALT + 20.0]
    nav._apply_gps_fix(measurement)
    est = nav.state.est_position()
    # estimate moved off the prior toward the measurement on each axis
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
class _StrictSignatureLoader:
    """DEM loader mirroring the *real* get_elevation_patch signature (no **kwargs)."""

    def get_elevation_patch(self, lat, lon, patch_size=7, normalized=True):
        return np.zeros((patch_size, patch_size))

    def lat_lon_to_pixel(self, lat, lon):
        return (0, 0)

    def pixel_to_lat_lon(self, row, col):
        return (0.0, 0.0)


def test_tercom_update_crashes_on_nromalized_typo():
    """BUG: _tercom_update calls dem_loader.get_elevation_patch(..., nromalized=False).
    Against the real DEMLoader signature (normalized=...) this raises TypeError,
    so a TERCOM fix can never run in production."""
    nav = bare_nav(dem_loader=_StrictSignatureLoader())
    with pytest.raises(TypeError):
        nav._tercom_update()


def _rough_patch_loader():
    """Permissive loader (accepts the kwarg typo) returning rough terrain."""
    loader = MagicMock()

    def fake(lat, lon, patch_size=7, normalized=True, **kw):
        return np.random.default_rng(1).normal(0.0, 30.0, size=(patch_size, patch_size))

    loader.get_elevation_patch.side_effect = fake
    return loader


def test_tercom_update_documents_active_flag_typo():
    """BUG: on a successful match _tercom_update sets `self.state.tecrom_active`
    (typo) instead of `self.state.tercom_active`. The intended flag is never set."""
    nav = bare_nav(dem_loader=_rough_patch_loader())
    nav.tercom.process_update.return_value = (LAT + 0.01, LON + 0.01, np.eye(3))
    nav.baro_alt.get_baro_msl.return_value = ALT
    nav.state.tercom_active = False

    nav._tercom_update()

    # the typo'd attribute is what actually gets written:
    assert getattr(nav.state, "tecrom_active") is True
    # ...while the real flag stays stale:
    assert nav.state.tercom_active is False


@pytest.mark.xfail(
    reason="BUG: tercom_active is misspelled `tecrom_active` on a successful "
    "TERCOM match, so the real state flag is never updated.",
    strict=True,
)
def test_tercom_update_should_set_active_flag():
    nav = bare_nav(dem_loader=_rough_patch_loader())
    nav.tercom.process_update.return_value = (LAT + 0.01, LON + 0.01, np.eye(3))
    nav.baro_alt.get_baro_msl.return_value = ALT
    nav._tercom_update()
    assert nav.state.tercom_active is True


def test_tercom_update_sensed_none_branch_is_noop_bug():
    """BUG: when the sensed patch is None, the code runs the statement
    `self.state.tercom_active is None` (comparison, no effect) instead of an
    assignment, so the flag is not cleared."""
    loader = MagicMock()

    def fake(lat, lon, patch_size=7, normalized=True, **kw):
        if patch_size == 25:
            return np.random.default_rng(2).normal(0.0, 30.0, size=(25, 25))
        return None  # sensed patch unavailable

    loader.get_elevation_patch.side_effect = fake
    nav = bare_nav(dem_loader=loader)
    nav.state.tercom_active = True  # pre-existing (stale) value
    nav._tercom_update()
    # No assignment happened -> stays True; process_update never called.
    assert nav.state.tercom_active is True
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
    """Stand-in for run_seconds: `sim_time < run_seconds` stays True for `limit`
    comparisons, then becomes False -- lets us bound the (otherwise infinite)
    loop deterministically without a wall-clock timeout."""

    def __init__(self, limit):
        self.limit = limit
        self.calls = 0

    def __gt__(self, other):  # evaluated as run_seconds > sim_time
        self.calls += 1
        return self.calls <= self.limit


def test_run_loop_respects_mission_terminated():
    nav = bare_nav()
    nav.state = MagicMock()
    nav.run_navigation_loop([0.0, 0.0, 0.0], mission_terminated=True)
    nav.state.update_physics.assert_not_called()


def test_run_loop_sim_time_never_advances_BUG():
    """BUG: in run_navigation_loop the `sim_time += self.ins_period` line is
    dedented OUTSIDE the while loop, so sim_time is frozen at 0.0. The loop
    body therefore runs the INS branch exactly once and then spins forever
    (an infinite loop in production). Here we bound it with a custom
    run_seconds object and assert the frozen-schedule symptom."""
    nav = bare_nav()
    nav.state = MagicMock()
    nav.ins = MagicMock()
    nav.KF = MagicMock()

    nav.run_navigation_loop([1.0, 0.0, 0.0], run_seconds=_CountdownLimit(500))

    # Over 500 iterations the INS branch fires only once (sim_time never moves).
    assert nav.state.update_physics.call_count == 1
    assert nav.ins.predict.call_count == 1
    assert nav.KF.predict.call_count == 1
    # GPS/TERCOM checkpoints (one period in) are never reached.
    nav.gps.get_gps_location.assert_not_called()


@pytest.mark.xfail(
    reason="BUG: sim_time is incremented outside the while loop, so the "
    "navigation loop never advances time and never terminates. A correct "
    "loop would step the INS branch many times over a multi-second run.",
    strict=True,
)
def test_run_loop_should_step_ins_many_times():
    nav = bare_nav()
    nav.state = MagicMock()
    nav.ins = MagicMock()
    nav.KF = MagicMock()
    # If sim_time advanced by ins_period each tick, ~1s / 0.002 ~ 500 INS steps.
    nav.run_navigation_loop([1.0, 0.0, 0.0], run_seconds=_CountdownLimit(2000))
    assert nav.state.update_physics.call_count > 100
