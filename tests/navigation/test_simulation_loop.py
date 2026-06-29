"""
End-to-end *navigation-only* simulation loop.

This test exercises the whole navigation stack working together along a fixed,
reasonable, consecutive flight path -- NO guidance, NO autopilot. The only
"physics" used is ``MissileState.update_physics`` to propagate ground truth
(a constant-velocity cruise leg), which is exactly the kind of trajectory the
real sim would feed in.

Components under test (all the real classes, not mocks):
    INS            -- dead-reckons the noisy IMU
    KalmanFilter   -- fuses INS prediction with GPS / TERCOM fixes
    GPS            -- noisy GPS receiver model
    TERCOM         -- terrain-contour matching against a DEM
    BaroAltimeter  -- vertical channel for the TERCOM fix

DEM: the real ``terrain.dem_loader`` is unavailable/huge in tests (and the
shared conftest stub returns flat terrain), so we use a self-contained
``SyntheticDEM`` that mirrors ``DEMLoader``'s public API on a 1-arc-second
(~30 m/pixel) grid filled with rough, locally-unique terrain. That gives TERCOM
real signal to correlate against.

The loop here deliberately mirrors ``NavigationComputer.run_navigation_loop``'s
fusion data-flow (INS.predict + KF.predict every tick; KF.update + INS resync on
each GPS/TERCOM fix) so the test reflects how the system is actually wired today.

Caveat documented elsewhere (audit item C3): TERCOM's sensed patch is pulled
from the same DEM as the search, so the correlation is an oracle -- the match is
essentially perfect and lands on the true pixel. This test therefore validates
*wiring and fusion*, not TERCOM's robustness to radar-altimeter noise.
"""
import math

import numpy as np
import pytest

from missile.navigation.ins import INS
from missile.navigation.kalman_filter import KalmanFilter
from missile.navigation.gps import GPS
from missile.navigation.tercom import TERCOM
from missile.state import MissileState
from simulation.sensors.baro_altimeter import BaroAltimeter
from terrain import coordinates


# ===========================================================================
# Synthetic DEM (mirrors terrain.dem_loader.DEMLoader's public surface)
# ===========================================================================
ARCSEC_DEG = 1.0 / 3600.0  # 1 arc-second ~ 30 m/pixel (nominal)


class SyntheticDEM:
    """In-memory DEM on a north-up 1-arc-second grid.

    Pixel (0,0) is the north-west corner. Rows increase southward (lat down),
    columns increase eastward (lon up) -- the same convention as a GeoTIFF.
    Mirrors the methods TERCOM actually calls:
    ``get_elevation_patch``, ``lat_lon_to_pixel``, ``pixel_to_lat_lon``.
    """

    def __init__(self, lat_top: float, lon_left: float, n_rows: int, n_cols: int,
                 seed: int = 7):
        self.lat_top = lat_top
        self.lon_left = lon_left
        self.shape = (n_rows, n_cols)

        rng = np.random.default_rng(seed)
        rows = np.arange(n_rows)[:, None]
        cols = np.arange(n_cols)[None, :]
        # Gentle large-scale relief so values look like elevations ...
        relief = (
            300.0 * np.sin(rows / 240.0)
            + 250.0 * np.cos(cols / 310.0)
            + 0.05 * (rows + cols)
        )
        # ... plus rough, per-pixel detail that makes every 7x7 patch unique
        # (so normalized cross-correlation has a single sharp maximum).
        detail = rng.normal(0.0, 40.0, size=(n_rows, n_cols))
        self.data = (800.0 + relief + detail).astype(np.float32)

    # -- coordinate <-> pixel (floor/center, matching rasterio rowcol/xy) ----
    def lat_lon_to_pixel(self, lat: float, lon: float):
        row = int(math.floor((self.lat_top - lat) / ARCSEC_DEG))
        col = int(math.floor((lon - self.lon_left) / ARCSEC_DEG))
        return row, col

    def pixel_to_lat_lon(self, row, col):
        lat = self.lat_top - (row + 0.5) * ARCSEC_DEG
        lon = self.lon_left + (col + 0.5) * ARCSEC_DEG
        return lat, lon

    def get_elevation(self, lat: float, lon: float):
        row, col = self.lat_lon_to_pixel(lat, lon)
        if 0 <= row < self.shape[0] and 0 <= col < self.shape[1]:
            return float(self.data[row, col])
        return None

    def get_elevation_patch(self, lat: float, lon: float,
                            patch_size: int = 7, normalized: bool = True):
        row, col = self.lat_lon_to_pixel(lat, lon)
        half = patch_size // 2
        r0 = max(0, row - half)
        r1 = min(self.shape[0], row + half + 1)
        c0 = max(0, col - half)
        c1 = min(self.shape[1], col + half + 1)
        if r0 >= r1 or c0 >= c1:
            return None
        patch = self.data[r0:r1, c0:c1].astype(float)
        if normalized:
            patch = (patch - patch.mean()) / (patch.std() + 1e-6)
        return patch


# ===========================================================================
# Scenario constants
# ===========================================================================
# Start near the middle of the synthetic DEM so the cruise leg stays in bounds
# with comfortable margin for TERCOM's 125-pixel search window.
START_LAT = 55.0
START_LON = 95.0
START_ALT = 1200.0  # m MSL

CRUISE_SPEED = 200.0          # m/s, representative subsonic cruise
HEADING_DEG = 45.0            # north-east, so both lat & lon evolve
DURATION_S = 40.0

INS_HZ = 100
GPS_HZ = 5
TERCOM_HZ = 1

# DEM big enough that 40 s @ 200 m/s (~8 km, a few hundred pixels) plus the
# 125-pixel search box never runs off the edge.
DEM_ROWS = DEM_COLS = 3000
# Place START at pixel (1500, 1500).
LAT_TOP = START_LAT + 1500 * ARCSEC_DEG
LON_LEFT = START_LON - 1500 * ARCSEC_DEG


def _horizontal_error_m(lat1, lon1, lat2, lon2):
    """Local-tangent-plane horizontal distance in meters between two lat/lon."""
    m_lat = coordinates.meter_per_deg_lat(lat1)
    m_lon = coordinates.meter_per_deg_lon_at(lat1)
    dn = (lat2 - lat1) * m_lat
    de = (lon2 - lon1) * m_lon
    return math.hypot(dn, de)


# ===========================================================================
# The simulation loop (runs once, shared across assertions)
# ===========================================================================
@pytest.fixture(scope="module")
def sim():
    np.random.seed(1234)  # GPSReceiver / BaroAltimeter use global np.random
    ins_rng = np.random.default_rng(99)

    dem = SyntheticDEM(LAT_TOP, LON_LEFT, DEM_ROWS, DEM_COLS, seed=7)

    # Initial ENU cruise velocity (constant -> zero acceleration input).
    heading = math.radians(HEADING_DEG)
    vel_east = CRUISE_SPEED * math.sin(heading)
    vel_north = CRUISE_SPEED * math.cos(heading)
    vel0 = [vel_east, vel_north, 0.0]
    accel = np.array([0.0, 0.0, 0.0])  # steady cruise

    # --- shared truth + estimate state ---
    state = MissileState(
        true_lat=START_LAT, true_lon=START_LON, true_alt=START_ALT,
        est_lat=START_LAT, est_lon=START_LON, est_alt=START_ALT,
        vel_east=vel_east, vel_north=vel_north, vel_up=0.0,
        roll=0.0, pitch=0.0, yaw=heading,
        time=0.0, distance_traveled=0.0, distance_to_target=0.0,
        gps_valid=True, tercom_active=False, ins_calibrated=True,
    )

    dt = 1.0 / INS_HZ

    # Real navigation components. INS is tactical-grade so it actually drifts
    # between fixes -- giving the KF/GPS/TERCOM something real to correct.
    ins = INS.tactical_grade(
        init_pos=[START_LAT, START_LON, START_ALT],
        init_vel=vel0,
        rng=ins_rng,
    )
    kf = KalmanFilter(
        dt=dt,
        init_position=[START_LAT, START_LON, START_ALT],
        init_velocity=vel0,
        process_noise_std=0.05,
    )
    gps = GPS()
    tercom = TERCOM([START_LAT, START_LON], dem_name="synthetic.tif")
    tercom.dem_loader = dem  # swap stub for the synthetic terrain
    baro = BaroAltimeter()

    gps_period = 1.0 / GPS_HZ
    tercom_period = 1.0 / TERCOM_HZ
    next_gps = gps_period
    next_tercom = tercom_period

    # --- bookkeeping for assertions ---
    errors = []            # horizontal est-vs-truth error each tick
    gps_fixes = 0
    tercom_attempts = 0
    tercom_matches = 0
    tercom_errors = []     # horizontal error of each accepted TERCOM fix

    n_ticks = int(round(DURATION_S * INS_HZ))
    for i in range(1, n_ticks + 1):
        t = i * dt

        # 1) Advance ground truth (the only physics in this test).
        state.update_physics(dt, accel, yaw_rate=0.0)

        # 2) INS + KF prediction every tick (mirrors NavigationComputer).
        ins.predict(accel, dt, angular_velocity=[0.0, 0.0, 0.0])
        kf.predict(accel)
        state.apply_ins_estimate(ins)

        # 3) GPS fix.
        if t >= next_gps - 1e-9:
            mea = gps.get_gps_location(state.true_position())
            if mea is not None:
                kf.update(list(mea), sensor_type="GPS")
                est_pos, est_vel = kf.get_state()
                ins.correct_state(est_pos, est_vel)
                state.apply_ins_estimate(ins)
                gps_fixes += 1
            next_gps += gps_period

        # 4) TERCOM fix.
        if t >= next_tercom - 1e-9:
            tercom_attempts += 1
            sensed = dem.get_elevation_patch(
                state.true_lat, state.true_lon, patch_size=7, normalized=True
            )
            m_lat, m_lon, _cov = tercom.process_update(
                sensed, state.est_lat, state.est_lon
            )
            if m_lat is not None:
                tercom_matches += 1
                tercom_errors.append(
                    _horizontal_error_m(state.true_lat, state.true_lon, m_lat, m_lon)
                )
                alt = baro.get_baro_msl(state.true_alt)
                kf.update([m_lat, m_lon, alt], sensor_type="TERCOM")
                est_pos, est_vel = kf.get_state()
                ins.correct_state(est_pos, est_vel)
                state.apply_ins_estimate(ins)
            next_tercom += tercom_period

        errors.append(
            _horizontal_error_m(
                state.true_lat, state.true_lon, state.est_lat, state.est_lon
            )
        )

    return {
        "state": state,
        "errors": np.array(errors),
        "final_error": errors[-1],
        "gps_fixes": gps_fixes,
        "tercom_attempts": tercom_attempts,
        "tercom_matches": tercom_matches,
        "tercom_errors": np.array(tercom_errors) if tercom_errors else np.array([]),
        "n_ticks": n_ticks,
    }


# ===========================================================================
# Assertions
# ===========================================================================
def test_loop_runs_to_completion(sim):
    """The whole stack steps through the flight without raising."""
    assert sim["n_ticks"] == int(round(DURATION_S * INS_HZ))
    assert len(sim["errors"]) == sim["n_ticks"]


def test_truth_actually_moved(sim):
    """Sanity: the cruise leg is consecutive and covers a sensible distance."""
    st = sim["state"]
    travelled = _horizontal_error_m(START_LAT, START_LON, st.true_lat, st.true_lon)
    expected = CRUISE_SPEED * DURATION_S
    assert travelled == pytest.approx(expected, rel=0.02)


def test_gps_fixes_were_applied(sim):
    expected = int(DURATION_S * GPS_HZ)
    assert sim["gps_fixes"] >= expected - 2  # allow off-by-one at the edges


def test_tercom_matched(sim):
    """Terrain is rough enough that TERCOM locks on every attempt."""
    assert sim["tercom_attempts"] >= int(DURATION_S * TERCOM_HZ) - 1
    assert sim["tercom_matches"] >= 1
    # With the oracle correlation, essentially every attempt should match.
    assert sim["tercom_matches"] >= sim["tercom_attempts"] - 1


def test_tercom_fix_lands_on_true_pixel(sim):
    """An accepted TERCOM fix should resolve to within ~1.5 pixels of truth."""
    assert sim["tercom_errors"].size > 0
    # 1 arc-second ~ 30 m; allow 1.5 px for pixel-center quantization.
    assert sim["tercom_errors"].max() < 45.0


def test_estimate_tracks_truth(sim):
    """The fused estimate stays locked to truth across the whole flight."""
    errors = sim["errors"]
    # GPS at 5 Hz + tactical INS: post-fix error is GPS-noise dominated.
    assert sim["final_error"] < 25.0
    assert errors.mean() < 15.0
    assert errors.max() < 75.0
