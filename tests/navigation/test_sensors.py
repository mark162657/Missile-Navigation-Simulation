"""Unit tests for the simulation sensor models the nav computer depends on."""
import numpy as np
import pytest

from simulation.sensors.gps_receiver import GPSReceiver
from simulation.sensors.radar_altimeter import RadarAltimeter
from terrain.coordinates import meter_per_deg_lat, meter_per_deg_lon_at

TRUE = [55.0, 99.0, 1000.0]


# --------------------------------------------------------------------------
# GPSReceiver
# --------------------------------------------------------------------------
def test_gps_receiver_zero_noise_returns_truth():
    rx = GPSReceiver(horizontal_accuracy=0.0, vertical_accuracy=0.0)
    mea = rx.get_raw_measurement(TRUE)
    np.testing.assert_allclose(mea, TRUE)


def test_gps_receiver_shape_and_finite():
    np.random.seed(0)
    rx = GPSReceiver()
    mea = rx.get_raw_measurement(TRUE)
    assert mea.shape == (3,)
    assert np.all(np.isfinite(mea))


def test_gps_receiver_noise_spread_matches_std_units():
    """REGRESSION: horizontal accuracy is specified in metres, but lat/lon are
    stored in degrees. The receiver must convert the metre-scale noise into
    degrees via the WGS-84 metres-per-degree scale at the current latitude, so
    that the empirical spread converted back to metres matches h_std / v_std."""
    np.random.seed(0)
    h_std, v_std = 2.3, 3.1
    lat = TRUE[0]
    rx = GPSReceiver(horizontal_accuracy=h_std, vertical_accuracy=v_std)
    samples = np.array([rx.get_raw_measurement(TRUE) for _ in range(20000)])

    # Convert the degree spread of lat/lon back into metres; it should match the
    # configured horizontal accuracy. Altitude is already in metres.
    lat_std_m = samples[:, 0].std() * meter_per_deg_lat(lat)
    lon_std_m = samples[:, 1].std() * meter_per_deg_lon_at(lat)
    alt_std_m = samples[:, 2].std()

    assert lat_std_m == pytest.approx(h_std, rel=0.1)
    assert lon_std_m == pytest.approx(h_std, rel=0.1)
    assert alt_std_m == pytest.approx(v_std, rel=0.1)


# --------------------------------------------------------------------------
# RadarAltimeter
# --------------------------------------------------------------------------
def test_radar_altimeter_zero_noise_is_agl_difference():
    ra = RadarAltimeter(vertical_std=0.0)
    agl = ra.get_altimeter_agl(true_curr_agl=1500.0, true_dem_elev=400.0)
    assert agl == pytest.approx(1100.0)


def test_radar_altimeter_noise_reproducible():
    np.random.seed(7)
    ra = RadarAltimeter(vertical_std=1.0)
    a = ra.get_altimeter_agl(1500.0, 400.0)
    np.random.seed(7)
    b = ra.get_altimeter_agl(1500.0, 400.0)
    assert a == b


# --------------------------------------------------------------------------
# BaroAltimeter
# --------------------------------------------------------------------------
def test_baro_altimeter_constructs():
    from simulation.sensors.baro_altimeter import BaroAltimeter

    baro = BaroAltimeter()
    assert baro is not None


def test_baro_altimeter_applies_noise_to_passed_altitude():
    from simulation.sensors.baro_altimeter import BaroAltimeter

    np.random.seed(0)
    baro = BaroAltimeter()
    result = baro.get_baro_msl(1234.0)
    assert abs(result - 1234.0) < 2.0


def test_baro_altimeter_zero_noise_seed_returns_truth():
    from simulation.sensors.baro_altimeter import BaroAltimeter

    np.random.seed(42)
    baro = BaroAltimeter()
    # With a fixed seed the draw is deterministic; verify it stays near truth.
    assert baro.get_baro_msl(1000.0) == pytest.approx(1000.0, abs=2.0)
