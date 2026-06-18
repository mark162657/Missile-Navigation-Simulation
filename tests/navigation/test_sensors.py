"""Unit tests for the simulation sensor models the nav computer depends on."""
import numpy as np
import pytest

from simulation.sensors.gps_receiver import GPSReceiver
from simulation.sensors.radar_altimeter import RadarAltimeter

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
    """DOCUMENTS a units concern: the receiver adds metre-scale Gaussian noise
    directly onto a [lat_deg, lon_deg, alt_m] vector. The horizontal noise is
    therefore injected in *degrees* even though h_std is specified in metres,
    i.e. ~2.3 'metres' becomes ~2.3 degrees (~250 km) of latitude error."""
    np.random.seed(0)
    rx = GPSReceiver(horizontal_accuracy=2.3, vertical_accuracy=3.1)
    samples = np.array([rx.get_raw_measurement(TRUE) for _ in range(4000)])
    lat_std = samples[:, 0].std()
    # The empirical spread of the latitude channel is ~h_std *in degrees*,
    # confirming the metre value is mis-applied to a degree quantity.
    assert lat_std == pytest.approx(2.3, rel=0.1)


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
# BaroAltimeter — broken construction (documented bug)
# --------------------------------------------------------------------------
def test_baro_altimeter_construction_is_broken():
    """BUG: BaroAltimeter.__init__ does `self.state = MissileState()`, but
    MissileState is a dataclass with ~18 required fields and no defaults, so
    construction always raises TypeError. This also makes NavigationComputer
    (which builds a BaroAltimeter) impossible to instantiate."""
    from simulation.sensors.baro_altimeter import BaroAltimeter
    with pytest.raises(TypeError):
        BaroAltimeter()


@pytest.mark.xfail(
    reason="BUG: BaroAltimeter cannot be constructed (MissileState() needs "
    "required args), and even if it could, it builds its OWN MissileState "
    "instead of referencing the missile's shared state, so get_baro_msl would "
    "never reflect the real altitude.",
    strict=True,
)
def test_baro_altimeter_reports_state_altitude():
    from simulation.sensors.baro_altimeter import BaroAltimeter
    baro = BaroAltimeter()
    baro.state.true_alt = 1234.0
    assert abs(baro.get_baro_msl() - 1234.0) < 5.0
