"""Unit tests for missile.navigation.gps.GPS and the GPS receiver sensor."""
import numpy as np
import pytest

from missile.navigation.gps import GPS

TRUE = [55.0, 99.0, 1000.0]


def test_gps_constructs_with_defaults():
    gps = GPS()
    assert gps.has_signal is True
    assert gps.is_jammed is False
    assert gps.receiver is not None


def test_get_gps_location_returns_three_component_measurement():
    np.random.seed(0)
    gps = GPS()
    mea = gps.get_gps_location(TRUE)
    arr = np.asarray(mea, dtype=float)
    assert arr.shape == (3,)
    # measurement should be close-ish to truth (sensor noise is small in degrees terms)
    assert np.all(np.isfinite(arr))


def test_get_gps_location_adds_noise():
    np.random.seed(1)
    gps = GPS()
    mea = np.asarray(gps.get_gps_location(TRUE), dtype=float)
    # extremely unlikely to be exactly equal to truth
    assert not np.allclose(mea, TRUE)


def test_detect_jammed_default_false():
    gps = GPS()
    assert gps.detect_jammed(np.array(TRUE)) is False


def test_jammed_path_sets_flag(monkeypatch):
    gps = GPS()
    monkeypatch.setattr(gps, "detect_jammed", lambda m: True)
    result = gps.get_gps_location(TRUE)
    assert gps.is_jammed is True
    # BUG (documented): jammed path returns a (None, None, None) tuple
    assert result == (None, None, None)


@pytest.mark.xfail(
    reason="BUG: GPS.get_gps_location return type is inconsistent. The nominal "
    "path returns a 3-vector ndarray, but the jammed path returns a "
    "(None, None, None) tuple instead of a single `None`. Callers in "
    "navigation_computer test `if mea is not None`, which is always True "
    "for the tuple, so a jammed fix is wrongly treated as valid.",
    strict=True,
)
def test_jammed_path_should_return_single_none(monkeypatch):
    gps = GPS()
    monkeypatch.setattr(gps, "detect_jammed", lambda m: True)
    result = gps.get_gps_location(TRUE)
    assert result is None
