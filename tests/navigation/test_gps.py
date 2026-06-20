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
    assert np.all(np.isfinite(arr))


def test_get_gps_location_adds_noise():
    np.random.seed(1)
    gps = GPS()
    mea = np.asarray(gps.get_gps_location(TRUE), dtype=float)
    assert not np.allclose(mea, TRUE)


def test_detect_jammed_default_false():
    gps = GPS()
    assert gps.detect_jammed(np.array(TRUE)) is False


def test_jammed_path_sets_flag_and_returns_none(monkeypatch):
    gps = GPS()
    monkeypatch.setattr(gps, "detect_jammed", lambda m: True)
    result = gps.get_gps_location(TRUE)
    assert gps.is_jammed is True
    assert result is None


def test_jammed_return_is_falsy_for_caller_guard(monkeypatch):
    """NavigationComputer uses `if mea is not None` before fusing a fix."""
    gps = GPS()
    monkeypatch.setattr(gps, "detect_jammed", lambda m: True)
    assert gps.get_gps_location(TRUE) is None
