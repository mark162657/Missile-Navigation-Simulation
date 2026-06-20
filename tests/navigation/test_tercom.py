"""Unit tests for the non-pathfinding parts of missile.navigation.tercom.TERCOM.

The DEM matching *search* (pixel geometry, real DEM tiles) is explicitly out of
scope; here we test the math that the rest of navigation depends on:
normalized cross-correlation and the noise covariance matrix. A single light
integration test exercises process_update() against a planted pattern using a
fake (in-memory) DEM loader -- no rasterio, no .tif files.
"""
import numpy as np
import pytest

from missile.navigation.tercom import TERCOM


def make_tercom():
    # Constructs against the hermetic StubDEMLoader installed in conftest.
    return TERCOM([55.0, 99.0], "fake.tif")


# --------------------------------------------------------------------------
# cross_correlation()
# --------------------------------------------------------------------------
def test_cross_correlation_identical_patch_is_one():
    t = make_tercom()
    rng = np.random.default_rng(0)
    patch = rng.normal(size=(7, 7))
    windows = patch[np.newaxis, ...]  # shape (1, 7, 7)
    ncc = t.cross_correlation(windows, patch)
    assert ncc.shape == (1,)
    assert ncc[0] == pytest.approx(1.0, abs=1e-4)


def test_cross_correlation_is_offset_and_scale_invariant():
    t = make_tercom()
    rng = np.random.default_rng(1)
    patch = rng.normal(size=(7, 7))
    # affine transform of the same pattern -> still perfectly correlated
    windows = (3.0 * patch + 100.0)[np.newaxis, ...]
    ncc = t.cross_correlation(windows, patch)
    assert ncc[0] == pytest.approx(1.0, abs=1e-4)


def test_cross_correlation_negated_patch_is_minus_one():
    t = make_tercom()
    rng = np.random.default_rng(2)
    patch = rng.normal(size=(7, 7))
    windows = (-patch)[np.newaxis, ...]
    ncc = t.cross_correlation(windows, patch)
    assert ncc[0] == pytest.approx(-1.0, abs=1e-4)


def test_cross_correlation_handles_batched_windows():
    t = make_tercom()
    rng = np.random.default_rng(3)
    patch = rng.normal(size=(7, 7))
    windows = rng.normal(size=(4, 5, 7, 7))
    windows[2, 3] = patch  # plant exact match
    ncc = t.cross_correlation(windows, patch)
    assert ncc.shape == (4, 5)
    best = np.unravel_index(np.argmax(ncc), ncc.shape)
    assert best == (2, 3)
    assert ncc[best] == pytest.approx(1.0, abs=1e-4)


# --------------------------------------------------------------------------
# get_noise_covariance()
# --------------------------------------------------------------------------
def test_noise_covariance_matrix():
    t = make_tercom()
    cov = t.get_noise_covariance()
    assert cov.shape == (3, 3)
    np.testing.assert_allclose(
        np.diag(cov),
        [t.lateral_accuracy ** 2, t.lateral_accuracy ** 2, t.vertical_accuracy ** 2],
    )
    # off-diagonals are zero (independent axes)
    off = cov - np.diag(np.diag(cov))
    np.testing.assert_array_equal(off, np.zeros((3, 3)))


# --------------------------------------------------------------------------
# process_update() — light integration against a planted pattern
# --------------------------------------------------------------------------
class _FakeMatchLoader:
    """In-memory DEM loader that returns a search patch with a planted pattern."""

    def __init__(self, search_patch):
        self._search = search_patch

    def lat_lon_to_pixel(self, lat, lon):
        return (100, 100)

    def get_elevation_patch(self, lat, lon, patch_size=7, normalized=True):
        return self._search

    def pixel_to_lat_lon(self, row, col):
        # deterministic invertible mapping for assertions
        return (row / 1000.0, col / 1000.0)


def test_process_update_finds_planted_pattern():
    t = make_tercom()
    rng = np.random.default_rng(5)
    search = rng.normal(size=(15, 15))
    # plant a known 7x7 pattern at window position (row=4, col=2)
    pattern = rng.normal(size=(7, 7))
    search[4:11, 2:9] = pattern
    t.dem_loader = _FakeMatchLoader(search)

    lat, lon, cov = t.process_update(pattern, est_lat=55.0, est_lon=99.0, search_size=15)
    assert lat is not None and lon is not None
    assert cov is not None
    np.testing.assert_allclose(np.diag(cov), np.diag(t.get_noise_covariance()))


def test_process_update_no_match_returns_none_triplet():
    t = make_tercom()
    # flat search patch -> zero variance windows -> correlation never exceeds threshold
    flat = np.zeros((15, 15))
    t.dem_loader = _FakeMatchLoader(flat)
    sensed = np.zeros((7, 7))
    result = t.process_update(sensed, est_lat=55.0, est_lon=99.0, search_size=15)
    assert result == (None, None, None)
