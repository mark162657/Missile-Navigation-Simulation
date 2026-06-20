"""
Shared pytest fixtures and import setup for the navigation test-suite.

Two responsibilities:

1. Make ``src/`` importable (so ``import missile...`` works even if the test
   runner did not export ``PYTHONPATH=src``).

2. Provide a hermetic, dependency-free stub for ``terrain.dem_loader`` so that
   importing the navigation modules never drags in ``rasterio`` / ``matplotlib``
   or touches real (large/absent) DEM ``.tif`` files. DEM / pathfinding is
   explicitly out of scope; everywhere the navigation code needs a DEM we use
   a fake elevation patch (a plain numpy array).

The stub ``DEMLoader`` deliberately mirrors the *real* public signature of
``get_elevation_patch(lat, lon, patch_size=7, normalized=True)`` so that bugs
such as the ``nromalized=`` keyword typo in ``navigation_computer`` surface as
real ``TypeError``s during tests instead of being silently swallowed by a
permissive MagicMock.
"""
import sys
import types
from pathlib import Path

import numpy as np
import pytest

# --- 1. Ensure src/ is importable -----------------------------------------
_SRC = Path(__file__).resolve().parents[2] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))


# --- 2. Install a hermetic stub for terrain.dem_loader --------------------
class StubDEMLoader:
    """A minimal DEMLoader replacement used for hermetic navigation tests.

    Signatures mirror the real ``terrain.dem_loader.DEMLoader``. By default it
    returns flat zero patches; individual tests override the methods or the
    ``patch_*`` attributes as needed.
    """

    def __init__(self, dem_path=None):
        self.dem_path = dem_path
        # default flat patch -> "terrain not suitable" for TERCOM
        self._default_value = 0.0

    def get_elevation_patch(self, lat, lon, patch_size=7, normalized=True):
        return np.full((patch_size, patch_size), self._default_value, dtype=float)

    def lat_lon_to_pixel(self, lat, lon):
        return (0, 0)

    def pixel_to_lat_lon(self, row, col):
        return (0.0, 0.0)


def _install_dem_loader_stub():
    if "terrain.dem_loader" in sys.modules:
        return
    module = types.ModuleType("terrain.dem_loader")
    module.DEMLoader = StubDEMLoader
    sys.modules["terrain.dem_loader"] = module


_install_dem_loader_stub()


@pytest.fixture
def stub_dem_loader_cls():
    """Expose the stub DEMLoader class to tests that want their own instance."""
    return StubDEMLoader
