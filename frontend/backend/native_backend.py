"""Cross-platform availability checks for the native pathfinding extensions.

Supports both engines exposed in the frontend:
  - ``rust``: PyO3 ``pathfinder`` module (CFA* — Coarse-to-Fine A*, built from
    ``src/missile/pathfinding/pathfinder_backend``).
  - ``cpp``: pybind11 ``missile.planning.missile_backend`` module (legacy A*,
    built from ``src/missile/planning/cpp``).
"""
from __future__ import annotations

import inspect
from typing import Literal

from . import bootstrap  # noqa: F401 - puts src/ on sys.path

EngineName = Literal["rust", "cpp"]
DEFAULT_ENGINE: EngineName = "rust"
VALID_ENGINES: tuple[EngineName, ...] = ("rust", "cpp")


def normalize_engine(engine: str | None) -> EngineName:
    """Validate and normalize an engine identifier."""
    key = (engine or DEFAULT_ENGINE).strip().lower()
    if key not in VALID_ENGINES:
        raise ValueError(f"Unknown pathfinding engine {engine!r}. Expected 'rust' or 'cpp'.")
    return key  # type: ignore[return-value]


def _rust_cfa_ready() -> bool:
    """True once Rust Pathfinding.find_path (or rs_pathfinder) accepts k / downsample_k."""
    try:
        from missile.pathfinding.pathfinding_backend import Pathfinding
        params = inspect.signature(Pathfinding.find_path).parameters
        if "downsample_k" in params or "k" in params:
            return True
    except Exception:  # noqa: BLE001
        pass
    try:
        import pathfinder
        params = inspect.signature(pathfinder.rs_pathfinder).parameters
        return "downsample_k" in params or "k" in params
    except Exception:  # noqa: BLE001
        return False


def _probe_rust() -> tuple[bool, str | None]:
    try:
        import pathfinder  # noqa: F401
        return True, None
    except (ImportError, OSError) as exc:
        return False, (
            "Rust pathfinding engine (CFA*) is unavailable for this Python interpreter. "
            "Build and install src/missile/pathfinding/pathfinder_backend (e.g. "
            "'maturin develop --release' or 'pip install --force-reinstall .') using "
            f"the same Python environment that runs frontend/run.py. ({exc})"
        )


def _probe_cpp() -> tuple[bool, str | None]:
    try:
        # Python selects .pyd on Windows and .so on macOS/Linux.
        from missile.planning import missile_backend  # noqa: F401
        return True, None
    except (ImportError, OSError) as exc:
        return False, (
            "C++ pathfinding engine is unavailable for this Python interpreter. "
            "Build src/missile/planning/cpp with CMake using the same Python "
            f"environment that runs frontend/run.py. See frontend/README.md. ({exc})"
        )


def engine_status() -> dict:
    """Return live availability metadata for both native pathfinding engines."""
    rust_ok, rust_err = _probe_rust()
    cpp_ok, cpp_err = _probe_cpp()
    cfa_ready = _rust_cfa_ready() if rust_ok else False
    return {
        "default": DEFAULT_ENGINE,
        "engines": {
            "rust": {
                "id": "rust",
                "name": "Rust",
                "algorithm": "CFA*" if cfa_ready else "A*",
                "label": "Rust · CFA*" if cfa_ready else "Rust · A*",
                "available": rust_ok,
                "cfa_ready": cfa_ready,
                "error": rust_err,
                "supports_downsample_k": True,
                "default_downsample_k": 4,
            },
            "cpp": {
                "id": "cpp",
                "name": "C++",
                "algorithm": "A*",
                "label": "C++ · A*",
                "available": cpp_ok,
                "error": cpp_err,
                "supports_downsample_k": False,
            },
        },
    }


def require_pathfinder_backend(engine: str = DEFAULT_ENGINE) -> EngineName:
    """Fail before loading a DEM when the selected native extension is missing."""
    key = normalize_engine(engine)
    ok, err = _probe_rust() if key == "rust" else _probe_cpp()
    if not ok:
        raise RuntimeError(err or f"{key} pathfinding engine is unavailable.")
    return key
