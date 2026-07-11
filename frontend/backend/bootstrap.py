"""Put the simulator's `src/` package root on sys.path.

The avionics stack uses absolute imports rooted at `src/` (e.g.
`from missile.state import MissileState`), so the web backend has to add that
directory before importing anything from the simulation.
"""
from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = PROJECT_ROOT / "src"
DATA_ROOT = PROJECT_ROOT / "data"

if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
