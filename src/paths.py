"""Project directory paths. Set PYTHONPATH=src when running Python (see README)."""
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SRC_ROOT.parent
