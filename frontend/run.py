#!/usr/bin/env python3
"""Launch the web control terminal.

    python3 frontend/run.py            # http://127.0.0.1:8000
    python3 frontend/run.py --port 9000 --reload

Run from the project root (or anywhere — paths are resolved absolutely).
"""
from __future__ import annotations

import argparse

import uvicorn


def main() -> None:
    ap = argparse.ArgumentParser(description="Missile-guidance web control terminal")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--reload", action="store_true", help="auto-reload on code changes (dev)")
    args = ap.parse_args()

    uvicorn.run(
        "backend.app:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        app_dir=str(__import__("pathlib").Path(__file__).resolve().parent),
    )


if __name__ == "__main__":
    main()
