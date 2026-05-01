#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
FRONTEND_DIR="$ROOT_DIR/frontend/web_planner"

# Missile Guidance Web Planner - Startup Script

echo "--- Starting Missile Guidance Web Planner ---"

# 1. Check if C++ backend is compiled
if [ ! -f "$ROOT_DIR/src/pathfinder/missile_backend.cpython-313-darwin.so" ] && [ ! -f "$ROOT_DIR/src/pathfinder/missile_backend.so" ]; then
    echo "[!] Warning: C++ backend (.so) not found in src/pathfinder/"
    echo "    Please ensure you have compiled the C++ code."
fi

# 2. Check for dependencies
echo "[*] Checking dependencies..."
python3 -m pip install -q fastapi uvicorn python-multipart rasterio numpy pydantic

# 3. Build React frontend if dist is missing
if [ ! -d "$FRONTEND_DIR/dist" ]; then
    echo "[*] Building React frontend..."
    (
        cd "$FRONTEND_DIR"
        npm run build
    )
fi

# 4. Launch server
echo "[*] Launching FastAPI server on http://localhost:8000"
echo "[*] Press Ctrl+C to stop the server."
echo ""

python3 "$ROOT_DIR/src/web_planner/main.py"
