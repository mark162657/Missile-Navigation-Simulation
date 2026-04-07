#!/bin/bash

# Missile Guidance Web Planner - Startup Script

echo "--- Starting Missile Guidance Web Planner ---"

# 1. Check if C++ backend is compiled
if [ ! -f "src/pathfinder/missile_backend.cpython-313-darwin.so" ] && [ ! -f "src/pathfinder/missile_backend.so" ]; then
    echo "[!] Warning: C++ backend (.so) not found in src/pathfinder/"
    echo "    Please ensure you have compiled the C++ code."
fi

# 2. Check for dependencies
echo "[*] Checking dependencies..."
pip install -q fastapi uvicorn python-multipart rasterio numpy pydantic

# 3. Launch server
echo "[*] Launching FastAPI server on http://localhost:8000"
echo "[*] Press Ctrl+C to stop the server."
echo ""

python3 src/web_planner/main.py
