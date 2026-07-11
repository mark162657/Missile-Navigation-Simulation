"""Web control terminal backend for the missile-guidance simulator.

Bridges the browser UI to the existing simulation stack:
  * catalog  — DEMs, missile profiles, saved mission results
  * dem_service — downsampled elevation grids for the map + 3D viewer
  * replay   — streams recorded flight-log CSVs as telemetry frames
  * live_runner — drives the real Simulation and streams the same frames
"""
