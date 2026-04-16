# Missile Guidance Framework

Simulates a Tomahawk-class cruise missile guidance system over real-world satellite terrain data (DEM), featuring GPS navigation, TERCOM terrain matching, Kalman filter sensor fusion, and proportional navigation guidance. Modular by design for scalable experimentation and educational use.

---

## Overview

The project focuses on realistic simulation of missile guidance over digital elevation maps. Initial implementation provides foundational features: terrain loading, coordinate conversion, GPS/TERCOM guidance components, and classic sensor fusion. This codebase is structured for easy extension to advanced AI, radar, and complex engagement scenarios in later phases.

---

## Features (Initial Phase)

- Load and merge real DEM (SRTM) tiles over 500-1000 km² areas
- Convert between GPS (lat/lon) and local Cartesian (XY meters)
- Query elevation and compute terrain profiles along paths
- Simulate GPS navigation with realistic sensor noise and dropouts
- Implement TERCOM (Terrain Contour Matching) for position correction
- Foundational sensor fusion via Kalman filter
- A* pathfinding over terrain with elevation constraints
- Proportional navigation guidance toward target

## Work In Progress.....

## Planning Interface

A lightweight desktop planning interface is available for drafting missions before the full simulator is finished.

Run it from the project root:

```bash
python3 planner.py
```

Missile configurations now have a dedicated manager app:

```bash
python3 missile_configurator.py
```

What it supports right now:

- Select a DEM file from `data/dem/`
- Select a named missile configuration from shared storage in `data/missiles/configurations.json`
- Open a dedicated missile configuration app to create, rename, edit, and delete missile presets
- Enter start and target GPS coordinates
- Add, edit, reorder, and remove waypoints
- Use an interactive embedded map with pan/zoom tools and click-to-place mission points
- Validate whether both coordinates are inside the loaded terrain
- Run the C++ pathfinder from the planner for start -> waypoints -> target routing
- Preview the DEM in the planner with automatic crop-and-downsample around the mission area
- Launch an optional external GPU preview window when `fastplotlib` is available
- Estimate direct-flight distance, time, and turning radius
- Save and load planning drafts as JSON in `plans/`
