# Advanced Missile Guidance Framework

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
