"""missile -- top-level package for the cruise-missile flight software.

Subpackages:
    controls    : autopilot, flight computer, guidance-law + PID inner loops.
    navigation  : INS/GPS/TERCOM + Kalman fusion (the navigation computer).
    guidance    : path following and terminal guidance.
    planning    : trajectory generation and the C++ pathfinding backend.
    datalink    : (reserved) mid-course datalink / command uplink.

Also holds the shared data models: state.py, profile.py, config_store.py.
"""
