"""missile.navigation -- state estimation (where am I, how fast, which way).

    navigation_computer.py : fuses the sensors below into a MissileState.
    ins.py                 : inertial navigation (integrates IMU, ENU frame).
    gps.py                 : GPS fix wrapper around the sensor model.
    tercom.py              : terrain-contour matching against the DEM.
    kalman_filter.py       : INS/aiding fusion filter.
    timer.py               : simulation clock helper.
"""
