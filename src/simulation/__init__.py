"""simulation -- the "truth" world the flight software runs against.

Subpackages:
    physics : 3-DoF dynamics, atmosphere, aerodynamics, engine, booster,
              flight sequencer, weather/wind.
    sensors : sensor models (IMU, GPS receiver, baro + radar altimeter) that
              corrupt the true state into measurements for navigation.

Data flows ONE way: control -> physics -> (new_state, sensor measurements).
"""
