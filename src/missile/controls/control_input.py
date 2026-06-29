"""
control_input.py -- the autopilot's output command (plant boundary).

This dataclass is the single command the avionics emit and the physics plant
consumes. It is owned by the MISSILE (avionics) side because the autopilot is
its PRODUCER; the plant (simulation.physics.dynamics) only CONSUMES it. Keeping
it here -- in its own dependency-light module -- lets both the autopilot and
dynamics import the TYPE without dragging in any behaviour, and keeps the
dependency arrow pointing the one allowed way (simulation -> missile).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ControlInput:
    """
    Autopilot output consumed by the physics (the only thing that drives it).

    NO CONTROL SURFACES: the missile is treated as a whole point mass. The
    autopilot commands a maneuver acceleration PERPENDICULAR to the velocity
    vector; physics clamps it to the g-envelope and rotates the velocity by it.

    Fields:
        throttle    : engine throttle, 0..1 (along-velocity energy input)
        accel_turn  : horizontal maneuver acceleration, m/s^2 (+ = to the right
                      of velocity). Rotates HEADING.
        accel_climb : vertical maneuver acceleration, m/s^2 (+ = pull up).
                      Rotates the FLIGHT-PATH ANGLE. NOTE: to hold level flight
                      the autopilot must command ~g here to counter gravity --
                      that is the controller's job, not the plant's.
    """
    throttle: float = 0.0
    accel_turn: float = 0.0
    accel_climb: float = 0.0
