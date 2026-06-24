"""
aerodynamics.py -- airframe drag model + maneuver (g) envelope.

WHAT IT DOES
    This project treats the missile as a WHOLE point mass with NO control
    surfaces -- it is an algorithm / navigation / guidance / pathfinding study,
    not a simulation of missile hardware. The autopilot commands a maneuver
    acceleration perpendicular to the velocity vector; the airframe is assumed
    to generate whatever lift is needed to achieve it. Aerodynamics therefore
    provides only the two things the point-mass plant still needs:
        - DRAG : parasite + induced (from the commanded lift) + transonic wave.
        - the g-ENVELOPE : the largest lateral acceleration the airframe can
          aerodynamically produce at the current dynamic pressure -- you cannot
          pull hard at low speed / high altitude (low q).

WHAT IT OUTPUTS
    - drag(q, mach, lift_force)   -> drag force, N
    - max_lateral_accel(q, mass)  -> aerodynamically-available |a_perp|, m/s^2

WHO CONSUMES IT
    - dynamics.py : sums (thrust - drag) along the velocity, applies the
      (g-clamped) commanded maneuver acceleration perpendicular to it, and adds
      gravity. There is no force coming from a control-surface deflection.

NO CONTROL SURFACES
    There are deliberately NO elevator/rudder/aileron inputs, no angle-of-attack
    trim, and no pitching-moment terms. Steering is commanded directly as an
    acceleration (by the autopilot) and is bounded here only by the g-envelope.
    Coefficient values are representative of a Tomahawk-class subsonic cruise
    airframe; exact numbers are not critical (rule 7), the structure is.
"""

from __future__ import annotations

import math


# ----------------------------------------------------------------------
# Airframe constants (representative, subsonic cruise missile)
# ----------------------------------------------------------------------
S_REF = 0.83          # aerodynamic reference area, m^2

# Drag: CD = CD0 + K_INDUCED * CL^2 + transonic wave-drag rise
CD0 = 0.028           # parasite (zero-lift) drag coefficient
K_INDUCED = 0.12      # induced-drag factor (1 / (pi * e * AR))

# Transonic wave-drag bump (small for a subsonic cruise missile, but present).
CD_WAVE_PEAK = 0.045  # extra CD added at the transonic peak
M_WAVE_CENTER = 1.0   # Mach where the bump peaks
M_WAVE_WIDTH = 0.18   # Gaussian half-width of the bump in Mach

# Maximum usable lift coefficient -> sets the aerodynamic g-envelope (the most
# perpendicular acceleration the airframe can pull before stalling/saturating).
CL_MAX = 1.2

# Boost configuration: wings folded + booster attached -> parasite drag only.
# A single representative parasite CD is used while the booster is attached.
BOOST_DRAG_CD = 0.30


def drag_coefficient(cl: float, mach: float) -> float:
    """Drag coefficient at lift coefficient `cl` and Mach: parasite + induced + wave."""
    cd = CD0 + K_INDUCED * cl * cl
    # Smooth transonic rise centred near M = 1; negligible deep in the subsonic
    # cruise regime, but keeps the model well-behaved if speed ever climbs.
    delta = (mach - M_WAVE_CENTER) / M_WAVE_WIDTH
    cd += CD_WAVE_PEAK * math.exp(-0.5 * delta * delta)
    return cd


class Aerodynamics:
    """
    Surface-free aerodynamic model: drag + maneuver (g) envelope for one airframe.

    Stateless apart from the reference area, so one instance can be shared by the
    whole simulation.
    """

    def __init__(self, reference_area: float = S_REF) -> None:
        self.reference_area = float(reference_area)

    def drag(self, dynamic_pressure: float, mach: float, lift_force: float) -> float:
        """
        Drag force (N) at the current flight condition and lift.

        Args:
            dynamic_pressure: q = 0.5 * rho * V^2, Pa
            mach: Mach number (V / speed_of_sound)
            lift_force: magnitude of the lift the airframe is generating, N
                (= mass * commanded perpendicular acceleration). Sets the
                induced-drag term, so maneuvering bleeds energy.
        """
        q_s = dynamic_pressure * self.reference_area
        cl = lift_force / q_s if q_s > 0.0 else 0.0
        return q_s * drag_coefficient(cl, mach)

    def max_lateral_accel(self, dynamic_pressure: float, mass: float) -> float:
        """
        Aerodynamically-available maneuver acceleration (m/s^2): the most |a_perp|
        the airframe can pull at this dynamic pressure, set by CL_MAX. Falls
        toward zero at low speed / high altitude (low q), so the plant can clamp
        an over-eager command to what the air can actually deliver.
        """
        if mass <= 0.0:
            return 0.0
        l_max = dynamic_pressure * self.reference_area * CL_MAX
        return l_max / mass
