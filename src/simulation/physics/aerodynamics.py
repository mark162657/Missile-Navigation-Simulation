"""
aerodynamics.py -- airframe aerodynamic force/moment model.

WHAT IT DOES
    Computes the aerodynamic lift, drag and side force (plus pitching moment,
    for the future 6-DoF upgrade) acting on the airframe as functions of:
        - Mach number          (compressibility / transonic drag rise)
        - angle of attack alpha (rad)
        - sideslip angle beta  (rad)
        - dynamic pressure q   (Pa)
    It also maps control-surface deflections to the trim incidence angles a
    point-mass (3-DoF) model flies at, via `trim_alpha` / `trim_beta`.

WHAT IT OUTPUTS
    An `AeroForces` bundle (lift, drag, side_force in N; pitching_moment in
    N*m). All forces are magnitudes expressed in the wind/stability sense;
    dynamics.py is responsible for turning them into ENU force vectors.

WHO CONSUMES IT
    - dynamics.py : sums these forces with thrust and gravity to integrate
                    the equations of motion.

3-DoF vs 6-DoF (rule 8)
    In this point-mass model there are no rotational states, so we assume the
    airframe is statically trimmed: a commanded elevator/rudder deflection
    instantly produces a steady angle of attack / sideslip (the `trim_*`
    helpers). The pitching-moment coefficients are still computed and exposed
    so that upgrading to 6-DoF only means integrating the moment in
    dynamics.py instead of assuming instantaneous trim -- no change here or in
    any other module.

Coefficient values below are representative of a Tomahawk-class subsonic
cruise airframe. Exact numbers are not critical (rule 7); the structure is.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ----------------------------------------------------------------------
# Airframe constants (representative, subsonic cruise missile)
# ----------------------------------------------------------------------
S_REF = 0.83          # aerodynamic reference area, m^2

# Lift: CL = CL0 + CL_ALPHA * alpha
CL0 = 0.0             # zero-AoA lift coefficient
CL_ALPHA = 3.8        # lift-curve slope, per rad

# Drag: CD = CD0 + K_INDUCED * CL^2 + wave-drag rise
CD0 = 0.028           # parasite (zero-lift) drag coefficient
K_INDUCED = 0.12      # induced-drag factor (1 / (pi * e * AR))

# Transonic wave-drag bump (small for a subsonic cruise missile, but present).
CD_WAVE_PEAK = 0.045  # extra CD added at the transonic peak
M_WAVE_CENTER = 1.0   # Mach where the bump peaks
M_WAVE_WIDTH = 0.18   # Gaussian half-width of the bump in Mach

# Side force: CY = CY_BETA * beta
CY_BETA = -1.6        # side-force slope, per rad

# Boost configuration: wings folded + booster attached -> drag only, no lift
# bookkeeping. A single representative parasite drag coefficient is used while
# the booster is attached (the cruise CD0 above does not apply in this config).
BOOST_DRAG_CD = 0.30

# Pitching moment (about CG): Cm = CM0 + CM_ALPHA*alpha + CM_DELTA_E*delta_e
# Sign convention: +elevator = nose-up. For that, the control moment must be
# nose-up positive (CM_DELTA_E > 0) while static stability keeps CM_ALPHA < 0,
# so the trim solution gives +elevator -> +alpha (climb). (Used by 6-DoF;
# informational in 3-DoF.)
CM0 = 0.0
CM_ALPHA = -0.45      # per rad (statically stable)
CM_DELTA_E = 0.80     # per rad (+elevator -> nose-up moment)
C_REF = 0.52          # reference chord/diameter for the moment, m

# Static-trim control effectiveness (rad of incidence per rad of deflection).
# In steady trim Cm = 0  =>  alpha = -(CM0 + CM_DELTA_E*delta_e) / CM_ALPHA.
# With the signs above this is positive: +elevator -> +alpha -> +lift. We expose
# the ratio as an explicit gain so a guidance/autopilot author can reason about
# it directly.
ALPHA_PER_ELEVATOR = -CM_DELTA_E / CM_ALPHA   # = +1.78 rad alpha / rad elevator
BETA_PER_RUDDER = 1.0                          # rad sideslip / rad rudder

# Safety clamps so a saturated fin command cannot produce absurd incidence.
MAX_ALPHA = 0.35      # ~20 deg
MAX_BETA = 0.35       # ~20 deg


@dataclass(frozen=True)
class AeroForces:
    """Aerodynamic force magnitudes (N) and pitching moment (N*m)."""
    lift: float            # perpendicular to velocity, in the pitch plane
    drag: float            # opposing velocity
    side_force: float      # perpendicular to velocity, in the yaw plane
    pitching_moment: float # about the CG (unused in 3-DoF; for 6-DoF later)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(value, limit))


def trim_alpha(delta_elevator: float) -> float:
    """Angle of attack (rad) the airframe statically trims to for an elevator
    deflection (rad). Point-mass quasi-steady assumption (see module docstring).
    """
    return _clamp(ALPHA_PER_ELEVATOR * delta_elevator, MAX_ALPHA)


def trim_beta(delta_rudder: float) -> float:
    """Sideslip angle (rad) the airframe statically trims to for a rudder
    deflection (rad)."""
    return _clamp(BETA_PER_RUDDER * delta_rudder, MAX_BETA)


def lift_coefficient(alpha: float) -> float:
    """Lift coefficient at angle of attack `alpha` (rad)."""
    return CL0 + CL_ALPHA * alpha


def drag_coefficient(alpha: float, mach: float) -> float:
    """Drag coefficient: parasite + induced + transonic wave-drag bump."""
    cl = lift_coefficient(alpha)
    cd = CD0 + K_INDUCED * cl * cl
    # Smooth transonic rise centred near M = 1; negligible deep in the subsonic
    # cruise regime, but keeps the model well-behaved if speed ever climbs.
    delta = (mach - M_WAVE_CENTER) / M_WAVE_WIDTH
    cd += CD_WAVE_PEAK * math.exp(-0.5 * delta * delta)
    return cd


def pitching_moment_coefficient(alpha: float, delta_elevator: float) -> float:
    """Pitching-moment coefficient (about CG). Exposed for the 6-DoF upgrade."""
    return CM0 + CM_ALPHA * alpha + CM_DELTA_E * delta_elevator


class Aerodynamics:
    """
    Aerodynamic force model for one airframe.

    Stateless apart from the reference geometry; one instance can be shared by
    the whole simulation. `reference_area` may be overridden if a profile ever
    grows an explicit aero section (today MissileProfile has none, so the
    representative constant above is used).
    """

    def __init__(self, reference_area: float = S_REF,
                 reference_chord: float = C_REF) -> None:
        self.reference_area = float(reference_area)
        self.reference_chord = float(reference_chord)

    # Trim helpers (module-level functions exposed as methods for convenience).
    @staticmethod
    def trim_alpha(delta_elevator: float) -> float:
        """See module-level `trim_alpha`."""
        return trim_alpha(delta_elevator)

    @staticmethod
    def trim_beta(delta_rudder: float) -> float:
        """See module-level `trim_beta`."""
        return trim_beta(delta_rudder)

    def compute(
        self,
        mach: float,
        alpha: float,
        beta: float,
        dynamic_pressure: float,
        delta_elevator: float = 0.0,
    ) -> AeroForces:
        """
        Compute aerodynamic forces and pitching moment.

        Args:
            mach: Mach number (V / speed_of_sound)
            alpha: angle of attack, rad
            beta: sideslip angle, rad
            dynamic_pressure: q = 0.5 * rho * V^2, Pa
            delta_elevator: elevator deflection, rad (for the moment term only)

        Returns:
            AeroForces with lift/drag/side_force in N, pitching_moment in N*m.
        """
        q_s = dynamic_pressure * self.reference_area

        cl = lift_coefficient(alpha)
        cd = drag_coefficient(alpha, mach)
        cy = CY_BETA * beta
        cm = pitching_moment_coefficient(alpha, delta_elevator)

        return AeroForces(
            lift=q_s * cl,
            drag=q_s * cd,
            side_force=q_s * cy,
            pitching_moment=q_s * self.reference_chord * cm,
        )
