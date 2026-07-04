import math
import numpy as np

from guidance.target_geometry import TargetGeometry
from missile.state import MissileState
from profile import MissileProfile

_G = 9.80665

class TerminalGuidance:
    def __init__(
            self,
            target: TargetGeometry,
            profile: MissileProfile,
            impact_angle_deg: float,
            approach_azimuth_rad: float,

    ):
        pass

    def update(self, state: MissileState, dt: float):
        pass

    def terminal_init_range(self):
        pass

    def _flight_path_angle(self, state: MissileState):
        """
        Get the flight path angle (relative to horizontal) of the missile.
        Mathematical expression:
            γm = atan2(v_up, sqrt(v_east^2 + v_north^2))

        Args:
            state: current missile state
        """
        return np.artan2(state.vel_up, np.sqrt(state.vel_east**2 + state.vel_north**2))

    def _radius_of_curvature(self):
        """
        Calculate the radius of curvature at the “nearest” point on ellipse

        Mathematical expression:
            R_C(t) = (a^2 * sin^2 t + b^2 * cos^2 t)^(3/2) / (a * b)

        Reference:
            Ellipse radius of curvature. (n.d.).
            SKM Classes Bangalore. https://skmclasses.weebly.com/ellipse-radius-of-curvature.html
        """
        pass