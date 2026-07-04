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

    @staticmethod
    def _flight_path_angle(self, state: MissileState):
        """
        Get the flight path angle (relative to horizontal) of the missile.
        Mathematical expression:
            γm = atan2(v_up, sqrt(v_east^2 + v_north^2))

        Args:
            state: current missile state
        """
        return np.arctan2(state.vel_up, np.sqrt(state.vel_east**2 + state.vel_north**2))

    @staticmethod
    def _radius_of_curvature(a: float, b: float, t: float) -> float:
        """
        Calculate the radius of curvature at a point on the terminal ellipse,
        given the ellipse parameter t (the "nearest point" comes from the
        nearest-point search; here we just evaluate the curvature there).

        For the ellipse (x, y) = (a * cos(t), b * sin(t)):
            R_C(t) = (a^2 * sin^2(t) + b^2 * cos^2(t))^(3/2) / (a * b)

        Args:
            a: semi-axis of the ellipse along x (downrange), meters
            b: semi-axis of the ellipse along y (height), meters
            t: ellipse parameter at the nearest point, radians

        Return:
            Radius of curvature R_C at parameter t, meters.

        Reference:
            Ellipse radius of curvature. (n.d.).
            SKM Classes Bangalore. https://skmclasses.weebly.com/ellipse-radius-of-curvature.html
        """
        sin_t = math.sin(t)
        cos_t = math.cos(t)
        return (a**2 * sin_t**2 + b**2 * cos_t**2) ** 1.5 / (a * b)