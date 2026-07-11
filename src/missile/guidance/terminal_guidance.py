"""

Assuming A. Optimal Guidance Law for Lag-Free Autopilot. As our autopilot is designed to be lag-free in acceleration.

Source:
    Ryoo, C., Cho, H., & Tahk, M. (2005). Optimal Guidance Laws with Terminal Impact Angle
        Constraint. Journal of Guidance Control and Dynamics, 28(4),
        724–732. https://doi.org/10.2514/1.8392
"""

import math
from dataclasses import dataclass

import numpy as np

from missile.guidance.target_geometry import TargetGeometry
from missile.state import MissileState
from missile.profile import MissileProfile

_G = 9.80665


@dataclass
class TerminalCommand:
    """
    Output for ControlInput's acceleration command (turn and climb) and target speed, stored in dataclass.
    The output from the acceleration command is stored and assigned here.

    Args:
        accel_turn: lateral (horizontal) accel command, m/s^2
        accel_climb: vertical accel command (incl. gravity hold), m/s^2
        target_spd: commanded speed, m/s
    """
    accel_turn: float
    accel_climb: float
    target_spd: float


class TerminalGuidance:
    def __init__(
            self,
            profile: MissileProfile,
            state: MissileState,
            target: TargetGeometry,
            impact_angle_deg: float,
    ):
        self.profile = profile
        self.state = state
        self.target = target


