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