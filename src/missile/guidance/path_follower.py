import numpy as np
import math

from missile.planning.trajectory import TrajectoryGenerator
from missile.profile import MissileProfile
from missile.state import MissileState
from terrain.coordinates import CoordinateSystem

class PathFollower:
    def __init__(
            self,
            trajectory: TrajectoryGenerator,
            profile: MissileProfile,
            coordinate: CoordinateSystem,
            l1_distance: float=300.0
    ):
        self.trajectory = trajectory
        self.profile = profile

    def _l1_lateral_accel(self):
        


