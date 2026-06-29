import numpy as np
import math

from missile.planning.trajectory import TrajectoryGenerator
from missile.profile import MissileProfile
from terrain.coordinates import CoordinateSystem

class PathFollower:
    def __init__(
            self,
            trajectory: TrajectoryGenerator,
            profile: MissileProfile,
            coordinate: CoordinateSystem
    ):
        self.trajectory = trajectory
        self.profile = profile

