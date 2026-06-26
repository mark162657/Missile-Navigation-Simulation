import numpy as np
import numpy.typing as npt

from missile.profile import MissileProfile
from missile.state import MissileState

class AutoPilot:
    def __init__(self, trajectory: npt.NDArray[np.float64], profile: MissileProfile, state: MissileState):
        """
        Args:
            trajectory: a non-writable numpy array of shape (N, 3), with lat/lon/terrain_elevation
        """
        self.trajectory = trajectory
        self.profile = MissileProfile()
        self.state = MissileState()





