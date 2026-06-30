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
        # dealing with path
        self.path = trajectory.get_trajectory()
        self.path_length = len(self.path)

        # set up the profile and coordinate system
        self.profile = profile
        self.coord = coordinate

        # convert path coordinates to enu
        if self.path.ndim != 2 or self.path.shape[1] < 2:
            raise ValueError(f"Expected trajectory shape (N, >=2), got {self.path.shape}")

        lat_lon = self.path[:, :2] # all rows, column from idx 0 up to 2
        self.traj_enu = np.asarray(
            [self.coord.latlong_to_enu(float(lat), float(lon)) for lat, lon in lat_lon], dtype=float
        )
        self.traj_enu.setflags(write=False)

        # extract ground elevation from path
        self.ground_elev = self.path[:, 2]

        self.l1 = l1_distance
        self.last_idx = 0

    def update(self):
        """

        """
        target_alt = self._target_altitude()
        target_spd = ()

    def _l1_lateral_accel(
            self,
            pos_enu: np.ndarray,
            heading: float,
            speed: float,
            aim_pt
    ) -> float:
        pass

    def _target_altitude(self, aim_idx: int) -> float:
        """
        Return target altitude, which is the ground elevation + preferred altitude AGL

        Args:
            aim_idx: the current index of path data

        Return:
            The target altitude.
        """
        pref_alt = self.profile.preferred_agl()
        return self.ground_elev[aim_idx] + pref_alt

    def enter_terminal_guidance(self):
        """
        A handoff to handle the normal in-flight guidance to terminal guidance for a final attack angle and detonation.
        """
        pass






