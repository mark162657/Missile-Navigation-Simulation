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
            lookahead_dist: float=300.0
    ):
        """
        Initialising necessary data, including convert entire trajectory from lat/lon to ENU, and
        calculating its distance, and other basic actions.

        Args:
            trajectory: the imported trajectory found by pathfinder
            profile: simply missile profile / specs
            coordinate: the coordinate system used to convert lat/lon to ENU...
            lookahead_dist: L1 distance, which guidance aims
        """
        # dealing with path
        self.path = trajectory.get_trajectory()

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
        self.traj_length = self.traj_enu.shape[0]
        # extract ground elevation from path
        self.ground_elev = self.path[:, 2]

        self.l1 = lookahead_dist
        self.last_idx = 0

    def update(self, state: MissileState):
        """

        """
        pos_enu = self.coord.latlong_to_enu(state.est_lat, state.est_lon)

        target_alt = self._target_altitude()
        target_spd = self.profile.basic.cruise_speed_ms

        aim_idx = self._lookahead(self.last_idx, self.l1)



    def _l1_lateral_accel(
            self,
            pos_enu: np.ndarray,
            heading: float,
            target_speed: float,
            aim_idx
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

    def _find_closest(self, pos_enu, window=50):
        """
        The purpose is to find the closest point on the path to the missile's position.
        As the wind and turbulence push the missie off-course, the missile needs to determine the closest point it
        is anchoring to.
        Search through 50 points (window)
        """

        end = min(self.last_idx + window, self.traj_length)
        seg = self.traj_enu[self.last_idx:end] # search traj_enu from last idx to end idx
        dist = np.linalg.norm(seg - pos_enu, axis=1)
        closest_idx = self.last_idx + int(np.argmin(dist))
        return closest_idx



    def _lookahead(self, closest_idx, l1):
        """
        L1: lookahead distance.
        Dist compute the distance from this point to the next point by conducting vector norm.
        Subtracting one point vector from the next point vector.

        closest_idx -> L1 meters -> aim_idx (target)
        """
        i, dist = closest_idx, 0.0
        while  i < self.traj_length - 1 and dist < l1:
            dist += np.linalg.norm(self.traj_enu[i+1] - self.traj_enu[i])
            i += 1

        return i











