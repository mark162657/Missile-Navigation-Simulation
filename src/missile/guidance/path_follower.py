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
        self.path = trajectory

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
        # turn the current lat/lon position to ENU
        pos_enu = self.coord.latlong_to_enu(state.est_lat, state.est_lon)

        # ground_speed and bearing using hypot(East, North) and atan(East, North)
        enu_ground_speed = np.hypot(state.vel_east, state.vel_north)
        enu_bearing = np.arctan2(state.vel_east, state.vel_north)

        closest_idx = self._find_closest(pos_enu)
        aim_idx = self._lookahead(closest_idx, self.l1)
        target_spd = self.profile.basic.cruise_speed_ms
        target_alt = self._target_altitude(aim_idx)
        aim_pt_enu = self.traj_enu[self.aim_idx]

        lateral_accel_cmd = self._l1_lateral_accel(pos_enu, enu_bearing, enu_ground_speed, aim_pt_enu, kl=2.0)

        return lateral_accel_cmd, target_alt, target_spd

    def _l1_lateral_accel(
            self,
            pos_enu: np.ndarray,
            enu_bearing: float,
            enu_ground_speed: float,
            aim_pt_enu: np.ndarray,
            kl: float = 2.0
    ) -> float:
        """
        Mathematical definitions:

        Reference:
            Stastny, T. (2018). L1 guidance logic extension for small UAVs: handling high winds and small loiter radii.
            ArXiv.org. https://doi.org/10.48550/arxiv.1804.04209
        """
        delta = np.asarray(aim_pt_enu) - np.asarray(pos_enu)

        v_g = enu_ground_speed

        # calculate raw tracking error (eta)
        chi_l = np.arctan2(delta[0], delta[1])
        eta = np.clip(np.arctan2(np.sin(chi_l - enu_bearing), np.cos(chi_l - enu_bearing)), -np.pi/2, np.pi/2)
        a_ref = kl * v_g ** 2 / self.l1 * np.sin(eta) # main formula

        # limit the acceleration command to be within the max lateral acceleration capable
        a_max = self.profile.get_max_lateral_acceleration()

        # clamp the a_ref in max acceleration and min acceleration (-max and +max)
        return float(np.clip(a_ref, -a_max, a_max))


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

    def _find_closest(self, pos_enu, window=50) -> int:
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
        self.last_idx = closest_idx # advance

        return closest_idx

    def _lookahead(self, closest_idx, l1) -> int:
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

    def progress_tracker(self, closest_idx) -> float:
        """
        This is a helper function to track the progress of the guidance that will be used in the GUI.
        Use the anchor point (closest_idx) and the entire path distance to find the relative progress.
        """
        return closest_idx / self.traj_length * 100











