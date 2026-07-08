"""
Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
"""
import math

import numpy as np

from missile.state import MissileState
from terrain.coordinates import CoordinateSystem


class TargetGeometry:
    def __init__(
            self,
            target_latlon: tuple[float, float],
            coordinate: CoordinateSystem,
            target_alt: float=0.0,
            path: "np.ndarray | None"=None,
    ):
        """
        Args:
            target_latlon: (lat, lon) of the aimpoint, degrees.
            coordinate: shared ENU frame (MUST be the same instance the path
                follower uses so the ENU coordinates agree).
            target_alt: target altitude, meters MSL.
            path: optional planned route as an (N, >=3) [lat, lon, ground_elev]
                array (the TrajectoryGenerator output). Needed only by the
                remaining_*_distance methods; can also be supplied later via
                set_path(). Left None, the direct_* methods still work.
        """
        self.coord = coordinate
        self.target_latlon = target_latlon                            # haversine uses this
        self.target_enu = np.asarray(coordinate.latlong_to_enu(*target_latlon), dtype=float)
        self.target_alt = target_alt

        # Route geometry, populated by set_path(): node ENU + the reverse
        # cumulative arc length from each node onward to the target.
        self._nodes_enu: np.ndarray | None = None     # (N, 2) east/north
        self._nodes_alt: np.ndarray | None = None     # (N,) ground elevation MSL
        self._rem_arc_2d: np.ndarray | None = None    # (N,) arc length node -> target, ground
        self._rem_arc_3d: np.ndarray | None = None    # (N,) arc length node -> target, with elevation
        if path is not None:
            self.set_path(path)

    # ------------------------------------------------------------------
    # Route setup (for the remaining-distance methods)
    # ------------------------------------------------------------------
    def set_path(self, path: np.ndarray) -> None:
        """
        Register the planned route and precompute per-node remaining arc lengths.

        The route is stored in the shared ENU frame; the target itself is
        appended as the terminal node so the final leg (last path point ->
        target) is included in the remaining distance. Runs once (O(N)); the
        per-call queries are then O(N) for the nearest-node search only.

        Args:
            path: (N, >=3) [lat, lon, ground_elev] array (TrajectoryGenerator output).
        """
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] < 3:
            raise ValueError(f"Expected path shape (N>=2, >=3), got {path.shape}")

        nodes_enu = np.array(
            [self.coord.latlong_to_enu(float(la), float(lo)) for la, lo in path[:, :2]],
            dtype=float,
        )
        nodes_alt = path[:, 2].astype(float)

        # Forward segment from each node to the next (the last node's "next" is
        # the target itself), so seg[i] = distance from node i to node i+1.
        fwd_enu = np.vstack([nodes_enu[1:], self.target_enu])          # (N, 2)
        fwd_alt = np.concatenate([nodes_alt[1:], [self.target_alt]])   # (N,)

        seg_2d = np.linalg.norm(fwd_enu - nodes_enu, axis=1)           # (N,)
        seg_3d = np.hypot(seg_2d, fwd_alt - nodes_alt)                 # (N,)

        # Remaining arc from node i onward = sum(seg[i:]) -> reverse cumsum.
        self._rem_arc_2d = np.cumsum(seg_2d[::-1])[::-1]
        self._rem_arc_3d = np.cumsum(seg_3d[::-1])[::-1]
        self._nodes_enu = nodes_enu
        self._nodes_alt = nodes_alt

    def _require_path(self) -> None:
        if self._nodes_enu is None:
            raise RuntimeError(
                "TargetGeometry has no path; pass path= to __init__ or call "
                "set_path() before using the remaining_*_distance methods."
            )

    def _nearest_node_index(self, pos_enu: np.ndarray) -> int:
        """Index of the path node closest to pos_enu IN THE GROUND PLANE (cross-track)."""
        dist = np.linalg.norm(self._nodes_enu - pos_enu, axis=1)
        return int(np.argmin(dist))


    def direct_ground_distance(
            self,
            state: MissileState,
            meter: bool=True
    ) -> float:
        """
        Get direct ground (2D) great-circle distance from the missile's current
        estimated position to the target, using the Haversine formula.

        Args:
            state: current missile state (uses est_lat / est_lon)
            meter: True -> meters, False -> kilometers

        Return:
            Great-circle ground distance to target.
        """
        return self._haversine_meter(
            self.target_latlon,
            (state.est_lat, state.est_lon),
            meter
        )

    def direct_3d_distance(
            self,
            state: MissileState,
            meter: bool=True
    ) -> float:
        """
        Direct 3D slant distance from the missile's current estimated position
        to the target: great-circle ground distance combined with the altitude
        difference.

        Args:
            state: current missile state (uses est_lat / est_lon / est_alt)
            meter: True -> meters, False -> kilometers

        Return:
            Straight-line slant distance to target.
        """
        return self._3d_haversine(
            self.target_latlon,
            self.target_alt,
            (state.est_lat, state.est_lon),
            state.est_alt,
            meter
        )

    def remaining_ground_distance(
            self,
            state: MissileState,
            meter: bool=True
    ) -> float:
        """
        Remaining distance TO GO ALONG THE ROUTE (2D / ground plane):

            |missile -> nearest path node|  +  arc length(nearest node -> target)

        Unlike direct_ground_distance (straight line to the target), this
        follows the planned path, so it reflects the actual distance the
        missile still has to fly. Requires a path (see set_path()).

        Args:
            state: current missile state (uses est_lat / est_lon).
            meter: True -> meters, False -> kilometers.
        """
        self._require_path()
        pos = np.asarray(self.coord.latlong_to_enu(state.est_lat, state.est_lon), dtype=float)
        i = self._nearest_node_index(pos)

        to_path = float(np.linalg.norm(self._nodes_enu[i] - pos))
        total_m = to_path + float(self._rem_arc_2d[i])
        return total_m if meter else total_m / 1000.0

    def remaining_3d_distance(
            self,
            state: MissileState,
            meter: bool=True
    ) -> float:
        """
        Remaining route distance in 3D (with elevation):

            slant(missile -> nearest path node)  +  arc length(nearest node -> target)

        The nearest node is chosen in the ground plane (cross-track), matching
        remaining_ground_distance; the along-route arc uses the path's terrain
        elevation profile, and the first leg adds the missile's height above
        that node. Requires a path (see set_path()).

        Args:
            state: current missile state (uses est_lat / est_lon / est_alt).
            meter: True -> meters, False -> kilometers.
        """
        self._require_path()
        pos = np.asarray(self.coord.latlong_to_enu(state.est_lat, state.est_lon), dtype=float)
        i = self._nearest_node_index(pos)

        to_path_2d = float(np.linalg.norm(self._nodes_enu[i] - pos))
        to_path_3d = math.hypot(to_path_2d, float(self._nodes_alt[i]) - state.est_alt)
        total_m = to_path_3d + float(self._rem_arc_3d[i])
        return total_m if meter else total_m / 1000.0

    def _haversine_meter(
            self,
            target_latlon: tuple[float, float],
            original_latlon: tuple[float, float],
            meter: bool=True
    ) -> float:
        """
        Great-circle ground distance between two geographic points using the
        Haversine formula (spherical Earth, mean radius 6371 km).

        Uses the atan2 form of the central angle, which stays numerically stable
        for both very short and near-antipodal separations.

        Args:
            target_latlon:   (lat, lon) of the target, degrees
            original_latlon: (lat, lon) of the origin point, degrees
            meter:           True -> meters, False -> kilometers

        Return:
            Surface (great-circle) distance between the two points.
        """
        if meter:
            R = 6371000.0  # Earth radius in meters
        else:
            R = 6371.0

        tlat, tlon = target_latlon
        olat, olon = original_latlon

        # Convert degrees to radians
        dLat = math.radians(tlat - olat)
        dLon = math.radians(tlon - olon)
        olat = math.radians(olat)
        tlat = math.radians(tlat)

        # Apply Haversine formula
        a = math.sin(dLat / 2) ** 2 + math.cos(olat) * math.cos(tlat) * math.sin(dLon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def _3d_haversine(
            self,
            target_latlon: tuple[float, float],
            target_alt: float,
            original_latlon: tuple[float, float],
            original_alt: float,
            meter: bool=True
    ) -> float:
        """
        3D straight-line (slant) distance built on top of the 2D haversine:
        the great-circle ground distance and the altitude difference form the
        two legs of a right triangle.

            slant = sqrt(ground^2 + d_alt^2)

        Args:
            target_latlon:   (lat, lon) of the target, degrees
            target_alt:      target altitude, meters MSL
            original_latlon: (lat, lon) of the missile, degrees
            original_alt:    missile altitude, meters MSL
            meter:           True -> meters, False -> kilometers
        """
        ground = self._haversine_meter(target_latlon, original_latlon, meter)

        d_alt = target_alt - original_alt
        if not meter:
            d_alt /= 1000.0  # altitudes are meters; match the km ground leg

        return math.hypot(ground, d_alt)