import math

from missile.state import MissileState
from terrain.coordinates import CoordinateSystem


class TargetGeometry:
    def __init__(
            self,
            target_latlon: tuple[float, float],
            coordinate: CoordinateSystem,
            target_alt: float=0.0
    ):
        self.coord = coordinate
        self.target_latlon = target_latlon                            # haversine uses this
        self.target_enu = coordinate.latlong_to_enu(*target_latlon)   # ENU / path methods use this
        self.target_alt = target_alt


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
            target_alt: float,
            meter: bool=True
    ):

        return self._haversine_meter(
            self.target_latlon,
            (state.est_lat, state.est_lon, state.est_alt),
            self.target_alt,
            meter
        )

    def ground_range(self, state: MissileState) -> float:
        pass

    def slant_range(self, state: MissileState) -> float:
        pass

    def bearing(self, state: MissileState) -> float:
        pass

    def _haversine_meter(
            self,
            target_latlon: tuple[float, float],
            original_latlon: tuple[float, float],
            meter: bool=True
    ) -> float:

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
            
    ):