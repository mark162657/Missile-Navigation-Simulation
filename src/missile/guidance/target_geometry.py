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