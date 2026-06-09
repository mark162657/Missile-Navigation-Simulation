import math
from typing import Tuple


_METER_PER_DEG_LAT = 111_320.0


class CoordinateSystem:
    """
    Local tangent-plane ENU offsets from a fixed geographic origin.

    Returns (east_m, north_m) in meters:
        - east_m  > 0  →  east of origin  |  < 0  →  west of origin
        - north_m > 0  →  north of origin |  < 0  →  south of origin

    South and west are handled by **signed** latitude/longitude deltas, not by
    taking absolute values of the scale factors. Example: origin (55°N, 100°E),
    point (54°N, 99°E) → negative north_m and negative east_m.

    This is **not** the missile-state x/y convention (x=lat°, y=lon°) and it is
    **not** the pathfinder pixel frame (which measures southward row offset as
    positive y from the DEM top-left).
    """

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon

        # Scale at the origin (used when no target latitude is available).
        self.meter_per_deg_lat = _METER_PER_DEG_LAT
        self.meter_per_deg_lon = _meter_per_deg_lon_at(origin_lat)

    @staticmethod
    def _delta_lon_deg(lon: float, origin_lon: float) -> float:
        """Shortest signed longitude difference in degrees (handles wrap)."""
        delta = lon - origin_lon
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        return delta

    def latlong_to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert geographic coordinates to local ENU meters from the origin.

        Returns (east_m, north_m). Works for any quadrant relative to the origin
        (north/east/south/west or any combination).
        """
        delta_lat = lat - self.origin_lat
        delta_lon = self._delta_lon_deg(lon, self.origin_lon)

        north_m = delta_lat * self.meter_per_deg_lat

        # Longitude scale shrinks toward the poles; evaluate at the midpoint
        # latitude of the segment so south/north offsets scale correctly.
        ref_lat = self.origin_lat + 0.5 * delta_lat
        east_m = delta_lon * _meter_per_deg_lon_at(ref_lat)

        return east_m, north_m

    def xy_to_latlong(self, east_m: float, north_m: float) -> Tuple[float, float]:
        """Convert local ENU meters from the origin back to (lat, lon) degrees."""
        lat = self.origin_lat + north_m / self.meter_per_deg_lat

        ref_lat = 0.5 * (self.origin_lat + lat)
        lon = self.origin_lon + east_m / _meter_per_deg_lon_at(ref_lat)

        return lat, lon

    def get_distance(self, orig_lon: float, orig_lat: float, dest_lon: float, dest_lat: float) -> float:
        """
            Get the distance between two points using Haversine Distance (meters).
            Which takes consider of the Earth radius and scale.

            Args:
                - orig_lon: longitude for start (origin)
                - orig_lat: latitude for start (origin)
                - dest_lon: longitude for destination
                - dest_lat: latitude for destination

            Return:
                the distance in meters
        """

        # distance between latitudes and longitudes
        d_lat = (dest_lat - orig_lat) * math.pi / 180.0
        d_lon = (dest_lon - orig_lon) * math.pi / 180.0

        # convert to radians
        orig_lat_rad = (orig_lat) * math.pi / 180.0
        dest_lat_rad = (dest_lat) * math.pi / 180.0

        # apply formulae
        a = (
            math.sin(d_lat / 2) ** 2
            + math.sin(d_lon / 2) ** 2 * math.cos(orig_lat_rad) * math.cos(dest_lat_rad)
        )
        earth_radius_m = 6_371_000.0
        return earth_radius_m * 2.0 * math.asin(math.sqrt(a))

    def get_heading(self, lat1: float, long1: float, lat2: float, long2: float) -> float:
        """Initial bearing from point 1 to point 2, degrees clockwise from north."""
        
        # convert lat1, lon1, lat2, lon2 into radian
        lat1, lat2 = math.radians(lat1), math.radians(lat2)
        long1, long2 = math.radians(long1), math.radians(long2)
        
        # calculate bearing/heading in degrees
        d_long = long2 - long1
        x = math.sin(d_long) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) *
             math.cos(d_long))
        
        theta = math.atan2(x, y)
        
        return (math.degrees(theta) + 360) % 360


def _meter_per_deg_lon_at(lat_deg: float) -> float:
    """Meters per degree of longitude at a given latitude (always positive)."""
    return _METER_PER_DEG_LAT * math.cos(math.radians(lat_deg))
