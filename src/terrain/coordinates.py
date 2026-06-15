import math
from typing import Tuple

class CoordinateSystem:
    """
    Local tangent-plane ENU offsets from a fixed geographic origin.

    Returns (east_m, north_m) in meters:
        - east_m  > 0  ->  east of origin  |  < 0  ->  west of origin
        - north_m > 0  ->  north of origin |  < 0  ->  south of origin

    South and west are handled by **signed** latitude/longitude deltas, not by
    taking absolute values of the scale factors. Example: origin (55°N, 100°E),
    point (54°N, 99°E) -> negative north_m and negative east_m.

    This is **not** MissileState geographic fields (est_lat°, est_lon°) and it is
    **not** the pathfinder pixel frame (row/col on the DEM grid).

    For meter_per_deg_lat/lon, test are performed on https://www.cqsrg.org/tools/GCDistance/
    Has proven to have only minor offset, and mostly accurate. Especially in polar area.
    """

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon

        # Scale at the origin (used when no target latitude is available).
        self.meter_per_deg_lat = meter_per_deg_lat(origin_lat)
        self.meter_per_deg_lon = meter_per_deg_lon_at(origin_lat)

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
        east_m = delta_lon * meter_per_deg_lon_at(ref_lat)

        return east_m, north_m

    def xy_to_latlong(self, east_m: float, north_m: float) -> Tuple[float, float]:
        """Convert local ENU meters from the origin back to (lat, lon) degrees."""
        lat = self.origin_lat + north_m / self.meter_per_deg_lat

        ref_lat = 0.5 * (self.origin_lat + lat)
        lon = self.origin_lon + east_m / meter_per_deg_lon_at(ref_lat)

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

def meter_per_deg_lat(lat: float) -> float:
    """
    Meter per latitude degree is similar, but not constant. To prevent any error, we will calculate meter_per_deg_lat manually using WGS-84 ellipsoid constants and further calculations.
    
    Formula used:
        -> Eccentricity Squared
        -> Degree to Radian
        -> Meridian Radius of Curvature
        -> Meters per Degree
    Sources:
        - WGS-84 Equatorial radius in meters:
            https://www.vcalc.com/wiki/vCalc/WGS-84-Earth-equatorial-radius-meters
        - WGS-84 Polar radius in meters:
            https://www.vcalc.com/wiki/vCalc/WGS-84-Earth-polar-radius
    """
    
    # equatorial radius in meters
    a = 6378137.0
    # polar radius in meters
    b = 6356752.31424518
    
    # eccentricity squared
    e2 = (a ** 2 - b ** 2) / (a ** 2)
    
    phi = lat * (math.pi / 180)

    # meridian radius of curvature
    num = a * (1 - e2)
    den = (1 - e2 * (math.sin(phi) ** 2)) ** 1.5
    m_rho = num / den

    # final meter_per_degree
    return m_rho * (math.pi / 180)

def meter_per_deg_lon_at(lat_deg: float) -> float:
    """
    Meters per degree of longitude at a given latitude. Calculated using the WGS-84 Prime vertical radius of curvature
    
    Source:
        https://www.oc.nps.edu/oc2902w/geodesy/radiigeo.pdf
    """

    a = 6378137.0
    b = 6356752.31424518

    e2 = (a ** 2 - b ** 2) / (a ** 2)
    phi = math.radians(lat_deg)

    # calculate prime vertical radius of curvature (N)
    n = a / math.sqrt(1 - e2 * (math.sin(phi) ** 2))

    return n * math.cos(phi) * (math.pi / 180)


if __name__ == "__main__":
    # Instantiate the class with dummy origin coordinates to satisfy __init__
    calc = CoordinateSystem(origin_lat=0.0, origin_lon=0.0)
    
    # Define globally accepted reference targets for validation
    test_cases = [
        {"name": "Equator", "lat": 0.0, "exp_lat": 110574.38, "exp_lon": 111319.49},
        {"name": "Melbourne", "lat": -37.8136, "exp_lat": 110985.64, "exp_lon": 87944.37},
        {"name": "North Pole", "lat": 90.0, "exp_lat": 111693.90, "exp_lon": 0.0}
    ]
    
    print("=" * 65)
    print(f"{'Location':<12} | {'Latitude Dist (m)':<18} | {'Longitude Dist (m)':<18}")
    print("=" * 65)
    
    for case in test_cases:
        # Call the standalone functions directly with just the latitude argument
        lat_res = meter_per_deg_lat(case["lat"])
        lon_res = meter_per_deg_lon_at(case["lat"])
        
        # Formatting to 2 decimal places matching real-world benchmarks
        print(f"{case['name']:<12} | {lat_res:<18,.2f} | {abs(lon_res):<18,.2f}")
        
    print("=" * 65)