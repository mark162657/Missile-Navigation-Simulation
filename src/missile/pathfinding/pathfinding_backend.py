import numpy as np
import time
import pathfinder

from paths import PROJECT_ROOT
from terrain.dem_loader import DEMLoader
from terrain.coordinates import meter_per_deg_lat, meter_per_deg_lon_at

# Main class that main file will call
class Pathfinding:
    def __init__(self, DEM_NAME: str) -> None:
        """
        Args:
            DEM_NAME: the name of the DEM file

        Initialise DEM route -> Initialise lookup table -> Rust
        """
        
        # Load dem, requesting dem file name:
        tif_path = PROJECT_ROOT / 'data' / 'dem' / f'{DEM_NAME}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem
        self.dem = np.ascontiguousarray(dem.data, dtype=np.float32)  # zero-copy view of memmap

        # --- Initialise Lookup Table ---
        self.rows, self.cols = self.dem_loader.shape

        start_lat = self.dem_loader.transform[5] # top edge latitude (row 0)
        pixel_height = self.dem_loader.transform[4]
        pixel_width = abs(self.dem_loader.transform[0])

        # Setup lookup table (1D NumPy array in float64)
            # Row 0: start_lat
            # Row 1: start_lat + pixel_height
            # Row 2: start_lat + 2 * pixel_height
            # And so on
        row_indices = np.arange(self.rows,
                                 dtype=np.float64) # double: float64

        self.latitudes = start_lat + (row_indices * pixel_height)

        self.meter_per_y = (np.array([meter_per_deg_lat(float(lat)) for lat in self.latitudes])
                            * abs(pixel_height))

        # x-axis (east/west) setup. width in meter with each latitude
        meters_per_x_raw = (np.array([meter_per_deg_lon_at(float(lat)) for lat in self.latitudes])
                            * pixel_width)

        # finally, construct two x and y lookup table
        self.meters_per_x_lookup = np.ascontiguousarray(meters_per_x_raw, dtype=np.float64)
        self.meter_per_y_lookup = np.ascontiguousarray(self.meter_per_y, dtype=np.float64)

    def find_path(self, start: tuple[int, int], end: tuple[int, int], heuristic_weight: float = 1.1) -> list:
        """
        Args:
            start: (row, col) pixel coordinate of origin
            end: (row, col) pixel coordinate of target
            heuristic_weight: heuristic inflation factor passed to Rust A*
        """
        # Check for nodata: missing data in dem.
        nd = self.dem_loader.nodata
        if nd is not None and (self.dem[start] == nd or self.dem[end] == nd):
            return None

        # Bound check (if out of bound or not); self.rows and cols are the max number for row and col
        if not (0 <= start[0] < self.rows and 0 <= start[1] < self.cols):
            raise ValueError(f"Start coordinate out of bounds {self.rows} x {self.cols}: {start}")
        if not (0 <= end[0] < self.rows and 0 <= end[1] < self.cols):
            raise ValueError(f"End coordinate out of bounds {self.rows} x {self.cols}: {end}")

        start_t = time.time()
        path = pathfinder.rs_pathfinder(
            self.dem,
            self.meters_per_x_lookup,
            self.meter_per_y_lookup,
            (int(start[0]), int(start[1])),
            (int(end[0]), int(end[1])),
            float(heuristic_weight),
        )
        duration = time.time() - start_t

        if not path:
            print(f"Path not found (Rust Pathfinder returned empty list). Time: {duration:.4f}s")
            return None

        print(f"Path found! Length: {len(path)}. Time: {duration:.4f}s")
        return path

    def convert_path_to_gps(self, pixel_path: list) -> list:
        """
        Batch convert path from pixels coordinates to GPS coordinates (lat, lon).
        For map plotting (2D) and path smoothing.

        Arg:
            pixel_path: the list returned by find_path function from Pathfinder
        """
        if not pixel_path: return []

        # Unzip into two lists
        rows, cols = zip(*pixel_path)

        # Convert using Rasterio transform
        lats, lons = self.dem_loader.pixel_to_lat_lon(rows, cols)

        # Zip back into (lat, lon) tuples
        return list(zip(lats, lons))

    def get_3d_path_points(self, pixel_path: list) -> list:
        """
        Converts pixel path to 3D World Coordinates (Z-Up Physics Standard).
        Format: [(x, y, z), ...]

        x = Longitude (Meters relative to top-left)
        y = Latitude (Meters relative to top-left) - Previously Z
        z = Altitude (Meters above sea level - from DEM) - Previously Y

        Arg:
            pixel_path: the list that contains the path (by pixels) found by pathfinding algorithm
        """
        if not pixel_path: return []

        path_3d = []

        for row, col in pixel_path:
            # Z = Altitude
            altitude = float(self.dem[row, col])  # self.dem[] to obtain the pixel's altitude

            # Y = Latitude displacement (row * meters_per_y)
            y_pos = row * self.meter_per_y

            # X = Longitude displacement (col * meter_per_x_lookup)
            x_pos = col * self.meters_per_x_lookup[row]

            path_3d.append((x_pos, y_pos, altitude))

        return path_3d


