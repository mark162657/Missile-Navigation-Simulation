import numpy as np
import time
from pathlib import Path
from ..terrain.dem_loader import DEMLoader

# --- Import C++ Backend ---
try:
    from . import missile_backend
    CPP_AVAILABLE = True
except ImportError as e:
    print("---------- ERROR! ----------")
    print(f"  WARNING: C++ Engine not found ({e}).")
    print("   Please compile the C++ code in src/guidance/cpp/ first.")
    print("   Locate src/guidance/cpp/ folder at terminal and enter 'make'.")
    print("   Ensure the .so file is in src/guidance/")
    print("----------------------------")

    CPP_AVAILABLE = False

# Main class that main file will call
class Pathfinding:
    def __init__(self, DEM_NAME: str) -> None:
        """
        Args:
            - dem_name: the name of the DEM file
        """
        
        # Load dem, requesting dem file name:
        tif_path = Path(__file__).parent.parent.parent / 'data' / 'dem' / f'{DEM_NAME}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem

        # load into ascontiguousarray for C++, flaot32 for dem
        self.dem = np.ascontiguousarray(dem.data, dtype=np.float32)
        
        # initiate the lookup table and column value
        self.rows = self.dem_loader.shape[0] # shape: (rows, columns)
        self.cols = self.dem_loader.shape[1]

        # y-axis (latitude) resolution, meter per pixel. Previously meter_per_z
        self.meter_per_y = abs(self.dem_loader.transform[4] * 111320)

        start_lat = self.dem_loader.transform[5]
        pixel_height = self.dem_loader.transform[4]


        # Setup lookup table
        row_indices = np.arange(self.rows, dtype=np.float64) # double: float64
        self.latitudes = start_lat + (row_indices * pixel_height)

        # x-axis (east/west) setup. width in meter with each latitude
        base_width_meters = abs(self.dem_loader.transform[0] * 111320)

        # Lookup table for C++
        meters_per_x_raw = base_width_meters * np.cos(np.radians(self.latitudes))
        self.meters_per_x_lookup = np.ascontiguousarray(meters_per_x_raw, dtype=np.float64)

        # Initiate C++ engine
        if CPP_AVAILABLE:
            print(f"Initiating C++ Engine with {self.rows * self.cols} pixels...")

            # start timer: boot up time (measure hard drive speed)
            start_time = time.time()

            # Sending data to CPP class
            self.engine = missile_backend.PathfinderCPP(
                self.dem,
                self.meters_per_x_lookup,
                self.meter_per_y,
                self.rows,
                self.cols
            )

            print(f"C++ Engine Ready ({time.time() - start_time:.4f}s)")

        else:
            self.engine = None

    def find_path(self, start: tuple[int, int], end: tuple[int, int], heuristic_weight: float=2) -> list:
        """
        Main interface to run the A* algorithm, serve as a caller for receiving info and calling the C++ engine.
        Args:
            - start: (row, col) tuple pair for pixel
            - end: (row, col) tuple pair for pixel
            - heuristic_weight: multiplies the heuristic cost; set to 2.0 for faster, directed searches or 1.0 for
            the guaranteed shortest path (currently no difference lol).
            The higher the heuristic_weight, potentially, the compile time increases.
        """

        if not self.engine:
            raise RuntimeError("C++ Engine is not loaded. Cannot run pathfinding.")
        
        # Bound check (if out of bound or not); self.rows and cols are the max number for rows and cols
        if not (0 <= start[0] < self.rows and 0 <= start[1] < self.cols):
            raise ValueError(f"Start coordinate out of bounds {self.rows} x {self.cols}: {start}")
        if not (0 <= end[0] < self.rows and 0 <= end[1] < self.cols):
            raise ValueError(f"End coordinate out of bounds {self.rows} x {self.cols}: {start}")
        
        # Convert Tuples to paked index expected by C++ (saving memory compared to tuple)
        start_idx = start[0] * self.cols + start[1]
        end_idx = end[0] * self.cols + end[1]

        # Calling C++ Engine
        start_t = time.time() # start timer for recording the compile time, operation time (measure CPU / algorithm efficiency)
        path = self.engine.find_path(start_idx, end_idx, heuristic_weight) # calling the exposed find_path function in C++ (not this Python function)
        duration = time.time() - start_t
        
        if not path:
            print(f"Path not found (C++ returned empty list). Time: {duration:.4f}s")
            return None
        
        print(f"Path found! Length: {len(path)}. Time: {duration:.4f}s")
        return path
    
    def convert_path_to_gps(self, pixel_path: list) -> list:
        """
        Batch convert path from pixels coordinates to GPS coordinates (lat, lon).
        For map plotting (2D) and path smoothing.
        
        Arg:
            - pixel_path: the list returned by find_path function from C++
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
            - pixel_path: the list that contains the path (by pixels) found by pathfinding algorithm
        """
        if not pixel_path: return []
        
        path_3d = []
        
        for row, col in pixel_path:
            # Z = Altitude
            altitude = float(self.dem[row, col]) # self.dem[] to obtain the pixel's altitude 
            
            # Y = Latitude displacement (row * meters_per_y)
            y_pos = row * self.meter_per_y
            
            # X = Longitude displaceemnt (col * meter_per_x_lookup)
            x_pos = col * self.meters_per_x_lookup[row]
            
            path_3d.append((x_pos, y_pos, altitude))
            
        return path_3d
