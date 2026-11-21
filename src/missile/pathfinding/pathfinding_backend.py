from curses.panel import top_panel
from turtle import right

import numpy as np
import time

from paths import PROJECT_ROOT
from terrain.dem_loader import DEMLoader
from terrain.coordinates import CoordinateSystem

# Main class that main file will call
class Pathfinding:
    def __init__(self, DEM_NAME: str) -> None:
        """
        Args:
            DEM_NAME: the name of the DEM file

        Initialise DEM route -> Initialise lookup table -> C++ engine
        """
        
        # Load dem, requesting dem file name:
        tif_path = PROJECT_ROOT / 'data' / 'dem' / f'{DEM_NAME}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem

        self.coordinates = CoordinateSystem()

        # --- Initialise Lookup Table ---
        self.rows, self.cols = self.dem_loader.shape

        start_lat = self.dem_loader.transform[5] # top edge latitude (row 0)
        pixel_height = self.dem_loader.transform[4]

        # Setup lookup table (1D NumPy array in float64)
            # Row 0: start_lat
            # Row 1: start_lat + pixel_height
            # Row 2: start_lat + 2 * pixel_height
            # And so on
        row_indices = np.arange(self.rows,
                                 dtype=np.float64) # double: float64

        self.latitudes = start_lat + (row_indices * pixel_height)

        self.meter_per_y = (np.array([self.coordinates.meter_per_deg_lat(lat) for lat in self.latitudes])
                            * abs(pixel_height))

        # x-axis (east/west) setup. width in meter with each latitude
        meters_per_x_raw = (np.array([self.coordinates.meter_per_deg_lon_at(lat) for lat in self.latitudes])
                            * abs(self.dem_loader.transform[0]))

        # finally, construct two x and y lookup table
        self.meters_per_x_lookup = np.ascontiguousarray(meters_per_x_raw, dtype=np.float64)
        self.meter_per_y_lookup = np.ascontiguousarray(self.meter_per_y, dtype=np.float64)