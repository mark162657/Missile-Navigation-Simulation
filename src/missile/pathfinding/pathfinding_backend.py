from curses.panel import top_panel
from turtle import right

import numpy as np
import time

from paths import PROJECT_ROOT
from terrain.dem_loader import DEMLoader

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

        # load into ascontiguousarray for C++, float32 for dem
        self.dem = np.ascontiguousarray(dem.data, dtype=np.float32)
        
        # initiate the lookup table and column value
        self.rows = self.dem_loader.shape[0] # shape: (rows, columns)
        self.cols = self.dem_loader.shape[1]

        # y-axis (latitude) resolution, meter per pixel.
        self.meter_per_y = abs(self.dem_loader.transform[4] * 111320)

        start_lat = self.dem_loader.transform[5] # top edge latitude (row 0)
        pixel_height = self.dem_loader.transform[4]

        # Setup lookup table
            # Row 0: start_lat
            # Row 1: start_lat + pixel_height
            # Row 2: start_lat + 2 * pixel_height
            # And so on
        row_indices = np.arange(self.rows,
                                 dtype=np.float64) # double: float64
        self.latitudes = start_lat + (row_indices * pixel_height)

        # x-axis (east/west) setup. width in meter with each latitude
        base_width_meters = abs(self.dem_loader.transform[0] * 111320)

        # Lookup table for C++
        meters_per_x_raw = base_width_meters * np.cos(np.radians(self.latitudes))
        self.meters_per_x_lookup = np.ascontiguousarray(meters_per_x_raw, dtype=np.float64)