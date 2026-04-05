"""
Simple test to run pathfinding with GPS coordinate, returning simplified path and time taken.


I handcoded this without AI... very tiring lol
"""

import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.pathfinder.pathfinding_backend import Pathfinding

def test_pathfinding_gps(start: tuple[float, float], end: tuple[float, float]):

    pf = Pathfinding()
    # Accept both GPS and direct pixel (row, col) coordinate

    start_row, start_col = pf.dem_loader.lat_lon_to_pixel(start[0], start[1])
    end_row, end_col = pf.dem_loader.lat_lon_to_pixel(end[0], end[1])

    start_loc = (start_row, start_col)
    end_loc = (end_row, end_col)

    gps_dist = pf.get_surfcae_distance(start, end)

    print("\n============================================================")
    print("DATA RECEIVED...")
    print("============================================================")
    
    # Check if coordinates are within bounds
    if not (0 <= start_row < pf.rows and 0 <= start_col < pf.cols):
        print(f"\n! ERROR: Start coordinate out of bounds!")
        return None
    
    if not (0 <= end_row < pf.rows and 0 <= end_col < pf.cols):
        print(f"\n! ERROR: End coordinate out of bounds!")
        return None
    
    print(f"\nFinding path from... \n- Pixel: ({start_row}, {start_col}) -> ({end_row}, {end_col}) \n- GPS: {start} -> {end}\n")
    print(f"GPS distance: {gps_dist} meter ({gps_dist / 1000:.2f}km)")

    print("\n============================================================")
    print("START PATHFINDING")
    print("============================================================\n")
    
    start_time = time.time()
    path = pf.find_path(start_loc, end_loc, heuristic_weight=1)
    end_time = time.time()

    time_elapsed = end_time - start_time

    print(f"\nExecution Time: {time_elapsed:.3f} seconds")
    return path

if __name__ == "__main__":

    ans = input(
        "Choose the following test length: \n"
        "1. 10km\n"
        "2. 50km\n"
        "3. 100km\n"
        "4. 500km\n"
        "5. 1000km\n"
    )

    distances = {
        "1": 10,
        "2": 50,
        "3": 100,
        "4": 500,
        "5": 1000
    }

    if ans in distances:
        km = distances[ans]

        # base coordinate
        start_gps = (56.0, 95.0)

        # degrees per km at lat 56° (longitude)
        deg_per_km_lon = 1 / (111.32 * 0.559193)  # cos(56°) = 0.559193

        # compute end point (east direction)
        end_gps = (
            start_gps[0],
            start_gps[1] + km * deg_per_km_lon
        )

        print("start_gps =", start_gps)
        print("end_gps   =", end_gps)
        test_pathfinding_gps(start_gps, end_gps)
    else:
        print("Invalid input.")
    