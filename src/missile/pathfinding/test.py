import pathfinder
from terrain.dem_loader import DEMLoader

dem = DEMLoader("srtm_43_02.tif")
print(pathfinder.rs_pathfinder(dem.data, (100, 200)))
print(dem.data[100, 200])