import numpy as np


"""
Radar altimeter in Tomahawk Cruise Missile can only sense altitude of a point, yet here,
we assume somehow, our radar altimeter can sensed a larger patch to simulate our simplified elevation-based DSMAC.
"""
class RadarAltimeter:
    def __init__(self, vertical_std: float=1.0):
        """
        Arg:
            current_altitude: current altitude in meters, the TRUE absolute altitude that we took and process
            to simulate the error and uncertainty in radar altimeter
            vertical_std: standard deviation of the vertical altitude measurement in meters, default to 1.0
        """

        self.std_mea = vertical_std

    def get_altimeter_agl(self, true_curr_agl, true_dem_elev) -> float:
        """
        Get the altitude above ground in meters, with noise. Simulating the radar altimeter.
        Args:
            true_curr_agl: the current 'altitude above ground' in meters of the missile
            true_dem_elev: the ground elevation of the dem map

        Return:
            The altitude above ground in meters, with noise.
        """

        agl = true_curr_agl - true_dem_elev

        # Add Gaussian noise
        noise = np.random.normal(0, self.std_mea)
        return agl + noise






