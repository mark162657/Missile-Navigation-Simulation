import numpy as np

from src.terrain.dem_loader import DEMLoader

class GPSReceiver():
    def __init__(self, horizontal_accuracy: float=2.3, vertical_accuracy: float=3.1):
        """

        Note: horizontal and vertical_accuracy value may not be accurate. I don't have a true reliable source,
        so based on Perplexity Pro Research's document currently (will definitely be updated or be filled by user)

        Args:
            - dem_loader: DEMLoader object
            - horizontal_accuracy: horizontal accuracy in meter
            - vertical_accuracy: vertical accuracy in meter (was kinda surprised to know GPS can measure altitude by
            trilateration of satellite).
        """

        self.dem_loader = DEMLoader()
        self.h_std = horizontal_accuracy
        self.v_std = vertical_accuracy

    def get_raw_measurement(self, location: tuple[float, float]) -> np.ndarray:
        """
        We will receive absolute location in GPS form from main simulation loop, it keep track of our TRUE location
        (as it is only a simulation, it has no REAL GPS SENSOR, noway it will know where it is by itself).
        Then we will apply the noise matrix to the true location to 'simulate' the GPS sensor with error.

        Args:
            - location: tuple (lat, lon), true location handled in simulation loop

        Return:
            - numpy array with matrix of true location applied with normalized random noise matrix
        """

        # Obtain latitude and longitude from pixel, and altitude from lon, lat using get_elevation
        lat, lon = location[0], location[1]
        alt = self.dem_loader.get_elevation(lat, lon)

        # Wrap true position into a matrix
        true_pos = np.array([lat, lon, alt]) # x, y, z if in pixel

        # Define normalized random noise
        noise = np.array([
            np.random.normal(0, self.h_std),
            np.random.normal(0, self.h_std),
            np.random.normal(0, self.v_std)
        ])

        return true_pos + noise