import numpy as np

from terrain.coordinates import meter_per_deg_lat, meter_per_deg_lon_at

class GPSReceiver():
    def __init__(self, horizontal_accuracy: float=2.3, vertical_accuracy: float=3.1):
        """

        Note: horizontal and vertical_accuracy value may not be accurate. I don't have a true reliable source,
        so based on Perplexity Pro Research's document currently (will definitely be updated or be filled by user)

        Args:
            - horizontal_accuracy: horizontal accuracy in meter
            - vertical_accuracy: vertical accuracy in meter (was kinda surprised to know GPS can measure altitude by
            trilateration of satellite).
        """

        self.h_std = horizontal_accuracy
        self.v_std = vertical_accuracy


    def get_raw_measurement(self, true_location: list[float, float, float]) -> np.ndarray:
        """
        We will receive absolute location in GPS form from main simulation loop, it keep track of our TRUE location
        (because it is only a simulation, it has no REAL GPS SENSOR, no way it will know where it is by itself).
        Then we will apply the noise matrix to the true location to 'simulate' the GPS sensor with error.

        Args:
            - true_location: tuple (lat, lon), true uncorrupted location handled in simulation loop

        Return:
            - numpy array with matrix of true location applied with normalized random noise matrix
        """


        # Wrap true position into a matrix [lat deg, lon deg, alt m]
        true_pos = np.array(true_location, dtype=float)

        # Horizontal accuracy and lat/lon has different unit. The former one uses meter, the lat/lon uses degrees
        # Sample the error in meters, then convert to degrees using the
        # WGS-84 meters-per-degree scale built in the coordinates.py.
        ref_lat = float(true_pos[0])
        lat_noise_m = np.random.normal(0, self.h_std)
        lon_noise_m = np.random.normal(0, self.h_std)
        alt_noise_m = np.random.normal(0, self.v_std)

        noise = np.array([
            lat_noise_m / meter_per_deg_lat(ref_lat),
            lon_noise_m / meter_per_deg_lon_at(ref_lat),
            alt_noise_m
        ])

        return true_pos + noise