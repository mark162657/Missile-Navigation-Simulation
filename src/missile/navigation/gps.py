import numpy as np
import math

from terrain.dem_loader import DEMLoader
from simulation.sensors.gps_receiver import GPSReceiver

class GPS:
    def __init__(self):
        """
        Assume (feature might be added in future, depends):
            - signal travel time does not exists + processing delay
            - PPS + WAGE enhancement signal (acheiving around 1 meter accuracy)
            - ignore signal processing layer, solved triangulation and output Cartesian coordinates (x, y, z) directly
        """

        dem = DEMLoader()
        self.dem_loader = dem

        # Check if GPS is active and jammed or not
        self.has_signal = True
        self.is_jammed = False

        # GPS receiver
        self.receiver = GPSReceiver()

    def get_gps_location(self, location: list[float, float]) -> list[float, float]:
        if not self.is_ready():
            return None, None

        # Pull measurement data from GPS receiver
        raw_measurement = self.receiver.get_raw_measurement(location)

        if self.detect_jammed(raw_measurement):
            self.is_jammed = True
            return None

        self.last_update_time = self.timer.get_time_elapsed()
        return raw_measurement

    def detect_jammed(self, measurement: np.ndarray) -> bool:
        """TODO: Future implementation"""
        return False




        


    
    