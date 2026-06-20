import numpy as np
import math

from simulation.sensors.gps_receiver import GPSReceiver

class GPS:
    def __init__(self):
        """
        Assume (feature might be added in future, depends):
            - signal travel time does not exists + processing delay
            - PPS + WAGE enhancement signal (acheiving around 1 meter accuracy)
            - ignore signal processing layer, solved triangulation and output Cartesian coordinates (x, y, z) directly
        """

        # Check if GPS is active and jammed or not
        self.has_signal = True
        self.is_jammed = False

        # GPS receiver
        self.receiver = GPSReceiver()

    def get_gps_location(self, true_location: list[float, float, float]) -> list[float, float, float]:

        # Pull measurement data from GPS receiver
        raw_measurement = self.receiver.get_raw_measurement(true_location)

        if self.detect_jammed(raw_measurement):
            self.is_jammed = True
            return None

        return raw_measurement

    def detect_jammed(self, measurement: np.ndarray) -> bool:
        """TODO: Future implementation"""
        return False




        


    
    