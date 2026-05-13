import numpy as np

from pathlib import Path
from navigation.kalman_filter import KalmanFilter
from navigation.tercom import TERCOM
from navigation.gps import GPS
from navigation.ins import INS
from src.control.timer import InternalTimer
from src.terrain.dem_loader import DEMLoader

class NavigationComputer:
    def __init__(self, start_gps: tuple[float, float, float],  dem_name: str, gps_freq_hz: int=5, ins_freq_hz: int=500,
                 tercom_freq_hz: int=1):
        """
        Args:
            start_gps: Starting GPS coordinates [lat, lon, altitude] corespondent to [x, y, z]
            dem_name: name of DEM
            gps_freq_hz: frequency of GPS measurements in Hz
            ins_freq_hz: frequency of INS measurements in Hz
            tercom_freq_hz: frequency of TERCOM measurements in Hz
        """

        self.start_gps = start_gps

        tif_path = Path(__file__).parent.parent.parent / 'data' / 'dem' / f'{dem_name}'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem

        # Set the update freq
        self.gps_period = 1.0 / gps_freq_hz
        self.ins_period = 1.0 / ins_freq_hz
        self.tercom_period = 1.0 / tercom_freq_hz

        # Initialise each navigation system
        self.gps = GPS()
        self.ins = INS(0.0, 0.0, 0.0) # initiate default relative position at 0, 0, 0
        self.tercom = TERCOM()
        self.KF = KalmanFilter()


        # Initialise timing checkpoints
        self.next_ins = 0.0
        self.next_gps = 0.0
        self.next_tercom = 0.0

    def run_navigation_loop(self, mission_terminated: bool=False, run_seconds: int=10000) -> None:
        """
        Run the navigation loop for a fixed amount of elapsed time.

        Args:
            run_seconds: Total runtime duration, in seconds, measured from the
                moment `self.timer.start()` is called. So this combines with
                condition checks, prevents the loop to endlessly run forever.
                We set it to 1,0000-second default, so it will stop in 1,0000 seconds,
                which is 2.78 hrs.

            mission_terminated: If True, the navigation loop will terminate
        """

        # START THE TIMER at this point. Based on when Navigation Computer started while launched.
        self.timer.start()

        while not mission_terminated:
            now = self.timer.get_time_elapsed()
            if now >= run_seconds:
                break

            if now >= self.next_ins:
                # TODO: basic ins and kf prediction and update
                self.next_ins += self.ins_period

            if now >= self.next_gps and self.gps.detect_jammed() is False:
                # TODO: basic gps and kf check and update
                self.next_gps += self.gps_period

            if now >= self.next_tercom:
                # TODO: basic TERCOM and kf check and update
                self.next_tercom += self.tercom_period

    def get_filtered_position(self) -> tuple[float, float, float]:
        """
        Return:
            Returns best-estimated location by Kalman Filter, tuple(lat, lon, alt).
        """



    def _check_terrain_roughness(self, half_search_size: int=25) -> bool:
        """
        Check for terrain roughness to determine whether the terrain is rough enough to conduct accurate TERCOM.
        TERCOM is highly based on terrain signature, a flat terrain with cause error and inaccuracy.

        Args:
            half_search_size: half-search size of terrain for roughness determination, in pixel.
        """

        # Check for ocean or not
        self.dem_loader.get_elevation_patch(half_search_size)







