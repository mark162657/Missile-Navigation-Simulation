import numpy as np

from paths import PROJECT_ROOT
from missile.navigation.kalman_filter import KalmanFilter
from missile.navigation.tercom import TERCOM
from missile.navigation.gps import GPS
from missile.navigation.ins import INS
from missile.navigation.timer import InternalTimer
from missile.state import MissileState
from simulation.sensors.baro_altimeter import BaroAltimeter
from terrain.dem_loader import DEMLoader

class NavigationComputer:
    def __init__(
        self,
        start_gps: tuple[float, float, float],
        dem_name: str,
        gps_freq_hz: int = 5,
        ins_freq_hz: int = 500,
        tercom_freq_hz: int = 1,
        process_noise_std: float = 0.05,
    ):
        """
        Args:
            start_gps: Starting position (lat, lon, alt) — maps to state est_lat, est_lon, est_alt
            dem_name: name of DEM
            gps_freq_hz: frequency of GPS measurements in Hz
            ins_freq_hz: frequency of INS measurements in Hz
            tercom_freq_hz: frequency of TERCOM measurements in Hz
        """

        self.start_gps = start_gps

        # Setup proper project root of DEM
        self.dem_name = dem_name
        tif_path = PROJECT_ROOT / "data" / "dem" / dem_name
        self.dem_loader = DEMLoader(tif_path)

        # Initialise basic missile systems
        self.timer = InternalTimer()

        # Initialise State through _build_initial_state method
        self.state = self._build_initial_state(start_gps)

        # Set the update freq
        self.gps_period = 1.0 / gps_freq_hz
        self.ins_period = 1.0 / ins_freq_hz
        self.tercom_period = 1.0 / tercom_freq_hz

        # Initialise each navigation system and Kalman Filter
        self.gps = GPS()
        self.ins = INS(
            init_pos=[start_gps[0], start_gps[1], start_gps[2]],
            init_vel=[0.0, 0.0, 0.0]
        )
        self.tercom = TERCOM(self.state.est_lat, self.state.est_lon)

        self.KF = KalmanFilter(
            dt = self.ins_period,
            init_position = list(start_gps),
            init_velocity = [0.0, 0.0, 0.0],
            process_noise_std = process_noise_std
        )

        # Setting threshold for stdev of patch height to determine if terrain is rough enough for TERCOM
        self.tercom_roughness_threshold_m = 5.0

    def _build_initial_state(start_gps: tuple[float, float, float]) -> MissileState:
        lat, lon, alt = start_gps
        return MissileState(
            true_lat=lat,
            true_lon=lon,
            true_alt=alt,
            est_lat=lat,
            est_lon=lon,
            est_alt=alt,
            vel_east=0.0,
            vel_north=0.0,
            vel_up=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            time=0.0,
            distance_traveled=0.0,
            distance_to_target=0.0,
            gps_valud=True,
            tercom_active=False,
            ins_calibrated=True
        )
        
    def run_navigation_loop(
        self, 
        acceleration: np.ndarray | list[float], 
        mission_terminated: bool=False, 
        angular_velocity: list[float] | None = None,
        run_seconds: int=10000) -> None:
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

        acceleration = np.asarray(acceleration, dtype=float)

        if angular_velocity is None:
            yaw_rate = 0.0
        else:
            yaw_rate = float(angular_velocity[2])

        # Reset schedule checkpoints to t = 0
        self.next_ins = 0.0
        self.next_gps = self.gps_period # first gps fix one period in
        self.next_tercom = self.tercom_period # first TERCOM fix one period in
        
        # Use our own sim_time instead of InternalTimer for accuracy
        sim_time = 0.0

        while not mission_terminated and sim_time < run_seconds:
            
            # -- INS update: predict (every tick) --
            if sim_time >= self.next_ins:
                # turn m/s into lat/lon/alt change
                self.state.update_physics(
                    self.ins_period,
                    acceleration,
                    yaw_rate
                )

                self.ins.predict(acceleration, self.ins_period, angular_velocity)
                self.KF.predict(acceleration)
                self.state.apply_ins_estimate(self.ins)

                self.next_ins += self.ins_period

            if sim_time >= self.next_gps and not self.gps.is_jammed:
                mea = self.gps.get_gps_location(self.state.true_position())

                if mea is not None:
                    self._apply_gps_fix(mea)
                    self.state.gps_valid = True
                else:
                    self.state.gps_valid = False
            
                self.next_gps += self.gps_period

            if sim_time >= self.next_tercom:
                self._tercom_update()
                self.next_tercom += self.tercom_period

        sim_time += self.ins_period

    # --- KF SYNC ---
    def _sync_kf_to_ins_and_state(self) -> None:
        """Push the processed KF state into INS, then mirror it into MissileState"""
        est_pos, est_vel = self.KF.get_state()
        self.ins.correct_state(est_pos, est_vel)
        self.state.apply_ins_estimate(self.ins)
        
    def _apply_gps_fix(self, measurement) -> None:
        """Fuse a 3D GPS fix [lat, lon, alt], then sync INS and shared state"""
        mea = np.asarray(measurement, dtype=float)
        self.KF.update(mea.tolist(), sensor_type="GPS")
        self._sync_kf_to_ins_and_state()

    def _apply_tercom_fix(self, lat: float, lon: float, alt_msl: float) -> None:
        """Fuse TERCOM's lat/lon coordinate with altitude (MSL) from BaroAltimeter. Turn 2D -> 3D (with alt)"""
        self.KF.update([float(lat), float(lon), float(alt_msl)], sensor_type="TERCOM")
        self._sync_kf_to_ins_and_state()

    # --- TERCOM RELATED ---
    def _tercom_update(self) -> None:
        """Run one TERCOM fix if terrain is suitable"""
        est_lat, est_lon, _ = self.state.est_position()

        patch = self.dem_loader.get_elevation_patch(
            est_lat, est_lon, patch_size=25, nromalized = False,
        )
        
    def _is_terrain_suitable(self,
        terrain_patch: np.ndarray,
        lat: float,
        lon: float,
        patch_size: int=25) -> bool:
        """
        Check for terrain roughness to determine whether the terrain is rough enough to conduct accurate TERCOM.
        TERCOM is highly based on terrain signature, a flat terrain with cause error and inaccuracy.
        Determined by standard deviation.
        """

        patch = terrain_patch

        if patch is None and self.dem_loader is not None and lat is not None and lon is not None:
            patch = self.dem_loader.get_elevation_patch(lat, lon, patch_size, normalized=False)
        if patch is None:
            return False

        values = np.asarray(patch, dtype=float)
        values = values[np.isfinite(values)] # filter inf

        if values.size == 0:
            return False
        return float(np.std(values)) >= self.tercom_roughness_threshold_m







