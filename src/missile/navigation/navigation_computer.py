import numpy as np

from paths import PROJECT_ROOT
from missile.navigation.kalman_filter import KalmanFilter
from missile.navigation.tercom import TERCOM
from missile.navigation.gps import GPS
from missile.navigation.ins import INS
from missile.state import MissileState
from simulation.sensors.baro_altimeter import BaroAltimeter
from simulation.sensors.imu import IMU
from terrain.dem_loader import DEMLoader

class NavigationComputer:
    def __init__(
        self,
        true_start_gps: tuple[float, float, float],
        dem_name: str,
        gps_freq_hz: int = 5,
        ins_freq_hz: int = 500,
        tercom_freq_hz: int = 1,
        process_noise_std: float = 0.05,
    ):
        """
        Args:
            true_start_gps: True starting position (lat, lon, alt) at launch (ground
                truth) — seeds both the true state and the initial estimate
            dem_name: name of DEM
            gps_freq_hz: frequency of GPS measurements in Hz
            ins_freq_hz: frequency of INS measurements in Hz
            tercom_freq_hz: frequency of TERCOM measurements in Hz
            process_noise_std: standard deviation of process noise for INS and KF
        """

        self.start_gps = true_start_gps

        # Set up proper project root of DEM
        self.dem_name = dem_name
        tif_path = PROJECT_ROOT / "data" / "dem" / dem_name
        self.dem_loader = DEMLoader(tif_path)

        # Initialise baro altimeter for msl height
        self.baro_alt = BaroAltimeter()

        # Set the update freq
        self.gps_period = 1.0 / gps_freq_hz
        self.ins_period = 1.0 / ins_freq_hz
        self.tercom_period = 1.0 / tercom_freq_hz

        # Initialise each navigation system and Kalman Filter
        self.gps = GPS()
        # Shared imperfect IMU: its one noisy reading feeds BOTH the INS and the KF
        self.imu = IMU.tactical_grade()
        self.ins = INS(
            init_pos=[true_start_gps[0], true_start_gps[1], true_start_gps[2]],
            init_vel=[0.0, 0.0, 0.0]
        )

        est_location = [self.state.est_lat, self.state.est_lon]
        self.tercom = TERCOM(est_location, dem_name)

        self.KF = KalmanFilter(
            dt = self.ins_period,
            init_position = list(true_start_gps),
            init_velocity = [0.0, 0.0, 0.0],
            process_noise_std = process_noise_std
        )

        # Setting threshold for stdev of patch height to determine if terrain is rough enough for TERCOM
        self.tercom_roughness_threshold_m = 5.0

        self.next_gps = self.gps_period # first GPS fix one period in
        self.next_tercom = self.tercom_period # first TERCOM fix one period in

    def step(self, imu: IMU, state: MissileState, sim_time: float, dt: float) -> None:

        # INS + KF predicts
        acc_meas, gyro_meas = self.imu.imu_error(
            # true accel_enu and angular_velocity from MissileDynamics
            np.asarray(imu.accel_enu, dtype=float),
            np.asarray(imu.angular_velocity, dtype=float),
            dt
        )
        self.ins.predict(acc_meas, dt, gyro_meas)
        self.KF.predict(acc_meas)

        # between fix from GPS/TERCOM, we rekon ins result as true location and apply it to state
        ins_pos, _, _ = self.ins.get_state()
        state.apply_ins_estimate(ins_pos)

        # GPS fix
        if sim_time >= self.next_gps and not self.gps.is_jammed:
            mea = self.gps.get_gps_location(self.state.true_position())

            if mea is not None:
                self._apply_gps_fix(mea, state)
                self.state.gps_valid = True
            else:
                self.state.gps_valid = False

            self.next_gps += self.gps_period

        # TERCOM fix
        if sim_time >= self.next_tercom:
            self._tercom_update()
            self.next_tercom += self.tercom_period


    # def run_navigation_loop(
    #     self,
    #     true_acceleration: np.ndarray | list[float],
    #     mission_terminated: bool=False,
    #     true_angular_velocity: list[float] | None = None,
    #     run_seconds: int=10000
    # ) -> None:
    #     """
    #     Run the navigation loop for a fixed amount of elapsed time.
    #
    #     Args:
    #         run_seconds: Total runtime duration, in seconds, measured from the
    #             start of the loop (sim_time = 0). So this combines with
    #             condition checks, prevents the loop to endlessly run forever.
    #             We set it to 1,0000-second default, so it will stop in 1,0000 seconds,
    #             which is 2.78 hrs.
    #         mission_terminated: If True, the navigation loop will terminate
    #     """
    #
    #     true_acceleration = np.asarray(true_acceleration, dtype=float)
    #
    #     if true_angular_velocity is None:
    #         yaw_rate = 0.0
    #     else:
    #         yaw_rate = float(true_angular_velocity[2])
    #
    #     # Reset schedule checkpoints to t = 0
    #     self.next_ins = 0.0
    #     self.next_gps = self.gps_period # first gps fix one period in
    #     self.next_tercom = self.tercom_period # first TERCOM fix one period in
    #
    #     # Use our own sim_time instead of InternalTimer for accuracy
    #     sim_time = 0.0
    #
    #     while not mission_terminated and sim_time < run_seconds:
    #
    #         # -- INS update: predict (every tick) --
    #         if sim_time >= self.next_ins:
    #             # turn m/s into lat/lon/alt change
    #             self.state.update_physics(
    #                 self.ins_period,
    #                 true_acceleration,
    #                 yaw_rate
    #             )
    #             # IMU turns the true motion into noisy measurement
    #             true_ang_vel = (np.zeros(3) if true_angular_velocity is None # if somehow true_ang_vel is None
    #                             else np.asarray(true_angular_velocity, dtype=float))
    #             acc_meas, gyro_meas = self.imu.imu_error(
    #                 true_acceleration, true_ang_vel, self.ins_period
    #             )
    #
    #             # corrupted measurements are fed into ins and kf
    #             self.ins.predict(acc_meas, self.ins_period, gyro_meas)
    #             self.KF.predict(acc_meas)
    #             self.state.apply_ins_estimate(self.ins)
    #
    #             self.next_ins += self.ins_period
    #
    #         if sim_time >= self.next_gps and not self.gps.is_jammed:
    #             mea = self.gps.get_gps_location(self.state.true_position())
    #
    #             if mea is not None:
    #                 self._apply_gps_fix(mea)
    #                 self.state.gps_valid = True
    #             else:
    #                 self.state.gps_valid = False
    #
    #             self.next_gps += self.gps_period
    #
    #         if sim_time >= self.next_tercom:
    #             self._tercom_update()
    #             self.next_tercom += self.tercom_period
    #
    #         sim_time += self.ins_period

    # --- KF SYNC ---
    def _sync_kf_to_ins_and_state(self, state: MissileState) -> None:
        """Push the processed KF state into INS, then mirror it into MissileState."""
        est_pos, est_vel = self.KF.get_state()
        self.ins.correct_state(est_pos, est_vel)
        ins_pos, _, _ = self.ins.get_state() # apply ins correction to state when we receive data from GPS/TERCOM
        state.apply_ins_estimate(self.ins)
        
    def _apply_gps_fix(self, gps_measurement, state: MissileState) -> None:
        """Fuse a 3D GPS fix [lat, lon, alt], then sync INS and shared state."""
        mea = np.asarray(gps_measurement, dtype=float)
        self.KF.update(mea.tolist(), sensor_type="GPS")
        self._sync_kf_to_ins_and_state(state)

    def _apply_tercom_fix(
            self,
            matched_lat: float,
            matched_lon: float,
            baro_alt_msl: float,
            state: MissileState
    ) -> None:
        """Fuse TERCOM's lat/lon coordinate with altitude (MSL) from BarAltimeter. Turn 2D -> 3D (with alt)"""
        self.KF.update([float(matched_lat), float(matched_lon), float(baro_alt_msl)], sensor_type="TERCOM")
        self._sync_kf_to_ins_and_state(state)

    # --- TERCOM RELATED ---
    def _tercom_update(self) -> None:
        """Run one TERCOM fix if terrain is suitable"""

        est_lat, est_lon, _ = self.state.est_position()

        patch = self.dem_loader.get_elevation_patch(
            est_lat, est_lon, patch_size=25, normalized=False
        )

        if not self._is_terrain_suitable(patch, est_lat, est_lon):
            self.state.tercom_active = False
            return

        true_lat, true_lon, _ = self.state.true_position()
        sensed_patch = self.dem_loader.get_elevation_patch(true_lat, true_lon, patch_size=7, normalized=True)

        if sensed_patch is None:
            self.state.tercom_active = False
            return

        matched_lat, matched_lon, _ = self.tercom.process_update(
            sensed_patch, est_lat, est_lon
        )

        self.state.tercom_active = matched_lat is not None

        if matched_lat is not None:
            self._apply_tercom_fix(matched_lat, matched_lon, self.baro_alt.get_baro_msl(self.state.true_alt)) # msl obtain from baro altimeter
        
    def _is_terrain_suitable(
            self,
            terrain_patch: np.ndarray,
            est_lat: float,
            est_lon: float,
            patch_size: int=25
    ) -> bool:
        """
        Check for terrain roughness to determine whether the terrain is rough enough to conduct accurate TERCOM.
        TERCOM is highly based on terrain signature, a flat terrain with cause error and inaccuracy.
        Determined by standard deviation.

        Return:
            True if terrain is rough enough, False otherwise.
        """

        patch = terrain_patch

        if patch is None and self.dem_loader is not None and est_lat is not None and est_lon is not None:
            patch = self.dem_loader.get_elevation_patch(est_lat, est_lon, patch_size, normalized=False)
        if patch is None:
            return False

        values = np.asarray(patch, dtype=float)
        values = values[np.isfinite(values)] # filter inf

        if values.size == 0:
            return False
        return float(np.std(values)) >= self.tercom_roughness_threshold_m