import numpy as np
from missile.navigation.ins import INS
from terrain.coordinates import CoordinateSystem

class KalmanFilter:
    def __init__(self, dt: float,
                 init_position: list[float],
                 init_velocity: list[float],
                 process_noise_std: float,
                 std_mea: float=0.05
                 ) -> None:
        """
        Set up the Kalman filter with the necessary initial and default matrices.
        """
        # Sampling time
        self.dt = dt
        self._process_noise_std = process_noise_std

        # Measurement error (GPS/TERCOM)
        self.std_mea = std_mea

        # Initial velocity
        if init_velocity is None:
            init_velocity = [0.0, 0.0, 0.0] # 0.0 for x, y, z

        # Initiate geographic origin position
        lat0, lon0, alt0 = map(float, init_position)
        self._origin_lat = lat0
        self._origin_lon = lon0
        self._origin_alt = alt0

        # Initialise CoordinateSystem to handle lat/lon to east/north conversion using WG-84
        self.cs = CoordinateSystem(lat0, lon0)
        east0, north0 = self.cs.latlong_to_enu(lat0, lon0)

        # State vector: [east_m, north_m, alt_m, vx_m/s, vy_m/s, vz_m/s]
        self.x = np.array([
            east0, north0, alt0,
            init_velocity[0], init_velocity[1], init_velocity[2]
        ], dtype=float)

        ref_lat = float(init_position[0])
        self.A = INS.get_transition_matrix(dt, reference_lat=ref_lat)
        self.B = INS.get_control_matrix(dt, reference_lat=ref_lat)

        # Observation matrix (H) - transformation matrix
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1  # observe lat
        self.H[1, 1] = 1  # observe lon
        self.H[2, 2] = 1  # observe alt

        # Process noise covariance matrix (Q)
        # internal uncertainty: how weather / physic disturb the missile
        self.Q = (self.B @ self.B.T) * (process_noise_std ** 2)

        # Sensor noise covariance matrix (R)
        # Scenario 1: GPS (+/- 1m with WAGE enhancement, vertical is usually 1.5x worse.)
        self.R_GPS = np.diag([1.0 ** 2, 1.0 ** 2, 3.0 ** 2])

        # Scenario 2: TERCOM (12m deviation)
        # lateral Accuracy: +/- 10-15m (Grid dependent)
        # vertical Accuracy: Radar Altimeter is very precise (+/- 1m), according to the vegetation and landscape.
        self.R_TERCOM = np.diag([13.0 ** 2, 8.0 ** 2, 1.0 ** 2])

        # Process covariance matrix, initial estimation of uncertainty (~50m initial error)
        pos_std_m = 50.0
        vel_std = 5.0
        self.P = np.diag([
            pos_std_m ** 2, pos_std_m ** 2, pos_std_m ** 2,
            vel_std ** 2, vel_std ** 2, vel_std ** 2
        ])

    @staticmethod
    def _build_control_matrix(dt: float) -> np.ndarray:
        """
        B matrix maps acceleration input to state change.
        Input u = (ax east, ay north, az up) in m/s^2
        Use standard kinematic model (as per definition in B matrix):
            position += 0.5 * a * dt^2
            velocity += a * dt
        A 3D B control matrix looks like this
        B =
            [ 0.5Δ^2,       0,       0],
            [      0, 0.5Δt^2,       0],
            [      0,       0, 0.5Δt^2],
            [     Δt,       0,       0],
            [      0,      Δt,       0],
            [      0,       0,      Δt]
        """
        b = np.zeros((6, 3))
        b[0, 0] = 0.5 * dt ** 2
        b[1, 1] = 0.5 * dt ** 2
        b[2, 2] = 0.5 * dt ** 2
        b[3, 0] = dt
        b[4, 1] = dt
        b[5, 2] = dt
        return b

    @staticmethod
    def _build_transition_matrix(dt: float) -> np.ndarray:
        """
        Build state transition matrix A, for a constant-velocity kinematic model.
            east += vx * dt
            north += vy * dt
        velocity unchanged (acceleration handled in B)

        A 3d A transition matrix looks like this
        A =
            [1, 0, 0, Δt,  0,  0],
            [0, 1, 0,  0, Δt,  0],
            [0, 0, 1,  0,  0, Δt]
        """
        a = np.eye(6)
        a[0, 3] = dt # east pos
        a[1, 4] = dt # north pos
        a[2, 5] = dt # altitude
        return a

    def _geo_meas_to_enu(self,
                         meas_lat: float,
                         meas_lon: float,
                         meas_alt_msl: float
                         ) -> np.ndarray:
        """
        Convert GPS deg to ENU position in meter. Altitude is ignored here,
        focusing on the horizontal fix.

        Args:
            meas_lat, meas_lon: degrees of measured GPS position
            meas_alt_msl: meters of measured altitude (no changes here)

        Returns:
            [east_m, north_m, alt_m] relative to launch origin (ENU position)
        """
        east_m, north_m = self.cs.latlong_to_enu(float(meas_lat), float(meas_lon))
        return np.array([east_m, north_m, meas_alt_msl], dtype=float)

    def _enu_pos_to_geo (self,
                         east_m: float,
                         north_m: float,
                         alt_msl_m: float
                         ) -> np.ndarray:
        """
        Convert ENU position in meter to geo position in degree.

        Args:
            east_m, north_m: meters of ENU position
            alt_msl_m: meters of altitude (no changes here, too)

        Return:
            [est_lat, est_lon, est_alt_msl], first two in degrees
        """
        est_lat, est_lon = self.cs.enu_to_latlong(east_m, north_m)
        return np.array([est_lat, est_lon, alt_msl_m], dtype=float)

    def predict(self, acc_vec_input: list[float]) -> None:
        """
        Generate a prediction of the next location of the missile, based on speed, acceleration, current position...
        The prediction will be implemented using ins.py, so INS will handle prediction instead of Kalman Filter itself.
        Arg:
            acc_vec_input: [ax east, ay north, az up] in m/s^2
        """
        u = np.array(acc_vec_input)
        ref_lat = float(self.x[0])
        self.A = self._build_transition_matrix(self.dt)
        self.B = self._build_control_matrix(self.dt)
        self.Q = (self.B @ self.B.T) * (self._process_noise_std ** 2)

        # Predictive state x = Ax + Bu:
        self.x = (self.A @ self.x) + (self.B @ u)
        
        # Predicted process covariance matrix P = AP * A.T + Q: 
        self.P = (self.A @ self.P @ self.A.T) + self.Q

    def update(self, measurement: list[float], sensor_type: str="GPS") -> None:
        """
        Update the position of the missile, combining measurement and prediction result, use Kalman Gain to determine
        which to trust more.

        Args:
            measurement: list[meas_lat, meas_lon, meas_alt_msl]
            sensor_type: default "GPS" or "TERCOM"
        """
        # Set the R matrix (measurement error) to different value based on sensor_type
        if sensor_type == "TERCOM":
            R_current = self.R_TERCOM
        else:
            R_current = self.R_GPS

        # Handle Measurement
        y = self._geo_meas_to_enu(*measurement)

        # Error = measurement - expected position
        error = y - (self.H @ self.x)

        # Kalman gain (KG)
        KG = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T) + R_current) # use np.linalg.inv() to sorta achieve division (x inverse)

        # Update state (x)
        self.x = self.x + (KG @ error)

        # Update process covariance (P)
        I = np.eye(6)
        self.P = (I - (KG @ self.H)) @ self.P

    def get_state(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Return the current best-estimated position and velocity of missile, computed by kalman filter.

        Return: two slices of an array with
            pos: [lat deg, lon deg, alt m MSL]
            vel: [vx east, vy north, vz up] m/s
        """
        pos_geo = self._enu_pos_to_geo(self.x[0], self.x[1], self.x[2])
        return pos_geo, self.x[3:].copy()