import numpy as np
from missile.navigation.ins import INS

class KalmanFilter:
    def __init__(self, dt: float, init_position: list[float], init_velocity: list[float], process_noise_std: float,
                 std_mea: float=0.05) -> None:
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

        # State vector: [lat, lon, alt, vx east, vy north, vz up]
        self.x = np.array([
            init_position[0], init_position[1], init_position[2],
            init_velocity[0], init_velocity[1], init_velocity[2]
        ])

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

        # Process covariance matrix (~50m initial error)
        self.P = np.eye(6) * 100
    
    def predict(self, acc_vec_input: list[float]) -> None:
        """
        Generate a prediction of the next location of the missile, based on speed, acceleration, current position...
        The prediction will be implemented using ins.py, so INS will handle prediction instead of Kalman Filter itself.
        Arg:
            acc_vec_input: [ax east, ay north, az up] in m/s^2
        """
        u = np.array(acc_vec_input)
        ref_lat = float(self.x[0])
        self.A = INS.get_transition_matrix(self.dt, reference_lat=ref_lat)
        self.B = INS.get_control_matrix(self.dt, reference_lat=ref_lat)
        self.Q = (self.B @ self.B.T) * (getattr(self, "_process_noise_std", 0.05) ** 2)

        # Predictive state x = Ax + Bu:
        self.x = (self.A @ self.x) + (self.B @ u)
        
        # Predicted process covariance matrix P = AP * A.T + Q: 
        self.P = (self.A @ self.P @ self.A.T) + self.Q

    def update(self, measurement: list[float], sensor_type: str="GPS") -> None:
        """
        Update the position of the missile, combining measurement and prediction result, use Kalman Gain to determine
        which to trust more.

        Args:
            measurement: list[lat, lon, alt]
        """
        # Set the R matrix (measurement error) to different value based on sensor_type
        if sensor_type == "TERCOM":
            R_current = self.R_TERCOM
        else:
            R_current = self.R_GPS

        # Handle Measurement
        y = np.array(measurement)

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

        Return: two slices of array with
            - slice 1: [lat, lon, alt]
            - slice 2: [vx east, vy north, vz up]
        """
        return self.x[:3], self.x[3:]