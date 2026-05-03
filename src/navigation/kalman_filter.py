import numpy as np
from navigation.ins import INS

class KalmanFilter:
    def __init__(self, dt: float, init_position: list[float], init_velocity: list[float], process_noise_std: float,
                 std_mea: float=0.05) -> None:
        """
        Set up the Kalman filter with the necessary initial and default matrices.
        """
        # Sampling time
        self.dt = dt

        # Measurement error (GPS/TERCOM)
        self.std_mea = std_mea

        # Initial velocity
        if init_velocity is None:
            init_velocity = [0.0, 0.0, 0.0] # 0.0 for x, y, z

        # State matrix (X)
        self.x = np.array([
            init_position[0], init_position[1], init_position[2],
            init_velocity[0], init_velocity[1], init_velocity[2]
        ])

        # Initialise INS
        ins = INS(init_pos=init_position, init_vel=init_velocity)

        # Replace A with what we have in ins.py
        self.A = ins.get_transition_matrix(dt)

        # Replace B with what we have in ins.py
        self.B = ins.get_control_matrix(dt)

        # Observation matrix (H) - transformation matrix
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1  # observe x
        self.H[1, 1] = 1  # observe y
        self.H[2, 2] = 1  # observe z

        # Process noise covariance matrix (Q)
        # internal uncertainty: how weather / physic disturb the missile
        self.Q = (self.B @ self.B.T) * (process_noise_std ** 2)

        # Sensor noise covariance matrix (R)
        # Scenario 1: GPS (+/- 1m with WAGE enhancement, vertical is usually 1.5x worse.)
        self.R_GPS = np.diag([1 ** 2, 1.5 ** 2, 1 ** 2])

        # Scenario 2: TERCOM (12m deviation)
        # lateral Accuracy: +/- 10-15m (Grid dependent)
        # vertical Accuracy: Radar Altimeter is very precise (+/- 1m), according to the vegetation and landscape.
        self.R_TERCOM = np.diag([13.0 ** 2, 8.0 ** 2, 5.0 ** 2])

        # Process covariance matrix (~50m initial error)
        self.P = np.eye(6) * 100
    
    def predict(self, acc_input: list[float]) -> None:
        """
        Generate a prediction of the next location of the missile, based on speed, acceleration, current position...
        The prediction will be implemented using ins.py, so INS will handle prediction instead of Kalman Filter itself.
        Arg:
            acc_input: list[x, y, z]
        """
        u = np.array(acc_input)

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
        KG = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T) + R_current) # use np.linalg.inv() to sorta acheive division (x inverse)

        # Update state (x)
        self.x = self.x + (KG @ error)

        # Update process covariance (P)
        I = np.eye(6)
        self.P = (I - (KG @ self.H)) @ self.P

    def get_state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.x[:3], self.x[3:]