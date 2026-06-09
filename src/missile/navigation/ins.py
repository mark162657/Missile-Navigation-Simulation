import math
import numpy as np


class INS:
    """
    Inertial Navigation System (strapdown simulation model).

    The INS dead-reckons the missile's basic inertial state -- position,
    velocity, and attitude -- by integrating IMU measurements (specific force
    from the accelerometers, angular rate from the gyroscopes).

    A real IMU is imperfect, so the ideal inputs are corrupted by:
      - turn-on bias       : constant offset fixed at power-up (accel + gyro)
      - in-run bias walk   : slow random walk on the bias -> long-term drift
      - white noise        : zero-mean Gaussian noise each step

    These errors make the dead-reckoned estimate drift away from truth over
    time. That drift is precisely the error the Kalman Filter corrects using
    GPS / TERCOM fixes. With all error terms at their default (0), the INS
    behaves as a clean, deterministic dead-reckoner.

    Frame (unified project convention):
      - position : [lat, lon, alt] — degrees, degrees, meters MSL (maps to MissileState est_* / INS pos[])
      - velocity : [vx, vy, vz] = [east, north, up] in m/s
      - attitude : [roll, pitch, yaw] in radians

    The two @staticmethods (get_transition_matrix / get_control_matrix) expose
    the linear motion model (A, B) consumed by the Kalman Filter in the same
    geographic frame (lat/lon rows scaled by meters-per-degree at reference_lat).
    """

    def __init__(
        self,
        init_pos: np.ndarray | list[float],
        init_vel: np.ndarray | list[float],
        init_att: np.ndarray | list[float] | None = None,
        accel_bias: np.ndarray | list[float] | None = None,
        gyro_bias: np.ndarray | list[float] | None = None,
        accel_noise_std: float = 0.0,
        gyro_noise_std: float = 0.0,
        accel_bias_walk_std: float = 0.0,
        gyro_bias_walk_std: float = 0.0,
        rng: np.random.Generator | None = None) -> None:
        """
        Args:
            init_pos: initial position [lat, lon, alt] — x, y in degrees, z in meters
            init_vel: initial velocity [vx east, vy north, vz up] in m/s
            init_att: initial attitude [roll, pitch, yaw] in radians
            accel_bias: constant accelerometer turn-on bias (m/s^2), per axis
            gyro_bias: constant gyroscope turn-on bias / drift (rad/s), per axis
            accel_noise_std: std of white accelerometer noise (m/s^2)
            gyro_noise_std: std of white gyroscope noise (rad/s)
            accel_bias_walk_std: std of accel bias random-walk increment (m/s^2/sqrt(s))
            gyro_bias_walk_std: std of gyro bias random-walk increment (rad/s/sqrt(s))
            rng: optional numpy Generator for reproducible error sampling
        """
        # --- core inertial state ---
        self.pos = np.asarray(init_pos, dtype=float).copy()
        self.vel = np.asarray(init_vel, dtype=float).copy()

        if init_att is None:
            init_att = [0.0, 0.0, 0.0]
        self.att = np.asarray(init_att, dtype=float).copy()
        self._normalize_attitude()

        # --- IMU error model ---
        self._rng = rng if rng is not None else np.random.default_rng()

        self.accel_bias = (np.zeros(3) if accel_bias is None
                           else np.asarray(accel_bias, dtype=float).copy())
        self.gyro_bias = (np.zeros(3) if gyro_bias is None
                          else np.asarray(gyro_bias, dtype=float).copy())

        self.accel_noise_std = float(accel_noise_std)
        self.gyro_noise_std = float(gyro_noise_std)
        self.accel_bias_walk_std = float(accel_bias_walk_std)
        self.gyro_bias_walk_std = float(gyro_bias_walk_std)

        # --- bookkeeping ---
        self.time = 0.0
        self.distance_traveled = 0.0

    @classmethod
    def tactical_grade(
        cls,
        init_pos: np.ndarray | list[float],
        init_vel: np.ndarray | list[float],
        init_att: np.ndarray | list[float] | None = None,
        rng: np.random.Generator | None = None) -> "INS":
        """
        Build an INS preconfigured with tactical-grade IMU error terms.

        Turn-on biases are sampled once from the given (or a fresh) RNG, so each
        constructed unit drifts differently. Values are representative, not
        sourced from a specific datasheet -- tune as real specs become available.
        """
        rng = rng if rng is not None else np.random.default_rng()

        accel_bias = rng.normal(0.0, 0.02, size=3)                      # ~2e-2 m/s^2 (~2 mg)
        gyro_bias = rng.normal(0.0, math.radians(5.0) / 3600.0, size=3)  # ~5 deg/hr

        return cls(
            init_pos,
            init_vel,
            init_att,
            accel_bias=accel_bias,
            gyro_bias=gyro_bias,
            accel_noise_std=0.05,                       # m/s^2
            gyro_noise_std=math.radians(0.1),           # rad/s
            accel_bias_walk_std=1e-3,                   # m/s^2 / sqrt(s)
            gyro_bias_walk_std=math.radians(0.01) / 60.0,  # rad/s / sqrt(s)
            rng=rng,
        )

    def _normalize_attitude(self) -> None:
        self.att = np.array([angle % (2 * math.pi) for angle in self.att], dtype=float)

    def _corrupt_imu(
        self,
        acc_true: np.ndarray,
        ang_true: np.ndarray,
        dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Apply the IMU error model to ideal inputs: advance the in-run bias
        random walk, then add bias and white noise. Returns the corrupted
        (acceleration, angular_velocity) the INS will actually integrate.
        """
        # In-run bias instability: random walk scaled by sqrt(dt).
        if self.accel_bias_walk_std > 0.0:
            self.accel_bias += self._rng.normal(0.0, self.accel_bias_walk_std, size=3) * math.sqrt(dt)
        if self.gyro_bias_walk_std > 0.0:
            self.gyro_bias += self._rng.normal(0.0, self.gyro_bias_walk_std, size=3) * math.sqrt(dt)

        acc = acc_true + self.accel_bias
        ang = ang_true + self.gyro_bias

        if self.accel_noise_std > 0.0:
            acc = acc + self._rng.normal(0.0, self.accel_noise_std, size=3)
        if self.gyro_noise_std > 0.0:
            ang = ang + self._rng.normal(0.0, self.gyro_noise_std, size=3)

        return acc, ang

    def predict(
        self,
        acceleration: np.ndarray | list[float],
        dt: float,
        angular_velocity: np.ndarray | list[float] | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Propagate the inertial state by one timestep using (noisy) IMU inputs.

        Args:
            acceleration: specific force [ax east, ay north, az up], m/s^2
            dt: timestep in seconds
            angular_velocity: measured body rates [roll_rate, pitch_rate, yaw_rate], rad/s

        Returns:
            (pos, vel, att) after propagation.
        """
        acc_true = np.asarray(acceleration, dtype=float)

        if angular_velocity is None:
            angular_velocity = [0.0, 0.0, 0.0]
        ang_true = np.asarray(angular_velocity, dtype=float)

        acc, ang_vel = self._corrupt_imu(acc_true, ang_true, dt)

        # Strapdown integration in geographic frame (constant-acceleration over dt).
        previous_velocity = self.vel.copy()
        m_lat = 111_320.0
        m_lon = 111_320.0 * math.cos(math.radians(self.pos[0]))

        self.pos[0] += (previous_velocity[1] * dt + 0.5 * acc[1] * dt ** 2) / m_lat
        self.pos[1] += (previous_velocity[0] * dt + 0.5 * acc[0] * dt ** 2) / m_lon
        self.pos[2] += previous_velocity[2] * dt + 0.5 * acc[2] * dt ** 2
        self.vel += acc * dt
        self.att += ang_vel * dt
        self._normalize_attitude()

        self.time += dt
        self.distance_traveled += float(np.linalg.norm(self.vel) * dt)

        return self.get_state()

    def correct_state(
        self,
        corrected_pos: np.ndarray | list[float],
        corrected_vel: np.ndarray | list[float],
        corrected_att: np.ndarray | list[float] | None = None) -> None:
        """
        Replace the INS estimate with an externally corrected state (e.g. fed
        back from the Kalman Filter after a GPS/TERCOM fix).
        """
        self.pos = np.asarray(corrected_pos, dtype=float).copy()
        self.vel = np.asarray(corrected_vel, dtype=float).copy()

        if corrected_att is not None:
            self.att = np.asarray(corrected_att, dtype=float).copy()
            self._normalize_attitude()

    def get_speed(self) -> float:
        """Return the current inertial speed (magnitude of velocity), m/s."""
        return float(np.linalg.norm(self.vel))

    def get_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return copies of (position, velocity, attitude)."""
        return self.pos.copy(), self.vel.copy(), self.att.copy()

    def get_state_vector(self) -> np.ndarray:
        """
        Return the Kalman-friendly 6D state vector [lat, lon, alt, vx, vy, vz].
        """
        return np.array(
            [self.pos[0], self.pos[1], self.pos[2], self.vel[0], self.vel[1], self.vel[2]],
            dtype=float,
        )

    def set_state_vector(self, state_vector: np.ndarray | list[float]) -> None:
        """
        Load position and velocity from a 6D Kalman state vector.
        """
        state = np.asarray(state_vector, dtype=float)
        self.pos = state[:3].copy()
        self.vel = state[3:6].copy()

    @staticmethod
    def get_transition_matrix(dt: float, reference_lat: float = 0.0) -> np.ndarray:
        """
        State transition matrix A for [lat, lon, alt, vx, vy, vz].

        Position rows couple north velocity to latitude and east velocity to
        longitude via meters-per-degree at reference_lat.
        """
        m_lat = 111_320.0
        m_lon = 111_320.0 * math.cos(math.radians(reference_lat))

        A = np.eye(6)
        A[0, 4] = dt / m_lat   # lat <- vy (north)
        A[1, 3] = dt / m_lon   # lon <- vx (east)
        A[2, 5] = dt           # alt <- vz (up)
        return A

    @staticmethod
    def get_control_matrix(dt: float, reference_lat: float = 0.0) -> np.ndarray:
        """
        Control matrix B for acceleration input [ax east, ay north, az up].
        """
        m_lat = 111_320.0
        m_lon = 111_320.0 * math.cos(math.radians(reference_lat))

        B = np.zeros((6, 3))
        B[0, 1] = 0.5 * dt ** 2 / m_lat
        B[1, 0] = 0.5 * dt ** 2 / m_lon
        B[2, 2] = 0.5 * dt ** 2
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt
        return B
