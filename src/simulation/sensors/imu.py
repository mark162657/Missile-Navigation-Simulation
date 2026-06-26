import numpy as np
import math

class IMU:
    """
    Dedicated IMU sensor to handle error and deviation of INS over time.
    Separating it from INS makes the design make sense and allows KF to read a full corrupted IMU measured.
    """
    def __init__(
        self,
        accel_bias: np.ndarray | list[float] | None = None,
        gyro_bias: np.ndarray | list[float] | None = None,
        accel_noise_std: float = 0.0,
        gyro_noise_std: float = 0.0,
        accel_bias_walk_std: float = 0.0,
        gyro_bias_walk_std: float = 0.0,
        rng: np.random.Generator | None = None
    ):
        """
        Args:
            accel_bias: constant accelerometer turn-on bias (m/s^2), per axis
            gyro_bias: constant gyroscope turn-on bias / drift (rad/s), per axis
            accel_noise_std: std of white accelerometer noise (m/s^2)
            gyro_noise_std: std of white gyroscope noise (rad/s)
            accel_bias_walk_std: std of accel bias random-walk increment (m/s^2/sqrt(s))
            gyro_bias_walk_std: std of gyro bias random-walk increment (rad/s/sqrt(s))
            rng: optional numpy Generator for reproducible error sampling
        """
        self._rng = rng if rng is not None else np.random.default_rng()

        self.accel_bias = (np.zeros(3) if accel_bias is None
                           else np.asarray(accel_bias, dtype=float).copy())
        self.gyro_bias = (np.zeros(3) if gyro_bias is None
                          else np.asarray(gyro_bias, dtype=float).copy())

        self.accel_noise_std = float(accel_noise_std)
        self.gyro_noise_std = float(gyro_noise_std)
        self.accel_bias_walk_std = float(accel_bias_walk_std)
        self.gyro_bias_walk_std = float(gyro_bias_walk_std)


    def imu_error(
            self,
            acc_true: np.ndarray,
            ang_true: np.ndarray,
            dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Apply the IMU error model to ideal inputs: advance the in-run bias
        random walk, then add bias and white noise. Returns the corrupted
        (acceleration, angular_velocity) the INS will actually integrate.

        Args:
            acc_true: true and uncorrupted accelerometer reading
            ang_true: true and uncorrupted gyroscope reading
            dt: timestep in seconds
        """

        # In-run bias instability: random walk scaled by sqrt(dt) / bias drift
        if self.accel_bias_walk_std > 0.0:
            self.accel_bias += self._rng.normal(0.0, self.accel_bias_walk_std, size=3) * math.sqrt(dt)
        if self.gyro_bias_walk_std > 0.0:
            self.gyro_bias += self._rng.normal(0.0, self.gyro_bias_walk_std, size=3) * math.sqrt(dt)

        acc = acc_true + self.accel_bias
        ang = ang_true + self.gyro_bias

        # Measurement noise
        if self.accel_noise_std > 0.0:
            acc = acc + self._rng.normal(0.0, self.accel_noise_std, size=3)
        if self.gyro_noise_std > 0.0:
            ang = ang + self._rng.normal(0.0, self.gyro_noise_std, size=3)

        return acc, ang

    @classmethod
    def tactical_grade(
            cls,
            rng: np.random.Generator | None = None) -> "INS":
        """
        Build an INS preconfigured with tactical-grade IMU error terms.

        Turn-on biases are sampled once from the given (or a fresh) RNG, so each
        constructed unit drifts differently. Values are representative, not
        sourced from a specific datasheet -- tune as real specs become available.
        """
        rng = rng if rng is not None else np.random.default_rng()

        accel_bias = rng.normal(0.0, 0.02, size=3)  # ~2e-2 m/s^2 (~2 mg)
        gyro_bias = rng.normal(0.0, math.radians(5.0) / 3600.0, size=3)  # ~5 deg/hr

        return cls(
            accel_bias=accel_bias,
            gyro_bias=gyro_bias,
            accel_noise_std=0.05,  # m/s^2
            gyro_noise_std=math.radians(0.1),  # rad/s
            accel_bias_walk_std=1e-3,  # m/s^2 / sqrt(s)
            gyro_bias_walk_std=math.radians(0.01) / 60.0,  # rad/s / sqrt(s)
            rng=rng
        )
