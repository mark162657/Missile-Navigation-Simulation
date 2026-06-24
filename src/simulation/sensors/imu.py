import numpy as np

class IMU:
    """
    Dedicated IMU sensor to handle error and deviation of INS over time.
    """
    def __init__(
        self,
        accel_bias: np.ndarray | list[float] | None = None,
        gyro_bias: np.ndarray | list[float] | None = None,
        accel_noise_std: float = 0.0,
        gyro_noise_std: float = 0.0,
        accel_bias_walk_std: float = 0.0,
        gyro_bias_walk_std: float = 0.0
    ):
        """

        """


    def imu_error(
        self,
        acc_true: np.ndarray,
        ang_true: np.ndarray,
        dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Apply the IMU error model to ideal inputs: advance the in-run bias
        random walk, then add bias and white noise. Returns the corrupted
        (acceleration, angular_velocity) the INS will actually integrate.
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