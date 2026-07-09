import math
import numpy as np

from terrain import coordinates

class INS:
    """
    Inertial Navigation System (nav-frame kinematic dead-reckoner).

    The INS dead-reckons the missile's basic inertial state -- position,
    velocity, and attitude -- by integrating IMU inputs: nav-frame (ENU)
    kinematic acceleration and angular rate from the gyroscopes.

    Frame (unified project convention):
      - position : [lat, lon, alt] — degrees, degrees, meters MSL (maps to MissileState est_* / INS pos[])
      - velocity : [vx, vy, vz] = [east, north, up] in m/s
      - attitude : [roll, pitch, yaw] in radians
    """

    def __init__(
            self,
            init_pos: np.ndarray | list[float],
            init_vel: np.ndarray | list[float],
            init_att: np.ndarray | list[float] | None = None,
        ):
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

        # --- bookkeeping ---
        self.time = 0.0
        self.distance_traveled = 0.0

    def _normalize_attitude(self) -> None:
        self.att = np.array([angle % (2 * math.pi) for angle in self.att], dtype=float)

    def predict(
        self,
        acceleration: np.ndarray | list[float],
        dt: float,
        angular_velocity: np.ndarray | list[float] | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Propagate the inertial state by one timestep using (noisy) IMU inputs.

        Args:
            acceleration: nav-frame kinematic acceleration [ax east, ay north, az up], m/s^2
            dt: timestep in seconds
            angular_velocity: measured body rates [roll_rate, pitch_rate, yaw_rate], rad/s

        Returns:
            (pos, vel, att) after propagation.
        """
        acc = np.asarray(acceleration, dtype=float)

        if angular_velocity is None:
            angular_velocity = [0.0, 0.0, 0.0]
        ang_vel = np.asarray(angular_velocity, dtype=float)

        # Strapdown integration in geographic frame (constant-acceleration over dt).
        previous_velocity = self.vel.copy()

        # Guard against a diverged solution: the latitude must stay physical.
        # meter_per_deg_lon_at(lat) carries a cos(lat) factor that -> 0 at the
        # poles, so an out-of-range latitude would turn the longitude update into
        # inf/NaN and only surface later as a cryptic error (e.g. a Haversine
        # "math domain error"). Fail loudly here, at the source, instead.
        lat_deg = float(self.pos[0])
        if not math.isfinite(lat_deg) or abs(lat_deg) > 89.9:
            raise RuntimeError(
                f"INS estimate diverged: non-physical latitude {lat_deg} deg at "
                f"t={self.time:.3f}s (pos={self.pos.tolist()}, vel={self.vel.tolist()}). "
                "The navigation solution has run away from the true trajectory."
            )

        m_lat = coordinates.meter_per_deg_lat(self.pos[0])
        m_lon = coordinates.meter_per_deg_lon_at(self.pos[0])

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
        An injection point to replace the INS estimate with an externally corrected state (e.g. fed
        back from the Kalman Filter after a GPS/TERCOM fix).
        Allow an external system to update the INS state.
        e
        """
        self.pos = np.asarray(corrected_pos, dtype=float).copy()
        self.vel = np.asarray(corrected_vel, dtype=float).copy()

        if corrected_att is not None:
            self.att = np.asarray(corrected_att, dtype=float).copy()
            self._normalize_attitude()

    def get_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return copies of (position, velocity, attitude)."""
        return self.pos.copy(), self.vel.copy(), self.att.copy()