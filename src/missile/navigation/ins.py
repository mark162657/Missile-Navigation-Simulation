import math
import numpy as np


class INS:
    """
    Lightweight INS for simulation and Kalman prediction.

    This class intentionally keeps the model simple:
    - position and velocity are tracked in world coordinates
    - acceleration is given in world coordinates
    - attitude is optional and updated separately
    """

    def __init__(
        self,
        init_pos: np.ndarray | list[float],
        init_vel: np.ndarray | list[float],
        init_att: np.ndarray | list[float] | None = None) -> None:

        self.pos = np.asarray(init_pos, dtype=float).copy()
        self.vel = np.asarray(init_vel, dtype=float).copy()

        if init_att is None:
            init_att = [0.0, 0.0, 0.0]
        self.att = np.asarray(init_att, dtype=float).copy()
        self._normalize_attitude()

    def _normalize_attitude(self) -> None:
        self.att = np.array([angle % (2 * math.pi) for angle in self.att], dtype=float)

    def predict(
        self,
        acceleration: np.ndarray | list[float],
        dt: float,
        angular_velocity: np.ndarray | list[float] | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Update the INS by dead reckoning.

        Args:
            acceleration: [ax, ay, az] in world frame
            dt: timestep in seconds
            angular_velocity: optional [pitch_rate, roll_rate, heading_rate]
        """
        acc = np.asarray(acceleration, dtype=float)

        if angular_velocity is None:
            angular_velocity = [0.0, 0.0, 0.0]
        ang_vel = np.asarray(angular_velocity, dtype=float)

        previous_velocity = self.vel.copy()

        self.pos += previous_velocity * dt + 0.5 * acc * (dt ** 2)
        self.vel += acc * dt
        self.att += ang_vel * dt
        self._normalize_attitude()

        return self.get_state()

    def correct_state(
        self,
        corrected_pos: np.ndarray | list[float],
        corrected_vel: np.ndarray | list[float],
        corrected_att: np.ndarray | list[float] | None = None,) -> None:
        """
        Replace INS estimate with corrected external state.
        """
        self.pos = np.asarray(corrected_pos, dtype=float).copy()
        self.vel = np.asarray(corrected_vel, dtype=float).copy()

        if corrected_att is not None:
            self.att = np.asarray(corrected_att, dtype=float).copy()
            self._normalize_attitude()

    def get_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.pos.copy(), self.vel.copy(), self.att.copy()

    def get_state_vector(self) -> np.ndarray:
        """
        Return Kalman-friendly 6D state vector:
        [x, y, z, vx, vy, vz]
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
    def get_transition_matrix(dt: float) -> np.ndarray:
        """
        State transition matrix for [x, y, z, vx, vy, vz].
        """
        A = np.eye(6)
        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt
        return A

    @staticmethod
    def get_control_matrix(dt: float) -> np.ndarray:
        """
        Control matrix for acceleration input [ax, ay, az].
        """
        B = np.zeros((6, 3))
        B[0, 0] = 0.5 * dt ** 2
        B[1, 1] = 0.5 * dt ** 2
        B[2, 2] = 0.5 * dt ** 2
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt
        return B
