from dataclasses import dataclass
from .profile import MissileProfile
from enum import Enum, auto
import math
import numpy as np

class FlightStage(Enum):
    """
    Enum class for flight stages of the cruise missile. Can be implemented on other missile.
    """
    PRE_LAUNCHED = auto() # missile stand-by, in launch tube
    BOOST = auto() # booster phase, conduct rapid acceleration using rocket booster
    CRUISE = auto() # cruise stage, fly low, manoeuvre, using jet engine (avoid detection)
    TERMINAL = auto() # confirming target, choose attack type, ready for detonation
    IMPACT = auto() # impact and detonate


@dataclass
class MissileState:
    # positional fields
    x: float
    y: float
    z: float # this is the altitude, not y

    # velocity
    vx: float
    vy: float
    vz: float

    # orientation
    pitch: float
    roll: float
    heading: float

    # navigation & pathfinder
    lon: float
    lat: float
    altitude: float

    # time tracking
    time: float

    # pathfinder system state
    gps_valid: bool
    tercom_active: bool
    ins_calibrated: bool

    # tracking
    distance_traveled: float
    distance_to_target: float

    def get_speed(self) -> float:
        """
        Return the speed of the missile using velocity by Pythagorean theorem.
        """
        return np.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2) # velocity stores in numpy array

    def current_position(self) -> np.ndarray:
        """
        Return the current position of the missile (xyz)
        """
        return np.array([self.x, self.y, self.z])

    def get_velocity(self) -> np.ndarray:
        """
        Return the velocity of the missile.
        """
        return np.array([self.vx, self.vy, self.vz])

    def update_physics(self, dt: float, acceleration: np.ndarray, heading_rate: float) -> bool:
        """
        Update the missile state.

        Args:
            - dt: time delta, in seconds
            - acceleration: acceleration vector, in m/s^2 [vx, vy, vz]
            - heading_rate: heading change rate (rad/s)

        """

        # Velocity by formula: v_t = v_0 + a * t
        self.vx += acceleration[0] * dt
        self.vy += acceleration[1] * dt
        self.vz += acceleration[2] * dt

        # Position: position = initial position + velocity * delta time
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Heading
        self.heading += (heading_rate * dt)
        self.heading %= (2 * math.pi)

        self.time += dt

        self.distance_traveled += self.get_speed() * dt

        # z-up: z represents altitude, matching that in the pathfinding and trajectory.py
        self.altitude = self.z
