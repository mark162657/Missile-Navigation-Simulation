from dataclasses import dataclass
from enum import Enum, auto
import math

import numpy as np

from missile.navigation.ins import INS


class FlightStage(Enum):
    """Flight stages of the cruise missile."""
    PRE_LAUNCHED = auto()
    BOOST = auto()
    CRUISE = auto()
    TERMINAL = auto()
    IMPACT = auto()


# WGS84-ish local scale factors (meters per degree).
_METER_PER_DEG_LAT = 111_320.0


def _meter_per_deg_lon(lat_deg: float) -> float:
    return _METER_PER_DEG_LAT * math.cos(math.radians(lat_deg))


@dataclass
class MissileState:
    """
    Shared missile state for simulation and navigation.

    Position convention (everywhere in this project):
        x = latitude (degrees)
        y = longitude (degrees)
        z = altitude MSL (meters)

    Velocity convention (local ENU, m/s):
        vx = east, vy = north, vz = up

    Attitude convention (radians, matches INS):
        roll, pitch, heading (yaw)

    True position (tx, ty, tz) is simulation-universe ground truth only.
    The INS never reads tx/ty/tz — the simulation feeds truth to sensor models
    (GPS, radar altimeter, TERCOM) which return noisy measurements to nav.
    """

    # Simulation ground truth (lat, lon, alt) — not visible to INS / nav brain
    tx: float
    ty: float
    tz: float

    # Navigation estimate (lat, lon, alt)
    x: float
    y: float
    z: float

    # Velocity (east, north, up) in m/s
    vx: float
    vy: float
    vz: float

    # Orientation (radians) — roll, pitch, yaw/heading
    roll: float
    pitch: float
    heading: float

    # Time and bookkeeping (mirrors INS where applicable)
    time: float
    distance_traveled: float
    distance_to_target: float

    # Sensor / nav flags
    gps_valid: bool
    tercom_active: bool
    ins_calibrated: bool

    def get_speed(self) -> float:
        """Return speed magnitude from velocity components, m/s."""
        return float(np.linalg.norm([self.vx, self.vy, self.vz]))

    def current_position(self) -> np.ndarray:
        """Return estimated position [lat, lon, alt]."""
        return np.array([self.x, self.y, self.z])

    def true_position(self) -> np.ndarray:
        """Return simulation ground truth [lat, lon, alt]."""
        return np.array([self.tx, self.ty, self.tz])

    def get_velocity(self) -> np.ndarray:
        """Return velocity [vx east, vy north, vz up] in m/s."""
        return np.array([self.vx, self.vy, self.vz])

    def get_attitude(self) -> np.ndarray:
        """Return attitude [roll, pitch, heading] in radians."""
        return np.array([self.roll, self.pitch, self.heading])

    def apply_ins_estimate(self, ins: INS) -> None:
        """Copy INS dead-reckoned / corrected state into the nav estimate fields."""
        pos, vel, att = ins.get_state()
        self.x, self.y, self.z = float(pos[0]), float(pos[1]), float(pos[2])
        self.vx, self.vy, self.vz = float(vel[0]), float(vel[1]), float(vel[2])
        self.roll, self.pitch, self.heading = float(att[0]), float(att[1]), float(att[2])
        self.time = ins.time
        self.distance_traveled = ins.distance_traveled

    def apply_kf_position(self, position: np.ndarray | list[float]) -> None:
        """Update estimated geographic position from Kalman filter output."""
        pos = np.asarray(position, dtype=float)
        self.x, self.y, self.z = float(pos[0]), float(pos[1]), float(pos[2])

    def update_physics(
        self,
        dt: float,
        acceleration: np.ndarray | list[float],
        heading_rate: float,
        *,
        reference_lat: float | None = None,
    ) -> None:
        """
        Advance simulation ground truth (tx, ty, tz) and kinematics.

        Does **not** modify the navigation estimate (x, y, z). That is owned
        by INS + Kalman filter. Only the simulation layer calls this.

        Args:
            dt: timestep in seconds
            acceleration: [ax east, ay north, az up] in m/s^2
            heading_rate: yaw rate in rad/s
            reference_lat: latitude for lon scaling; defaults to current tx
        """
        acc = np.asarray(acceleration, dtype=float)
        lat_ref = float(self.tx if reference_lat is None else reference_lat)
        m_lon = _meter_per_deg_lon(lat_ref)

        prev_vx, prev_vy, prev_vz = self.vx, self.vy, self.vz

        # Integrate truth position in geographic frame.
        self.tx += (prev_vy * dt + 0.5 * float(acc[1]) * dt ** 2) / _METER_PER_DEG_LAT
        self.ty += (prev_vx * dt + 0.5 * float(acc[0]) * dt ** 2) / m_lon
        self.tz += prev_vz * dt + 0.5 * float(acc[2]) * dt ** 2

        self.vx += float(acc[0]) * dt
        self.vy += float(acc[1]) * dt
        self.vz += float(acc[2]) * dt

        self.heading = (self.heading + heading_rate * dt) % (2 * math.pi)

        self.time += dt
        self.distance_traveled += self.get_speed() * dt
