from dataclasses import dataclass
from enum import Enum, auto
import math

import numpy as np

from missile.navigation.ins import INS
from terrain import coordinates


class FlightStage(Enum):
    """Flight stages of the cruise missile."""
    PRE_LAUNCHED = auto()
    BOOST = auto()
    CRUISE = auto()   # Cruise/in-flight guidance
    TERMINAL = auto() # Terminal guidance (with impact angle)
    IMPACT = auto()   # Detonate boom


@dataclass
class MissileState:
    """
    Shared missile state for simulation and navigation.

    Geographic position (degrees, degrees, meters MSL):
        true_lat / true_lon / true_alt  — simulation ground truth only
        est_lat  / est_lon  / est_alt   — navigation estimate (INS + KF)

    Velocity (local ENU, m/s — not lat/lon rates):
        vel_east, vel_north, vel_up

    Attitude (radians, matches INS):
        roll, pitch, yaw

    The INS never reads true_lat/true_lon/true_alt. The simulation feeds truth
    to sensor models (GPS, radar altimeter, TERCOM), which return noisy
    measurements to the nav computer.

    Pixel (row, col) and ENU meter offsets (east_m, north_m) are not stored
    here — use DEMLoader and CoordinateSystem at conversion boundaries.
    """
    
    # Simulation ground truth (lat, lon, alt MSL)
    true_lat: float
    true_lon: float
    true_alt: float

    # Navigation estimate (lat, lon, alt MSL)
    est_lat: float
    est_lon: float
    est_alt: float

    # Velocity (east, north, up) in m/s
    vel_east: float
    vel_north: float
    vel_up: float

    # Orientation (radians)
    roll: float # USELESS for now
    pitch: float# mostly USELESS
    yaw: float # heading

    # Time and bookkeeping (mirrors INS where applicable)
    time: float
    distance_traveled: float
    distance_to_target: float

    # Sensor / nav flags
    gps_valid: bool
    tercom_active: bool
    ins_calibrated: bool

    # Missile flight stage record (defaults to pre-launch; physics/sim loop advance it)
    missile_stage: FlightStage = FlightStage.PRE_LAUNCHED

    def get_ground_speed(self) -> float:
        """Return speed magnitude from velocity components, m/s."""
        return float(np.linalg.norm([self.vel_east, self.vel_north, self.vel_up]))

    def est_position(self) -> np.ndarray:
        """Return estimated geographic position [lat, lon, alt]."""
        return np.array([self.est_lat, self.est_lon, self.est_alt])

    def true_position(self) -> np.ndarray:
        """Return simulation ground truth [lat, lon, alt]."""
        return np.array([self.true_lat, self.true_lon, self.true_alt])

    def get_velocity(self) -> np.ndarray:
        """Return ENU velocity [vel_east, vel_north, vel_up] in m/s."""
        return np.array([self.vel_east, self.vel_north, self.vel_up])

    def get_attitude(self) -> np.ndarray:
        """Return attitude [roll, pitch, yaw] in radians."""
        return np.array([self.roll, self.pitch, self.yaw])

    def apply_ins_estimate(self, ins: INS) -> None:
        """Copy INS dead-reckoned / corrected state into the nav estimate fields."""
        pos, vel, att = ins.get_state()
        self.est_lat = float(pos[0])
        self.est_lon = float(pos[1])
        self.est_alt = float(pos[2])
        self.vel_east = float(vel[0])
        self.vel_north = float(vel[1])
        self.vel_up = float(vel[2])
        self.roll = float(att[0])
        self.pitch = float(att[1])
        self.yaw = float(att[2])
        self.time = ins.time
        self.distance_traveled = ins.distance_traveled

    def apply_kf_position(self, position: np.ndarray | list[float]) -> None:
        """Update estimated geographic position from Kalman filter output."""
        pos = np.asarray(position, dtype=float)
        self.est_lat = float(pos[0])
        self.est_lon = float(pos[1])
        self.est_alt = float(pos[2])

    def update_physics(
        self,
        dt: float,
        acceleration: np.ndarray | list[float],
        yaw_rate: float,
        *,
        reference_lat: float | None = None,
    ) -> None:
        """
        Advance simulation ground truth (true_lat, true_lon, true_alt) and kinematics.

        Does **not** modify the navigation estimate (est_lat, est_lon, est_alt).
        That is owned by INS + Kalman filter. Only the simulation layer calls this.
                
        Args:
            dt: timestep in seconds
            acceleration: [ax east, ay north, az up] in m/s^2
            yaw_rate: yaw rate in rad/s
            reference_lat: latitude for lon scaling; defaults to current true_lat
        """
        # We use m/s to get meters moved, then using meter_per_deg_lat/lon_at to get lat/lon change in degrees.
        acc = np.asarray(acceleration, dtype=float)
        lat_ref = float(self.true_lat if reference_lat is None else reference_lat)
        m_lon = coordinates.meter_per_deg_lon_at(lat_ref)
        m_lat = coordinates.meter_per_deg_lat(lat_ref)

        prev_east = self.vel_east
        prev_north = self.vel_north
        prev_up = self.vel_up

        # Integrate truth position in geographic frame.
        self.true_lat += (prev_north * dt + 0.5 * float(acc[1]) * dt ** 2) / m_lat
        self.true_lon += (prev_east * dt + 0.5 * float(acc[0]) * dt ** 2) / m_lon
        self.true_alt += prev_up * dt + 0.5 * float(acc[2]) * dt ** 2

        self.vel_east += float(acc[0]) * dt
        self.vel_north += float(acc[1]) * dt
        self.vel_up += float(acc[2]) * dt

        self.yaw = (self.yaw + yaw_rate * dt) % (2 * math.pi)

        self.time += dt
        self.distance_traveled += self.get_ground_speed() * dt
