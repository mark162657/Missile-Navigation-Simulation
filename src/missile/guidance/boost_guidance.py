"""
boost_guidance.py -- closed-loop boost guidance (3-DoF point mass).

WHAT IT DOES
    Replaces the open-loop, clock-driven boost pitch schedule with a closed-loop
    guidance law that produces the commanded body attitude (roll, pitch, yaw)
    during BOOST. Two independent channels:

    1. VERTICAL (pitch-over): a reference flight-path angle gamma_ref is scheduled
       on ALTITUDE CLIMBED (not the burn clock), eased from near-vertical at
       launch to level at handoff with a quintic. A PID tracks it and outputs the
       extra pitch (angle of attack, alpha = theta - gamma) needed to fight
       gravity. The pitch command is theta_cmd = gamma_ref + alpha, slew-limited.

    2. LATERAL (trajectory intercept): a pure-pursuit heading toward a look-ahead
       point on the planned A* route, so the missile leaves boost already aligned
       with the cruise path instead of on a fixed bearing.

SOURCE / REDUCTION (Koklucan 2019, "Guidance and Control of a Submarine-Launched
Cruise Missile", METU MSc; equation numbers below refer to it)
    - The thesis boost model is longitudinal: state [u, w, q, theta, z] with
      control [T, theta_T] (booster thrust + thrust-vector deflection), Eq (5.15);
      pitch kinematics theta_dot = q, z_dot = -sin(theta)u + cos(theta)w, Eq (5.4).
    - We reduce it to a point mass: drop the pitch-rate q and the moment tier
      (I_y, C_m, theta_T) and command the pitch attitude theta directly. Flight-
      path angle gamma = atan2(vel_up, |vel_horizontal|) (point-mass analog of
      alpha = atan(w/u), Eq 3.71).
    - Control objective mirrors Sec 6.3.2: "the aim is to follow a desired pitch
      angle profile ... and keep roll zero." Target end-state is level pitch at
      burnout, theta_f = 0, Eq (5.40).
    - The angle-of-attack cap reflects the thesis TVC bound -12 deg <= theta_T
      <= 12 deg, Eq (5.35): our steering authority comes from bounded thrust
      deflection, so |alpha| is capped at 12 deg.
    - The thesis boost is planar (no lateral motion, Sec 5.2.2). The lateral
      trajectory-intercept channel here is our addition for a 3-D route-following
      mission (pure-pursuit / L1-style capture of the planned path).

Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from missile.controls.pid_controller import PIDController
from missile.state import MissileState
from terrain.coordinates import CoordinateSystem


@dataclass
class BoostGuidanceSpecs:
    """
    Tuning constants for boost guidance (representative; exact values not critical).

    Attributes:
        gamma_launch:   reference flight-path angle at launch (rad, ~near-vertical)
        gamma_handoff:  reference flight-path angle at end of pitch-over (rad, ~0)
        H_pitch:        altitude band over which the pitch-over completes (m) -- the
                        main knob trading altitude for speed (smaller -> faster/lower)
        v_min:          below this speed gamma is ill-defined; hold the launch attitude
        theta_rate_max: max slew of the commanded pitch (rad/s), actuator stand-in
        lookahead_dist: pure-pursuit look-ahead distance for lateral capture (m)
        kp, ki, kd:     pitch-tracking PID gains (output = angle of attack alpha)
        alpha_limit:    |alpha| cap (rad); the TVC bound of Eq (5.35), 12 deg
    """
    gamma_launch: float
    gamma_handoff: float

    H_pitch: float = 250.0
    v_min: float = 15.0
    theta_rate_max: float = math.radians(40.0)
    lookahead_dist: float = 500.0

    # Pitch-tracking PID on the flight-path-angle error; output is the angle of
    # attack alpha, bounded to the thesis TVC deflection limit (Eq 5.35).
    kp: float = 0.5
    ki: float = 0.05
    kd: float = 0.0
    alpha_limit: float = math.radians(12.0)


class BoostGuidance:
    """
    Closed-loop boost guidance: commands (roll, pitch, yaw) each BOOST tick.

    Vertical channel = altitude-scheduled flight-path-angle tracking (pitch-over).
    Lateral channel  = pure-pursuit intercept of the planned A* route (optional;
    falls back to the cruise heading when no trajectory is supplied).
    """

    def __init__(
        self,
        specs: BoostGuidanceSpecs,
        coordinate: CoordinateSystem | None = None,
        trajectory: np.ndarray | None = None,
        cruise_heading_rad: float = 0.0,
    ) -> None:
        """
        Args:
            specs:              tuning constants (see BoostGuidanceSpecs).
            coordinate:         CoordinateSystem for lat/lon -> ENU (lateral channel).
            trajectory:         planned path, (N, >=3) [lat, lon, ground_elev]. When
                                given with `coordinate`, enables lateral intercept.
            cruise_heading_rad: fallback / initial heading (rad, CW from north).
        """
        self.specs = specs
        self.cruise_heading = float(cruise_heading_rad)

        # Vertical channel: PID output is the angle of attack, capped at the TVC bound.
        self.pitch_pid = PIDController(
            kp=specs.kp, ki=specs.ki, kd=specs.kd,
            out_max=specs.alpha_limit, out_min=-specs.alpha_limit,
        )
        self._h_launch: float | None = None          # captured on first call
        self._theta_prev: float = specs.gamma_launch  # for the pitch slew limit

        # Lateral channel: project the planned route to ENU once (if provided).
        self.coord = coordinate
        self._traj_enu: np.ndarray | None = None
        self._last_idx = 0
        if coordinate is not None and trajectory is not None:
            self._traj_enu = self._project_trajectory(coordinate, trajectory)

    # ------------------------------------------------------------------
    # Public entry point (called once per BOOST tick by the sequencer)
    # ------------------------------------------------------------------
    def commanded_attitude(self, state: MissileState, dt: float) -> tuple[float, float, float]:
        """
        Return the commanded (roll, pitch, yaw) in radians for this BOOST tick.

        Roll is held at zero (thesis Sec 6.3.2 "keep roll zero"); pitch comes from
        the vertical pitch-over channel; yaw from the lateral trajectory-intercept
        channel.
        """
        v_e, v_n, v_u = state.vel_east, state.vel_north, state.vel_up
        speed = math.sqrt(v_e * v_e + v_n * v_n + v_u * v_u)
        gamma = math.atan2(v_u, math.hypot(v_e, v_n))   # flight-path angle (rad)

        pitch = self._pitch_command(state.est_alt, speed, gamma, dt)
        yaw = self._heading_command(state, speed)
        return (0.0, pitch, yaw)

    def reset(self) -> None:
        """Clear PID + internal state (e.g. if boost is re-armed)."""
        self.pitch_pid.reset()
        self._h_launch = None
        self._theta_prev = self.specs.gamma_launch
        self._last_idx = 0

    # ------------------------------------------------------------------
    # Vertical channel: pitch-over (flight-path-angle tracking)
    # ------------------------------------------------------------------
    def _pitch_command(self, h: float, speed: float, gamma: float, dt: float) -> float:
        """theta_cmd = gamma_ref(altitude) + PID(gamma_ref - gamma), slew-limited."""
        if self._h_launch is None:
            self._h_launch = h

        # Below flying speed gamma is numerically garbage -> hold the launch angle.
        if speed < self.specs.v_min:
            self._theta_prev = self.specs.gamma_launch
            return self.specs.gamma_launch

        sigma = self._progress(h)                    # [0, 1] on altitude climbed
        s = self._smoothstep(sigma)                  # quintic
        gamma_ref = self._gamma_ref(s)               # reference flight-path angle

        # PID supplies alpha = theta - gamma (bounded to the TVC limit) to track the
        # reference and (via ki) trim the steady gravity sag.
        alpha = self.pitch_pid.update(gamma_ref - gamma, gamma, dt)
        theta_cmd = gamma_ref + alpha

        theta_cmd = self._rate_limit(theta_cmd, dt)
        self._theta_prev = theta_cmd
        return theta_cmd

    def _progress(self, h: float, h_pitch:float=BoostGuidanceSpecs.H_pitch) -> float:
        """sigma = clip((h - h_launch) / H_pitch, 0, 1)."""
        span = self.specs.H_pitch
        if span <= 0.0:
            return 1.0
        return max(0.0, min(1.0, (h - self._h_launch) / span))

    @staticmethod
    def _smoothstep(sigma: float) -> float:
        """Quintic smoothstep 10s^3 - 15s^4 + 6s^5 (zero 1st & 2nd derivative at ends)."""
        return sigma * sigma * sigma * (sigma * (sigma * 6.0 - 15.0) + 10.0)

    def _gamma_ref(self, s: float) -> float:
        """gamma_ref = gamma_launch*(1 - s) + gamma_handoff*s."""
        return self.specs.gamma_launch * (1.0 - s) + self.specs.gamma_handoff * s

    def _rate_limit(self, theta_cmd: float, dt: float) -> float:
        """Limit how fast the commanded pitch may move (actuator stand-in)."""
        max_step = self.specs.theta_rate_max * dt
        lo = self._theta_prev - max_step
        hi = self._theta_prev + max_step
        return max(lo, min(theta_cmd, hi))

    # ------------------------------------------------------------------
    # Lateral channel: trajectory intercept (pure pursuit onto the A* route)
    # ------------------------------------------------------------------
    def _heading_command(self, state: MissileState, speed: float) -> float:
        """
        Pure-pursuit heading (rad, CW from north) toward a look-ahead point on the
        planned route. Falls back to the cruise heading with no route or below
        flying speed (heading of a near-zero velocity is meaningless).
        """
        if self._traj_enu is None or speed < self.specs.v_min:
            return self.cruise_heading

        pos = np.asarray(self.coord.latlong_to_enu(state.est_lat, state.est_lon), dtype=float)
        closest = self._find_closest(pos)
        aim = self._lookahead(closest)
        delta = self._traj_enu[aim] - pos                 # (east, north)
        if float(np.hypot(delta[0], delta[1])) < 1e-6:
            return self.cruise_heading
        return math.atan2(delta[0], delta[1])             # bearing CW from north

    @staticmethod
    def _project_trajectory(coord: CoordinateSystem, trajectory: np.ndarray) -> np.ndarray:
        """Project the (N, >=2) [lat, lon, ...] route to (N, 2) ENU east/north."""
        traj = np.asarray(trajectory, dtype=float)
        return np.asarray(
            [coord.latlong_to_enu(float(la), float(lo)) for la, lo in traj[:, :2]],
            dtype=float,
        )

    def _find_closest(self, pos: np.ndarray, window: int = 50) -> int:
        """Nearest route node to `pos`, searched forward from the last index (monotonic)."""
        end = min(self._last_idx + window, len(self._traj_enu))
        seg = self._traj_enu[self._last_idx:end]
        dist = np.linalg.norm(seg - pos, axis=1)
        self._last_idx = self._last_idx + int(np.argmin(dist))
        return self._last_idx

    def _lookahead(self, closest_idx: int) -> int:
        """Index of the node ~lookahead_dist metres ahead of the closest node."""
        i, dist = closest_idx, 0.0
        n = len(self._traj_enu)
        while i < n - 1 and dist < self.specs.lookahead_dist:
            dist += float(np.linalg.norm(self._traj_enu[i + 1] - self._traj_enu[i]))
            i += 1
        return i

    def _terrain_ahead(self):
        pass
