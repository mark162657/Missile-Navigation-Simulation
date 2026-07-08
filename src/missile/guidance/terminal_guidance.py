"""
Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

Terminal impact-angle guidance (Ryoo optimal guidance law).

Steers the missile in the 2-D vertical plane (downrange x height-above-target)
so it arrives at the aimpoint with a commanded flight-path (dive) angle as well
as zero miss distance. Implements the closed-form lag-free optimal guidance law
and the "mean-velocity" time-to-go estimator from:

    Ryoo, Cho, Tahk, "Optimal Guidance Laws with Terminal Impact Angle
    Constraint," J. Guidance, Control, and Dynamics, 28(4), 2005.
        - guidance law   : Eq. 26  (LOS form of Eq. 23)
        - time-to-go     : Table 1, Method 2 (range / mean velocity)
        - engage trigger : Eq. 41  (min-command circular-arc feasibility)

See docs/terminal_guidance_reference.md for the full extraction.

CONVENTIONS (all in the vertical plane, angles measured from horizontal, +up):
  * gamma_m  (flight-path angle) : atan2(v_up, v_horizontal); dive => negative.
  * lam      (LOS angle)         : atan2(target_alt - missile_alt, ground_range);
                                   target below the missile => negative.
  * gamma_f  (impact angle)      : desired flight-path angle AT impact, passed in
                                   as impact_angle_deg. Dive => NEGATIVE
                                   (-90 deg = straight down, -30 deg = shallow).
  * a_n (guidance normal accel)  : from the law; per the paper EOM  V*gamma_m_dot
                                   = -a_n, a POSITIVE command pitches the velocity
                                   DOWN (decreasing gamma_m).

PLANT MODEL (dynamics._force_enu): accel_climb is applied along lift_hat (perp to
velocity, up-ish = (-sin gamma_m, 0, cos gamma_m)), and full gravity (0,0,-g) is
added separately. So the net path-bending accel delivered is
    a_perp_net = accel_climb - g*cos(gamma_m).
We want a_perp_net = -a_n, hence  accel_climb = -a_n + g*cos(gamma_m).  (The cruise
autopilot's flat _G baseline is just this g*cos term at the gamma_m ~ 0 limit.)

INTEGRATION (FlightComputer must branch on stage == TERMINAL):
  accel_climb already carries the g*cos(gamma_m) gravity-hold term, so route
  TerminalCommand straight into ControlInput WITHOUT the autopilot's own _G
  baseline / altitude PID -- the dive is flown open of the altitude loop.
"""

import math
from dataclasses import dataclass

import numpy as np

from missile.guidance.target_geometry import TargetGeometry
from missile.state import MissileState
from missile.profile import MissileProfile

_G = 9.80665


@dataclass
class TerminalCommand:
    """
    Output of TerminalGuidance.update().

    Unlike PathFollower (which returns a target *altitude* for the autopilot's
    altitude PID), terminal guidance produces normal-acceleration commands
    decomposed into climb / turn directly.
    """
    accel_turn: float    # lateral (horizontal) accel command, m/s^2
    accel_climb: float   # vertical accel command (incl. gravity hold), m/s^2
    target_spd: float    # commanded speed, m/s


class TerminalGuidance:
    def __init__(
            self,
            target: TargetGeometry,
            profile: MissileProfile,
            impact_angle_deg: float,
            approach_azimuth_rad: float,
            *,
            k_az: float = 1.0,          # azimuth-hold gain (horizontal channel)
            size_factor: float = 3.0,   # safety multiple on the Eq.41 min engage range
            t_go_floor: float = 0.30,   # s, lower clamp on t_go (avoids 1/t_go blow-up)
    ):
        """
        Args:
            target: target geometry (aimpoint position + range helpers).
            profile: missile profile / specs (speed, max lateral accel).
            impact_angle_deg: desired flight-path angle AT impact, degrees.
                NEGATIVE for a dive (-90 = vertical, -30 = shallow). See module docstring.
            approach_azimuth_rad: plan-time approach bearing to hold (horizontal channel),
                radians from North (matches state.yaw convention).
            k_az: proportional gain for the azimuth-hold turn command.
            size_factor: safety multiple applied to the Eq. 41 minimum engage range.
            t_go_floor: minimum time-to-go used in the 1/t_go guidance gain.
        """
        self.target = target
        self.profile = profile
        self.gamma_f = math.radians(impact_angle_deg)   # flight-path angle at impact (dive => negative)
        self.approach_az = approach_azimuth_rad

        self.k_az = k_az
        self.size_factor = size_factor
        self.t_go_floor = t_go_floor

        self.d_init = self.terminal_init_range()

    # ------------------------------------------------------------------
    # Phase entry
    # ------------------------------------------------------------------
    def should_engage(self, state: MissileState) -> bool:
        """
        True once within the geometric terminal-initiation range. The FlightComputer
        owns the one-way latch (set stage = TERMINAL once, never revert).
        """
        return self.target.direct_ground_distance(state) <= self.d_init

    def terminal_init_range(self) -> float:
        """
        Downrange at which to begin the terminal maneuver.

        From the minimum-command circular-arc solution (Ryoo Eq. 41), the peak
        normal acceleration of an impact-angle engagement started at range R0 is

            a_min = 2 * V^2 * |sin(gamma_f)| / R0.

        Setting a_min = a_max gives the closest feasible start range; we begin
        earlier by `size_factor` so the law has margin under the accel limit:

            d_init = size_factor * 2 * V^2 * |sin(gamma_f)| / a_max.

        Steeper dives (larger |gamma_f|) => larger d_init, i.e. start earlier.
        """
        v = self.profile.basic.cruise_speed_ms
        a_max = self.profile.get_max_lateral_acceleration()
        r_min = 2.0 * v * v * abs(math.sin(self.gamma_f)) / a_max
        return self.size_factor * r_min

    # ------------------------------------------------------------------
    # Main guidance step
    # ------------------------------------------------------------------
    def update(self, state: MissileState, dt: float) -> TerminalCommand:
        gamma_m = self._flight_path_angle(state)
        lam = self._los_angle(state)
        speed = max(state.get_ground_speed(), 1e-3)

        t_go = self._time_to_go(state, speed, gamma_m, lam)
        a_n = self._guidance_accel(speed, t_go, gamma_m, lam)

        accel_climb, accel_turn = self._decompose(a_n, gamma_m, speed, state)

        return TerminalCommand(
            accel_turn=accel_turn,
            accel_climb=accel_climb,
            target_spd=self.profile.basic.cruise_speed_ms,
        )

    # ------------------------------------------------------------------
    # Plane geometry
    # ------------------------------------------------------------------
    def _flight_path_angle(self, state: MissileState) -> float:
        """gamma_m = atan2(v_up, sqrt(v_east^2 + v_north^2)), relative to horizontal."""
        return math.atan2(state.vel_up, math.hypot(state.vel_east, state.vel_north))

    def _los_angle(self, state: MissileState) -> float:
        """
        Line-of-sight elevation angle to the target, from horizontal.

            lam = atan2(target_alt - missile_alt, ground_range)

        Target below the missile (the normal terminal case) => lam < 0.
        """
        ground = self.target.direct_ground_distance(state)
        d_up = self.target.target_alt - state.est_alt
        return math.atan2(d_up, ground)

    # ------------------------------------------------------------------
    # Time-to-go  (Ryoo Table 1, Method 2: range / mean velocity)
    # ------------------------------------------------------------------
    def _time_to_go(self, state: MissileState, speed: float, gamma_m: float, lam: float) -> float:
        """
        t_go = R / V_mean, where the mean velocity along the curved impact-angle
        trajectory is corrected by the look-ahead angles relative to the LOS:

            look_m  = gamma_m - lam        (velocity above the LOS)
            look_f  = gamma_f - lam        (impact direction relative to the LOS)

            V_mean = V * [ 1 - (look_m^2 + look_f^2)/15 + look_m*look_f/30
                             + (look_m^4 + look_f^4)/420
                             - look_m*look_f*(look_m^2 + look_f^2 - look_m*look_f)/840 ]

        Plain R/V under-estimates t_go on the curved terminal path; this is the
        estimator the paper found most accurate (Method 2).
        """
        r = self.target.direct_3d_distance(state)
        lm = gamma_m - lam
        lf = self.gamma_f - lam

        lm2, lf2 = lm * lm, lf * lf
        v_mean = speed * (
            1.0
            - (lm2 + lf2) / 15.0
            + (lm * lf) / 30.0
            + (lm2 * lm2 + lf2 * lf2) / 420.0
            - (lm * lf) * (lm2 + lf2 - lm * lf) / 840.0
        )
        v_mean = max(v_mean, 1e-3)   # series can go non-physical at large look angles

        return max(r / v_mean, self.t_go_floor)

    # ------------------------------------------------------------------
    # Optimal guidance law  (Ryoo Eq. 26)
    # ------------------------------------------------------------------
    def _guidance_accel(self, speed: float, t_go: float, gamma_m: float, lam: float) -> float:
        """
        Lag-free optimal impact-angle command (Ryoo Eq. 26):

            a_n = (V / t_go) * ( -6*lam + 4*gamma_m + 2*gamma_f )

        The three coefficients sum to zero, so the law is invariant to the choice
        of angular reference line -- measuring lam / gamma_m / gamma_f all from
        horizontal is valid. An N=6 PN-like LOS term plus the impact-angle bias.

        Sign: per the paper EOM (V*gamma_m_dot = -a_n) a POSITIVE result pitches
        the velocity DOWN. Clamped to the airframe normal-accel envelope.
        """
        a_n = (speed / t_go) * (-6.0 * lam + 4.0 * gamma_m + 2.0 * self.gamma_f)

        a_max = self.profile.get_max_lateral_acceleration()
        return float(np.clip(a_n, -a_max, a_max))

    def _decompose(self, a_n: float, gamma_m: float, speed: float,
                   state: MissileState) -> tuple[float, float]:
        """
        Resolve the guidance normal accel into the flight-computer channels.

        Vertical: accel_climb acts along lift_hat (perp to V); dynamics adds full
        gravity separately, so the delivered path-bending accel is
        accel_climb - g*cos(gamma_m). Setting that equal to the guidance target
        -a_n (a positive a_n pitches DOWN) gives the g*cos gravity-hold term:

            accel_climb = -a_n + _G*cos(gamma_m)

        Horizontal: simple proportional azimuth hold onto the plan-time approach
        bearing (the vertical-plane law says nothing about the horizontal channel).
        """
        accel_climb = -a_n + _G * math.cos(gamma_m)

        az_err = self._wrap(self.approach_az - state.yaw)
        a_max = self.profile.get_max_lateral_acceleration()
        accel_turn = float(np.clip(self.k_az * speed * az_err, -a_max, a_max))

        return accel_climb, accel_turn

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap an angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
