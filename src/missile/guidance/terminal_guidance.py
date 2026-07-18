"""
While:
    - v_mean = mean velocity
    - t_go = time to go
    - theta = LOS
    - theta_m = flight path angle
    - theta_mf = impact angle
Source:
    Ryoo, C., Cho, H., & Tahk, M. (2005). Optimal Guidance Laws with Terminal Impact Angle
        Constraint. Journal of Guidance Control and Dynamics, 28(4),
        724–732. https://doi.org/10.2514/1.8392
"""

import math
from dataclasses import dataclass

import numpy as np

from missile.guidance.target_geometry import TargetGeometry
from missile.state import MissileState
from missile.profile import MissileProfile
from terrain.coordinates import CoordinateSystem

_G = 9.80665

# steepest impact angle, if over it, the missile will stall during terminal guidance
# and causing the missile to get deviated from target
_MAX_IMPACT_ANGLE_DEG = 55.0
# same deviation will occur if impact angle less than 5 degree
_MIN_IMPACT_ANGLE_DEG = 5


@dataclass
class TerminalCommand:
    """
    Output for ControlInput's acceleration command (turn and climb) and target speed, stored in dataclass.
    The output from the acceleration command is stored and assigned here.

    Args:
        accel_turn: lateral (horizontal) accel command, m/s^2
        accel_climb: vertical accel command (incl. gravity hold), m/s^2
        target_spd: commanded speed, m/s
    """
    accel_turn: float
    accel_climb: float
    target_spd: float


class TerminalGuidance:
    def __init__(
            self,
            profile: MissileProfile,
            target: TargetGeometry,
            impact_angle_deg: float,
            approach_azimuth_rad: float,
            terminal_dist_size_factor: float = 3.0,
            t_go_min: float=0.30,
            horizontal_nav_ratio:float=2.0
    ):
        self.profile = profile
        self.target = target
        
        # clamping impact angle in range of min to max
        sign = -1.0 if impact_angle_deg <= 0 else 1.0
        mag = min(_MAX_IMPACT_ANGLE_DEG, max(_MIN_IMPACT_ANGLE_DEG, abs(impact_angle_deg)))
        self.theta_mf = math.radians(sign * mag)
        self.r_size_factor = terminal_dist_size_factor
        self.init_range = self.terminal_init_range()
        self.t_go_min = t_go_min
        self.approach_az_rad = approach_azimuth_rad
        """
        in the case of λh > 2 the required turn-ing acceleration of the vehicle is approaching zero near the final point
        """
        self.h_nav_ratio = horizontal_nav_ratio
        self.accel_max = self.profile.get_max_lateral_acceleration()

    def terminal_init_range(self) -> float:
        """
        Eq. 41
        """
        v_cruise = self.profile.basic.cruise_speed_ms
        accel_max = self.profile.get_max_lateral_acceleration()
        r_min = 2 * v_cruise ** 2 * abs(math.sin(self.theta_mf)) / accel_max

        # size factor to allow earlier pull up to prevent entering terminal guidance at last minimum
        return self.r_size_factor * r_min

    def engage_terminal(self, state: MissileState) -> bool:
        """
        """
        return self.target.direct_3d_distance(state) <= self.init_range

    def update(self, state: MissileState):
        los = self._los_angle(state)
        theta_m = state.get_flight_path_angle()
        speed = max(state.get_ground_speed(), 1e-3)

        # vertical acceleration command
        t_go = self._time_to_go(
            state=state,
            v_inst=speed,
            los=los,
            theta_mf=self.theta_mf
        )

        accel_climb = self._accel_climb(
            v_inst=speed,
            t_go=t_go,
            los=los,
            theta_m=theta_m
        )

        # horizontal acceleration command
        accel_turn = self.proportional_navigation(state)

        return TerminalCommand(
            accel_turn=accel_turn,
            accel_climb=accel_climb,
            target_spd=speed
        )


    def _los_angle(self, state: MissileState) -> float:
        """
        Line-of-sight elevation angle to the target, from horizontal.

            los = atan2(target_alt - missile_alt, ground_range)

        Target below the missile (normal terminal case) => los < 0.
        """
        ground = self.target.direct_ground_distance(state)
        d_up = self.target.target_alt - state.est_alt
        return math.atan2(d_up, ground)

    def _time_to_go(self, state: MissileState, v_inst: float, los: float, theta_mf: float) -> float:
        """
        Estimate remaining flight time until impact
        Table 1, Eq 2.

        """
        r = self.target.direct_3d_distance(state)
        theta_m = state.get_flight_path_angle()
        theta_m_bar = theta_m - los
        theta_mf_bar = theta_mf - los

        v_mean = v_inst * (
                1.0
                - (theta_m_bar ** 2 + theta_mf_bar ** 2) / 15.0
                + theta_m_bar * theta_mf_bar / 30.0
                + (theta_m_bar ** 4 + theta_mf_bar ** 4) / 420.0
                - theta_m_bar * theta_mf_bar *
                (theta_m_bar ** 2 + theta_mf_bar ** 2 - theta_m_bar * theta_mf_bar) / 840.0
        )

        v_mean = max(v_mean, 1e-3)

        # max t_go is limited by t_go_min, preventing acceleration command to go crazy
        return max(r/v_mean, self.t_go_min)


    def _accel_climb(self, v_inst: float, t_go: float, los: float, theta_m: float):
        """
        The main formula for acceleration command for terminal guidance.
        Eq.26: Accel_cmd = Vm/t_go[-6theta(t) + 4theta_m(t) + 2theta_mf].

        Args:
            v_inst: instantaneous ground speed (from state)
            t_go: time to go (from _time_to_go)
            los: line-of-sight angle to target (theta)
            theta_m: flight path angle (from state)
        """
        theta = los
        a_n = (v_inst / t_go) * (-6 * theta + 4 * theta_m + 2 * self.theta_mf)
        a_n = float(np.clip(a_n, -self.accel_max, self.accel_max))

        # Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
        # a_n is the guidance normal accel; per the paper EOM V*gamma_m_dot = -a_n,
        # a positive a_n pitches the velocity DOWN. The plant applies accel_climb
        # along lift_hat (perp to V, +up) and adds full gravity separately, so the
        # delivered path-bending accel is accel_climb - g*cos(theta_m). Setting that
        # equal to the guidance target -a_n gives the gravity-hold conversion:
        return -a_n + _G * math.cos(theta_m)

    # -- Horizontal Guidance ---

    def proportional_navigation(self, state: MissileState):
        """

        """
        curr_east, curr_north = self.target.coord.latlong_to_enu(state.est_lat, state.est_lon)
        target_east, target_north = float(self.target.target_enu[0]), float(self.target.target_enu[1])

        # missile -> target bearing
        missile_target_los = CoordinateSystem.enu_bearing(curr_east, curr_north, target_east, target_north)
        # target -> missile bearing
        target_missile_los = CoordinateSystem.enu_bearing(target_east, target_north, curr_east, curr_north)

        ground_track = math.atan2(state.vel_east, state.vel_north)
        hdg_error = self._wrap_pi(missile_target_los - ground_track) # eq 1b

        r_h = max(self.target.direct_ground_distance(state), 1e-3) # replaced eq 1a
        v_h = state.get_horizontal_speed()
        nav_ratio = self._navigation_ratio(ground_track, target_missile_los)

        los_rate = (v_h / r_h) * math.sin(hdg_error)

        # substitute eq 1b to 2
        hdg_rate = nav_ratio * los_rate

        # acceleration = rate * speed
        a_turn = hdg_rate * v_h

        return float(np.clip(a_turn, -self.accel_max, self.accel_max))


    def _navigation_ratio(self, heading: float, los: float) -> float:
        """

        Define:
            ox_axis: a reference direction, opposite of the final approach heading
                e.g. E 90 -> W 270

        Args:
            heading: the heading of the missile
            los: the line-of-sight angle from target to missile
        """
        ox_axis = self._wrap_pi(self.approach_az_rad - math.pi)
        hdg_shift = self._wrap_pi(heading - ox_axis)
        los_shift = self._wrap_pi(los - ox_axis)

        if abs(los_shift) < 1e-3:
            return self.h_nav_ratio

        # main formula for navigation ratio (lambda_h), eq5
        nav_ratio = (math.copysign(math.pi, los_shift) + hdg_shift) / los_shift
        return max(nav_ratio, self.h_nav_ratio) # clamp nav ratio to at least h_nav_ratio (2.0 by default)

    def _wrap_pi(self, angle_rad: float) -> float | int:
        """
        Wrap an angle difference to [-pi, pi].

        Args:
            angle_rad: angle in radians
        """
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi



