"""
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

_G = 9.80665


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
            state: MissileState,
            target: TargetGeometry,
            impact_angle_deg: float,
            terminal_dist_size_factor: float = 3.0,
            t_go_min: float=0.30,
            horizontal_nav_ratio:float=3.0
    ):
        self.profile = profile
        self.state = state
        self.target = target
        self.theta_mf = math.radians(impact_angle_deg)
        self.r_size_factor = terminal_dist_size_factor
        self.init_range = self.terminal_init_range()
        self.t_go_min = t_go_min

        """
        in the case of λh > 2 the required turn-ing acceleration of the vehicle is approaching zero near the final point
        """
        self.h_nav_ratio = horizontal_nav_ratio

    def terminal_init_range(self) -> float:
        """
        Eq. 41
        """
        v_cruise = self.profile.basic.cruise_speed_ms
        a_max = self.profile.get_max_lateral_acceleration()
        r_min = 2 * v_cruise ** 2 * abs(math.sin(self.theta_mf)) / a_max

        # size factor to allow earlier pull up to prevent entering terminal guidance at last minimum
        return self.r_size_factor * r_min

    def engage_terminal(self, state: MissileState) -> bool:
        """

        """
        return self.target.direct_ground_distance(state) <= self.init_range

    def update(self, state: MissileState,):
        self.theta = self._los_angles(state)
        self.theta_m = state.get_flight_path_angle()
        cruise_speed = max(state.get_ground_speed(), 1e-3)

        self._accel_climb()

    def _los_angle(self, state: MissileState) -> float:
        """
        Line-of-sight elevation angle to the target, from horizontal.

            LOS = atan2(target_alt - missile_alt, ground_range)

        Target below the missile (normal terminal case) => LOS < 0.
        """
        ground = self.target.direct_ground_distance(state)
        d_up = self.target.target_alt - state.est_alt
        return math.atan2(d_up, ground)

    def _time_to_go(self, state: MissileState, v_inst: float, LOS: float, theta_mf: float) -> float:
        """
        Estimate remaining flight time until impact
        Table 1, Eq 2.
        """
        r = self.target.direct_3d_distance(state)
        theta_m = state.get_flight_path_angle()
        theta_m_bar = theta_m - LOS
        theta_mf_bar = theta_mf - LOS

        v_mean = v_inst * (
                1.0
                - theta_m_bar ** 2 + theta_mf_bar ** 2 / 15.0
                + theta_m_bar * theta_mf_bar / 30.0
                + theta_m_bar ** 4 + theta_mf_bar ** 4 / 420.0
                - theta_m_bar * theta_mf_bar *
                (theta_m_bar ** 2 + theta_mf_bar ** 2 - theta_m_bar * theta_mf_bar) / 840.0
        )

        v_mean = max(v_mean, 1e-3)

        # max t_go is limited by t_go_min, preventing acceleration command to go crazy
        return max(r/v_mean, self.t_go_min)


    def _accel_climb(self, v_mean: float, t_go: float, LOS: float, theta_m: float):
        """
        The main formula for acceleration command for terminal guidance.
        Eq.26: Accel_cmd = Vm/t_go[-6theta(t) + 4theta_m(t) + 2theta_mf].

        Args:
            v_mean: mean velocity
            t_go: time to go (from _time_to_go)
            LOS: line-of-sight angle to target (theta)
            theta_m: flight path angle (from state)
        """
        theta = LOS
        accel_cmd = (v_mean / t_go) * (-6 * theta + 4 * theta_m + 2 * self.theta_mf)
        accel_max = self.profile.get_max_lateral_acceleration()
        return float(np.clip(accel_cmd, -accel_max, accel_max))

    def proportional_navigation(self):
        pass

    def _navigation_ratio(self, init_hdg: float, init_LOS: float) -> float:
        nav_ratio = np.sign(init_LOS) * math.pi + init_hdg / init_hdg
        return nav_ratio


    def _wrap_pi(self, angle_rad: float) -> float | int:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

