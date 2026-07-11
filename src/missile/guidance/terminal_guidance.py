"""

Assuming A. Optimal Guidance Law for Lag-Free Autopilot. As our autopilot is designed to be lag-free in acceleration.
Main formula: A_cmd = Vm/t_go[-6theta(t) + 4theta_m(t) + 2theta_mf]

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

    ):
        self.profile = profile
        self.state = state
        self.target = target
        self.theta_mf = math.radians(impact_angle_deg)


    def _min_init_range(self):
        """
        Eq. 41
        """
        

    def _los_angle(self, state: MissileState) -> float:
        """
        Line-of-sight elevation angle to the target, from horizontal.

            LOS = atan2(target_alt - missile_alt, ground_range)

        Target below the missile (normal terminal case) => LOS < 0.
        """
        ground = self.target.direct_ground_distance(state)
        d_up = self.target.target_alt - state.est_alt
        return math.atan2(d_up, ground)

    def _time_to_go(self, state: MissileState, speed: float, theta_mf: float):
        """

        """
        theta_m = state.get_flight_path_angle()
        v_m = speed * (
                1.0
                - theta_m ** 2 + theta_mf ** 2 / 15.0
                + theta_m * theta_mf / 30.0
                + theta_m ** 4 + theta_mf ** 4 / 420.0
                - theta_m * theta_mf * (theta_m ** 2 + theta_mf ** 2 - theta_m * theta_mf) / 840.0
        )



    def _accel_cmd(self, vm: float, t_go, LOS: float, theta_m: float):
        """
        Eq.26: Accel_cmd = Vm/t_go[-6theta(t) + 4theta_m(t) + 2theta_mf]

        While:
            - Vm = mean velocity
            - t_go = time to go
            - theta(t) = LOS
            - theta_m = flight path angle
            - theta_mf = impact angle
        """
        accel_cmd = vm / t_go(-6 * LOS + 4 * theta_m + 2 * self.theta_mf)
        accel_max = self.profile.get_max_lateral_acceleration()
        return float(np.clip(accel_cmd, -accel_max, accel_max))