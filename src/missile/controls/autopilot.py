"""
Autopilot serves as an outer layer that calls the PID controller at the bottom of the simulation.
It handles mainly altitude and speed, while turn is the job for guidance.
"""

import math

import numpy as np
import numpy.typing as npt

from missile.profile import MissileProfile
from missile.state import MissileState
from missile.controls.pid_controller import PIDController
from missile.controls.control_input import ControlInput
from simulation.physics import atmosphere

_G = 9.80665 # gravity m/s
_K_H = 0.5 # 1/s, scaling factor of converting altitude error to V/S command
_VS_MAX = 25 # maximum vertical speed cap m/s

# prevents the missile from slamming into the ground (dive angles too sharp) when transition to CRUISE from BOOST
_MAX_DIVE_ANGLE = math.radians(8.0)

class AutoPilot:
    def __init__(self, profile: MissileProfile):
        self.profile = profile
        accel_max = profile.get_max_lateral_acceleration()

        # initiate pid controller for altitude and speed
        # with soft limit (pid saturation)
        self.vs_pid = PIDController(kp=2.0, ki=0.3, kd=0.5, out_max=accel_max, out_min=-accel_max)
        self.spd_pid = PIDController(kp=0.02, ki=0.005, kd=0.5, out_max=1.0, out_min=0) # throttle: 0 - 1

        # Last-tick command telemetry (for UI / debugging). Populated by update().
        self.last_target_alt = 0.0
        self.last_target_spd = 0.0
        self.last_vs_cmd = 0.0
        self.last_accel_climb = 0.0
        self.last_throttle = 0.0

    def update(
            self,
            state: MissileState,
            target_alt: float,
            target_spd: float,
            lateral_accel_cmd,
            dt: float
    ) -> ControlInput:
        """
        Receive input of what guidance want + where we are now + how much time passed.
        Pass them into PID and we make the output into ControlInput.
        As turn acceleration was handled in path_following / guidance related components,
        we will handle speed: throttle, and climb here.

        Args:
            state: the MissileState
            target_alt: the target altitude we want to reach / maintain
            target_spd: the target speed
            lateral_accel_cmd: the lateral acceleration command calculated in guidance path_follower
            dt: delta time (time changes

        Return:
            Fill in the fields for ControlInput, including: throttle (0-1), accel_turn, accel_climb.
        """

        curr_alt = state.est_alt
        curr_spd = state.get_ground_speed()
        curr_vs = state.vel_up

        # handles error for PID (target - measurement)
        alt_error = target_alt - curr_alt # diff in target and current altitude
        spd_error = target_spd - curr_spd

        # V/S command
        # h_cmd = K_h * Delta_h (altitude error)

        # cap the sink, by the smallest of max dive angle's vertical speed or the max vs speed
        sink_cap = min(_VS_MAX, max(curr_spd, 1.0) * math.tan(_MAX_DIVE_ANGLE)) # tan is used as we use ground speed
        vs_cmd = max(-sink_cap, min(_VS_MAX, _K_H * alt_error)) # outer max() cap the dive, inner min() cap the climb

        # altitude command
        vs_error = vs_cmd - curr_vs
        accel_climb = _G + self.vs_pid.update(vs_error, curr_vs, dt)

        # speed command
        throttle = self.spd_pid.update(spd_error, curr_spd, dt)

        # record last-tick commands for telemetry / UI
        self.last_target_alt = float(target_alt)
        self.last_target_spd = float(target_spd)
        self.last_vs_cmd = float(vs_cmd)
        self.last_accel_climb = float(accel_climb)
        self.last_throttle = float(throttle)

        return ControlInput(
            throttle=throttle,
            accel_turn=lateral_accel_cmd,
            accel_climb=accel_climb
        )

    def reset(self) -> None:
        """
        Clearing both PID when transition of target, and boost -> cruise phase
        Resetting allows us to clear out the accumulated error and control in PID,
        as cruise and boost phase has totally different behaviour.
        """
        self.vs_pid.reset()
        self.spd_pid.reset()













