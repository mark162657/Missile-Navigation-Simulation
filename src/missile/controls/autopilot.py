"""
Autopilot serves as an outer layer that calls the PID controller at the bottom of the simulation.
It handles mainly altitude and speed, while turn is the job for guidance.
"""

import numpy as np
import numpy.typing as npt

from missile.profile import MissileProfile
from missile.state import MissileState
from pid_controller import PIDController
from control_input import ControlInput
from simulation.physics import atmosphere

# gravity m/s
_G = 9.80665

class AutoPilot:
    def __init__(self, profile: MissileProfile):
        self.profile = profile
        accel_max = profile.get_max_lateral_acceleratio()

        # initiate pid controller for altitude and speed
        # with soft limit (pid saturation)
        self.alt_pid = PIDController(kp=0.0, ki=0.0, kd=0.0, out_max=accel_max, out_min=-accel_max)
        self.spd_pid = PIDController(kp=0.0, ki=0.0, kd=0.0, out_max=1.0, out_min=0) #throttle: 0 - 1

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

        # handles error for PID (target - measurement)
        alt_error = target_alt - curr_alt
        spd_error = target_spd - curr_spd

        # altitude command
        accel_climb = _G + self.alt_pid.update(alt_error, curr_alt, dt)

        # speed command
        throttle = self.spd_pid.update(spd_error, curr_spd, dt)

        accel_turn = lateral_accel_cmd

        return ControlInput(
            throttle=throttle,
            accel_turn=accel_turn,
            accel_climb=accel_climb
        )

    def reset(self):
        """
        Clearing both PID when transition of target, and boost -> cruise phase
        Resetting allows us to clear out the accumulated error and control in PID,
        as cruise and boost phase has totally different behaviour.
        """
        self.alt_pid.reset()
        self.spd_pid.reset()













