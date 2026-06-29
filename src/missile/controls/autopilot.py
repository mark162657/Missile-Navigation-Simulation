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
        Pass them into PID and we make the output into ControlInput

        Args:
            state: the MissileState
            target_alt: the target altitude we want to reach / maintain
            target_spd: the target speed
            lateral_accel_cmd: the lateral acceleration command calculated in guidance path_follower
            dt: delta time (time changes
        """

        curr_alt = state.est_alt
        curr_spd = state.get_speed()

        # handles error for PID (target - measurement)
        alt_error = target_alt - curr_alt
        spd_error = target_spd - curr_spd













