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
            target_speed: float,
            lateral_accel_cmd,
            dt: float
    ) -> ControlInput:
        """
        What guidance want + where we are now + how much time passed
        Args:
            state: the MissileState
            target_alt: the target altitude we want to reach / maintain
            target_speed: the target speed
            lateral_accel_cmd: the lateral acceleration command calculated in guidance path_follower
            dt: delta time (time changes
        """










