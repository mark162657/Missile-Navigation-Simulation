import math

from dataclasses import dataclass
from missile.controls.pid_controller import PIDController

@dataclass
class BoostGuidanceSpecs:
    """
    Constants for boost guidance pitch-over.

    Args:
        gamma_launch: flight path angle at launch
        gamma_handoff: flight path angle at handoff
        H_pitch: level-off once h_pitch meter is climbed
        v_min: below the speed we will hold vertical angle
        theta_rate_max: maximum pitch rate (rad/s)
        kp, ki, kd: PID controller factors
    """
    gamma_launch: float
    gamma_handoff: float

    H_pitch: float = 200.0
    v_min: float = 15.0
    theta_rate_max: float = math.radians(40.0)

    # PID controller factors
    kp: float = 0.5
    ki: float = 0.05
    kd: float = 0.0
    alpha_limit: float= math.radians(20.0)

class BoostGuidance:
    """
    The entire boost trajectory will be modified and based on 3-dof.

    """

    def __init__(self, specs: BoostGuidanceSpecs):
        self.specs = specs if specs is not None else BoostGuidanceSpecs()
        self.pid_pitch = PIDController(
            kp=self.specs.kp,
            ki=self.specs.ki,
            kd=self.specs.kd,
            out_max=self.specs.alpha_limit,
            out_min=self.specs.alpha_limit
        )
        self.h_launch: float
        self.prev_theta: float = self.param.gamma_launch

    def pitch_command(self, h: float, v: float, gamma: float, dt: float):
        """
        Formula:
            - γ = arctan(ROC, TAS):
            - θ_cmd = γ_ref + PID(γ_ref - γ)"
        """
        if self.h_launch is None:
            self.h



    def _progress(self):
        pass

    def _smooth_step(self):
        pass