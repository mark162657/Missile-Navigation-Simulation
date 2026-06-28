class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, out_max: float, out_min: float):
        """
        Args:
            kp, ki, kd: weighting factor for P, I and D controller
        """
        if out_max > out_min:
            raise ValueError(f"out_min: {out_min} must be less than out_max: {out_max}.")
        self.Kp = float(kp)
        self.Ki = float(ki)
        self.Kd = float(kd)

        self.out_max = float(out_max)
        self.out_min = float(out_min)

        self.integral = 0.0 # I: integral controllers accumulate error over time (error * dt), unlike D and P
        self.prev_mea = None


    def update(self, error: float, measurement: float, dt: float) -> float:
        if dt <= 0.0: # when dt is broken (cannot be ≤ 0), we will only rely on P controller
            raise ValueError("dt must be greater than 0.0")

        # P: proportional: error * multiplier/weight factor = output
        p = error * self.Kp
        # I
        self.integral += error * dt
        i = self.Ki * self.integral

        # D







    def reset(self) -> None:
        """
        Reset integral and previous measurements when switching from: boost -> cruise phase
        """
        self.integral = 0.0
        self.prev_mea = None

    def _clamp(self, value):
        """
        Clamping prevent integrator windup
        Clamp when:
            - output is saturating
            - error is same sign as controller output
        """
