class PIDController:
    def __init__(self, P: float, I: float, D: float, out_max: float, out_min: float):
        """"""
        if out_max > out_min:
            raise ValueError(f"out_min: {out_min} must be less than out_max: {out_max}.")
        self.Kp = float(P)
        self.Ki = float(I)
        self.Kd = float(D)

        self.out_max = float(out_max)
        self.out_min = float(out_min)

        self.integral = 0.0 # I: integral controllers accumulate error over time (error * dt), unlike D and P
        self.prev_mea = None


    def update(self, error: float, measurement: float, dt: float) -> float:
        if dt <= 0.0: # when dt is broken (cannot be ≤ 0), we will only rely on P controller
            raise ValueError("dt must be greater than 0.0")

        # P
        # I
        # D
        






    def reset(self) -> None:
        """
        Reset integral and previous measurements when switching from: boost -> cruise phase
        """
        self.integral = 0.0
        self.prev_mea = None

