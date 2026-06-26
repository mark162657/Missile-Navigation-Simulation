class PIDController:
    def __init__(self, P: float, I: float, D: float, output_max: float, output_min: float):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.output_max = output_max
        self.output_min = output_min



