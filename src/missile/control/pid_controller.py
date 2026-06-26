class PIDController:
    def __init__(self, P: float, I: float, D: float):
        self.Kp = P
        self.Ki = I
        self.Kd = D

