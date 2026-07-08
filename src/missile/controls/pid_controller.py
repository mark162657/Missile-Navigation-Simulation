class PIDController:
    def __init__(
            self,
            kp: float,
            ki: float,
            kd: float,
            out_max: float,
            out_min: float
    ):
        """
        Setting up PID controller parameters
        Args:
            kp, ki, kd: weighting factor for P, I, and D controller
            out_max, out_min: the max and min output available (for e.g. max and min thrust level)
        """
        if out_max <= out_min:
            raise ValueError(f"out_min: {out_min} must be less than out_max: {out_max}.")

        self.Kp = float(kp)
        self.Ki = float(ki)
        self.Kd = float(kd)

        self.out_max = float(out_max)
        self.out_min = float(out_min)

        self.integral = 0.0 # I: integral controllers accumulate error over time (error * dt), unlike D and P
        self.prev_mea = None

    def update(self, error: float, measurement: float, dt: float) -> float:
        """
        The main part of PID controller, updating each P I and D value throughout the tick with new measurement
        Math formula: u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        Our PID controller doesn't need filter, as we took est locations, which was filtered by KF already.
        Integrator anti-windup check is also conducted:
            Clamp when:
                - output is saturating
                - error has the same sign as controller output
        Args:
            error: setpoint - measurement (target - measurement)
        """
        if dt <= 0.0: # when dt is broken (cannot be ≤ 0), we will only rely on P controller
            raise ValueError("dt must be greater than 0.0")

        # P: proportional: error * multiplier/weight factor = output
        p = error * self.Kp
        # I: accumulation of error * delta_time (error over time)
        self.integral += error * dt
        i = self.Ki * self.integral

        # D:
        if self.prev_mea is None:
            d = 0.0
        else:
            # Derivative based on MEASUREMENT (not error): avoids a spike when the
            # setpoint jumps. Negative sign makes it a brake against fast motion.
            d = - self.Kd * (measurement - self.prev_mea) / dt

        # updating previous measurement by newest measurement
        self.prev_mea = measurement

        # clamp (limit the output raw in range of out_min and out_max)
        raw = p + i + d
        output = self._clamp(raw)

        # anti-windup (two conditions met)
        over_max = raw > self.out_max and error > 0.0
        under_min = raw < self.out_min and error < 0.0

        # freeze / rollback the integrator
        if over_max or under_min:
            self.integral -= error * dt # revert back

        return output

    def reset(self) -> None:
        """
        Reset integral and previous measurements when switching from: boost -> cruise phase
        """
        self.integral = 0.0
        self.prev_mea = None

    def _clamp(self, value):
        """
        Clamping prevent integrator windup
        """
        return max(self.out_min, min(self.out_max, value))
