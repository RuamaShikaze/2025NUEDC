class PID:
    def __init__(self, p=0, i=0, d=0, setpoint=0):
        self.Kp, self.Ki, self.Kd = p, i, d
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def __call__(self, error):
        self.integral += error
        derivative = error - self.last_error
        output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.last_error = error
        return output
