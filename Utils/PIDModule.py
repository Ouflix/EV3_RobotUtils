class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0

    def compute(self, measurement):
        error = self.setpoint - measurement
        self.integral += error
        derivative = error - self.last_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return output

    def reset(self):
        self.integral = 0
        self.last_error = 0