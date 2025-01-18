import time
from Utils.PIDModule import PIDController

class RobotPID:
    def __init__(self, gyro, left_wheel, right_wheel):
        self.gyro = gyro
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel 
        self.pid = PIDController(kp=3.0, ki=0.0, kd=1.0, setpoint=0)

    def resetAngles(self):
        self.gyro.reset_angle(0)
        self.left_wheel.reset_angle(0)
        self.right_wheel.reset_angle(0)
        time.sleep(0.1)

    def get_filtered_angle(self, samples=3):
        angles = []
        for _ in range(samples):
         angles.append(self.gyro.angle())
        return sum(angles) / len(angles)    
    
    def turn(self, turn_angle, stopMargin = 1, speed = 150):
        self.resetAngles()

        self.pid.setpoint = turn_angle

        left_dir = 0
        right_dir = 0

        if (turn_angle > 0):

            left_dir = 1
            right_dir = -1

        elif (turn_angle < 0):

            left_dir = -1
            right_dir = 1    
            

        while True:
            filter_angle = self.get_filtered_angle(samples=3)
            error = filter_angle - turn_angle

            if abs(error) < stopMargin:
                print("Robot has reached the desired angle")
                break

            correction = self.pid.compute(filter_angle)

            left_power = speed * left_dir + correction
            right_power = speed * right_dir - correction

            self.left_wheel.dc(left_power)
            self.right_wheel.dc(right_power)
        
        self.left_wheel.brake()
        self.right_wheel.brake()
        self.pid.reset()    

    def forward(self, distance, speed = 100):
        self.resetAngles()

        wheel_diameter = 56 # 5.6 cm in 56 mm
        circumference = wheel_diameter * 3.14

        degrees = (distance / circumference) * 360

        initial_heading = self.get_filtered_angle(samples=3)
        self.pid.setpoint = initial_heading

        while True:
            left_angle = self.left_wheel.angle()
            right_angle = self.right_wheel.angle()

            average_angle = (left_angle + right_angle) / 2

            if average_angle >= degrees:
                break

            c_angle = self.get_filtered_angle(samples=3)

            correction = self.pid.compute(c_angle)
            
            self.left_wheel.dc(speed - correction)
            self.right_wheel.dc(speed + correction)

        self.left_wheel.brake()
        self.right_wheel.brake()
        self.pid.reset() 




