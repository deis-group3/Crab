import time
from adafruit_motorkit import MotorKit

class MotorController:

    def __init__(self):
        self.kit = MotorKit()
        
        self.motor1 = self.kit.motor1
        self.motor2 = self.kit.motor4

    def drive(self, vel):
        speed1 = max(-1.0, min(1.0, vel[0]))
        speed2 = max(-1.0, min(1.0, vel[1]))

        self.motor1.throttle = speed1
        self.motor2.throttle = speed2
        