import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from time import sleep
from math import pi


class LinearActuator(Node):

    def __init__(self):
        super().__init__('Linear_Actuator')

        self.dutyCycle = 100

        # Pins
        self.in1 = 0
        self.in2 = 5
        self.right = 22
        self.center = 27
        self.left = 17

        # Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.setup(self.right,GPIO.OUT)
        GPIO.setup(self.center,GPIO.OUT)
        GPIO.setup(self.left,GPIO.OUT)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        self.rightLA = GPIO.PWM(self.right,1000)
        self.centerLA = GPIO.PWM(self.center,1000)
        self.leftLA = GPIO.PWM(self.left,1000)

        # Start duty cyle at 0%
        self.rightLA.start(0)
        self.centerLA.start(0)
        self.leftLA.start(0)

    def moveSideLA(self, moveDown):
        self.stopAll()
        self.rightLA.ChangeDutyCycle(self.dutyCycle)
        self.leftLA.ChangeDutyCycle(self.dutyCycle)
        if moveDown:
            GPIO.output(self.in1,GPIO.HIGH)
            GPIO.output(self.in2,GPIO.LOW)
        else:
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.HIGH)

    def moveCenterLA(self, moveDown):
        self.stopAll()
        self.centerLA.ChangeDutyCycle(self.dutyCycle)
        if moveDown:
            GPIO.output(self.in1,GPIO.HIGH)
            GPIO.output(self.in2,GPIO.LOW)
        else:
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.HIGH)
        
    def stopAll(self):
        self.rightLA.ChangeDutyCycle(0)
        self.centerLA.ChangeDutyCycle(0)
        self.leftLA.ChangeDutyCycle(0)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        
def main(args=None):
    rclpy.init(args=args)

    linear_actuator = LinearActuator()

    rclpy.spin(linear_actuator)

    linear_actuator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
