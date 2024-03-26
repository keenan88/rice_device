import rclpy
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
from gpiozero import Button
from time import sleep
from std_msgs.msg import Bool
from std_msgs.msg import String
from rclpy.node import Node


class CentralShaft(Node):

    def __init__(self):
        super().__init__('Central_Shaft')
        # Publishers
        self.buttonPublisher = self.create_publisher(Bool, '/central_shaft_button', 10)
        button_timer_period = 0.5
        self.timer = self.create_timer(button_timer_period, self.button_callback)

        self.centralShaftPublisher = self.create_publisher(Bool, '/central_shaft_status', 10)
        central_shaft_timer_period = 0.5
        self.timer = self.create_timer(central_shaft_timer_period, self.central_shaft_status_callback)

        # Subscribers
        self.subCommand = self.create_subscription(
            String, 
            '/central_shaft_command', 
            self.commandCallback, 
            10)
        self.subCommand  # prevent unused variable warning

        # Setup
        self.dutyCycle = 25

        # Main Loop
        control_timer_period = 0.001
        self.control_timer = self.create_timer(control_timer_period, self.moveLA)

        # Pins
        self.in1 = 10
        self.in2 = 9
        self.laPin = 4
        self.buttonPin = 11
        self.upperbound = 17600
        self.lowerbound = 500

        self.topPosition = 500
        self.farmPosition = 16000
        self.holePosition = 17000
        self.threshold = 200
        self.targetPos = self.topPosition
        self.reachedPos = False


        # Setup
        # Linear actuator setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.setup(self.laPin,GPIO.OUT)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        self.la=GPIO.PWM(self.laPin,1000)

        # Start duty cyle at 0%
        self.la.start(self.dutyCycle)

        # Pot/Button setup
        self.button = Button(self.buttonPin, pull_up=False)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)
        self.pot = AnalogIn(self.ads, ADS.P0)

    def button_callback(self):
        msg = Bool()
        msg.data = self.button.is_pressed
        self.buttonPublisher.publish(msg)

    def central_shaft_status_callback(self):
        msg = Bool()
        msg.data = self.reachedPos
        self.centralShaftPublisher.publish(msg)

    def commandCallback(self, msg):
        if (msg.data == 'high'):
            self.targetPos = self.topPosition
            self.reachedPos = False
        elif (msg.data == 'mid'):
            self.targetPos = self.farmPosition
            self.reachedPos = False
        elif (msg.data == 'low'):
            self.targetPos = self.holePosition
            self.reachedPos = False

    def moveLA(self):
        if self.pot.value < self.targetPos:
            if self.pot.value < self.upperbound and not self.button.is_pressed:
                GPIO.output(self.in1,GPIO.HIGH)
                GPIO.output(self.in2,GPIO.LOW)
            else:
                GPIO.output(self.in1,GPIO.LOW)
                GPIO.output(self.in2,GPIO.LOW)
        elif self.pot.value > self.targetPos:
            if self.pot.value > self.lowerbound:
                GPIO.output(self.in1,GPIO.LOW)
                GPIO.output(self.in2,GPIO.HIGH)
            else:
                GPIO.output(self.in1,GPIO.LOW)
                GPIO.output(self.in2,GPIO.LOW)
        else:
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.LOW)

        # Check if reached
        if abs(self.pot.value - self.targetPos) <= self.threshold:
            self.reachedPos = True
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.LOW)

        if self.pot.value > self.upperbound or self.pot.value < self.lowerbound:
            # print('limit reached', upperbound, lowerbound, button.is_pressed)
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.LOW)
        
    def stop(self):
        self.la.ChangeDutyCycle(0)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)



        
def main(args=None):
    rclpy.init(args=args)

    central_shaft = CentralShaft()

    rclpy.spin(central_shaft)

    central_shaft.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
