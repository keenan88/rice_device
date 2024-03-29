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
from li_interface.action import MoveLIAction
from rclpy.action import ActionServer


class CentralShaft(Node):

    def __init__(self):
        super().__init__('Central_Shaft')
        # Publishers
        self.buttonPublisher = self.create_publisher(Bool, '/central_shaft_button', 10)

        button_timer_period = 0.5
        self.timer = self.create_timer(button_timer_period, self.button_callback)

        self.central_shaft_server = ActionServer(
            self,
            MoveLIAction,
            'move_central_shaft',
            self.central_shaft_cb
        )

        # Setup
        self.dutyCycle = 25

        # Pins
        self.in1 = 10
        self.in2 = 9
        self.laPin = 4
        self.buttonPin = 11

        self.upperbound = 17600
        self.lowerbound = 500

        self.topPosition = 500
        self.aboveWheels = 5000
        self.farmPosition = 16000
        self.holePosition = 17000

        self.threshold = 200
        self.targetPos = self.topPosition
        self.reachedPos = True

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

    def central_shaft_cb(self, goal_handle):
        self.get_logger().info('Executing central shaft goal...')

        result = MoveLIAction.Result()

        if goal_handle.request.desired_pos in ["high", "touching_farm", "in_hole", "above_wheels"] and self.reachedPos:

            self.reachedPos = False

            desired_pos = goal_handle.request.desired_pos
            if (desired_pos == 'high'):
                self.targetPos = self.topPosition
                self.reachedPos = False
            elif (desired_pos == 'touching_farm'):
                self.targetPos = self.farmPosition
                self.reachedPos = False
            elif (desired_pos == 'in_hole'):
                self.targetPos = self.holePosition
                self.reachedPos = False
            elif (desired_pos == "above_wheels"):
                self.targetPos = self.aboveWheels
                self.reachedPos = False

            feedback_msg = MoveLIAction.Feedback()
            feedback_msg.in_motion = True

            while not self.reachedPos:
                
                goal_handle.publish_feedback(feedback_msg)

                print(self.pot.value, self.targetPos)

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
                    print("within threshold")
                    self.reachedPos = True
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.LOW)

                past_upper = (self.pot.value >= self.upperbound - self.threshold and self.targetPos >= self.upperbound)
                past_lower = (self.pot.value <= self.lowerbound + self.threshold and self.targetPos <= self.lowerbound)

                if past_upper or past_lower:
                    if past_upper: print("below upper")
                    if past_lower: print("below lower")
                    # print('limit reached', upperbound, lowerbound, button.is_pressed)
                    self.reachedPos = True
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.LOW)

            result.movement_time_completed = True
            goal_handle.succeed()

        else:
            result.movement_time_completed = False
            goal_handle.abort()
        
        return result

    def stop(self):
        self.la.ChangeDutyCycle(0)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)

    def button_callback(self):
        msg = Bool()
        msg.data = self.button.is_pressed
        self.buttonPublisher.publish(msg)



        
def main(args=None):
    rclpy.init(args=args)

    central_shaft = CentralShaft()

    rclpy.spin(central_shaft)

    central_shaft.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
