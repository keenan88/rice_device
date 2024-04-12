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

        self.max_extension = 17600
        self.min_extension = 500
        self.threshold = 200

        self.positions = {
            "high": self.min_extension + self.threshold, 
            "above_wheels": 5000,
            "touching_farm": 8300, 
            "in_hole": 16500
            
        }

        self.targetPos = self.positions['high']
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

        # Validate given position
        # Validate that not in motion
        if goal_handle.request.desired_pos in self.positions.keys() and self.reachedPos:
            self.get_logger().info('Central shaft request in execution')

            self.reachedPos = False

            desired_pos = goal_handle.request.desired_pos
            self.targetPos = self.positions[desired_pos]

            feedback_msg = MoveLIAction.Feedback()
            feedback_msg.in_motion = True

            # Configure hardware to move central shaft in desired direction
            if self.pot.value < self.targetPos:
                if self.pot.value < self.max_extension and not self.button.is_pressed:
                    GPIO.output(self.in1,GPIO.HIGH)
                    GPIO.output(self.in2,GPIO.LOW)
                else:
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.LOW)

            elif self.pot.value > self.targetPos:
                if self.pot.value > self.min_extension:
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.HIGH)
                else:
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.LOW)
            else:
                GPIO.output(self.in1,GPIO.LOW)
                GPIO.output(self.in2,GPIO.LOW)

            while not self.reachedPos:
                
                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info('Central shaft position: %d, Button value: %d' % (self.pot.value, self.button.is_pressed))

                # Stop if position reached
                if abs(self.pot.value - self.targetPos) <= self.threshold: 
                    if desired_pos != "touching_farm": # Let the touching farm case run until button is engaged with farm
                        self.get_logger().info('Central shaft within threshold of target position')

                        self.reachedPos = True

                # Stop if we have button press
                elif self.button.is_pressed and self.targetPos >= self.pot.value: # Button pressed while going down
                    if desired_pos == "touching_farm": # This is the success case for touching the farm
                        self.reachedPos = True
                    else:
                        self.reachedPos = False # Any other case that ends in button press is a fail
                        break

                else: # Stop if past min/max extension
                    past_max = (self.pot.value >= self.max_extension - self.threshold and self.targetPos >= self.max_extension)
                    past_min = (self.pot.value <= self.min_extension + self.threshold and self.targetPos <= self.min_extension)
                    if past_max or past_min:
                        self.reachedPos = True
                        if past_min: self.get_logger().info('Central shaft at min extension')
                        if past_max: self.get_logger().info('Central shaft at max extension')
                        

            # Stop movement
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.LOW)

            result.movement_time_completed = self.reachedPos
            goal_handle.succeed()

        else:
            result.movement_time_completed = False
            goal_handle.abort()

        self.reachedPos = True # Not part of the return, just necessary so that the next call to the server is allowed in
        
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
