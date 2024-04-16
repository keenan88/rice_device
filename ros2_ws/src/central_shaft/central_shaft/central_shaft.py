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

        # Pins
        self.in1 = 10
        self.in2 = 9
        self.laPin = 4
        self.buttonPin = 11

        self.max_extension = 17600
        self.min_extension = 500

        self.Kp = 0.0585 / 4

        self.positions = {
            "high": self.min_extension + 100, 
            "above_wheels": 5000,
            "touching_farm": 9000,
            "barely_in_hole": 9300,
            "in_hole": 16500,
            
        }

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

        duty_cycle = 0
        self.la.start(duty_cycle)

        # Pot/Button setup
        self.button = Button(self.buttonPin, pull_up=False)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)
        self.pot = AnalogIn(self.ads, ADS.P0)

    def central_shaft_cb(self, goal_handle):
        self.get_logger().info('Executing central shaft goal...')

        result = MoveLIAction.Result()

        desired_pos = goal_handle.request.desired_pos

        # Validate given position
        # Validate that not in motion
        if desired_pos in self.positions.keys() and self.reachedPos:
            self.get_logger().info('Central shaft position request in execution')

            self.reachedPos = False

            targetPos = self.positions[desired_pos]
            threshold = 100

            feedback_msg = MoveLIAction.Feedback()
            feedback_msg.in_motion = True            

            while not self.reachedPos:

                btn_val = self.button.is_pressed # Get once per loop, may change during loop
                pot_val = self.pot.value # Get pot val once per loop, since it will be changing small amounts throughout the loop
                err = pot_val - targetPos
                abs_err = abs(err)

                duty_cycle = self.Kp * abs_err
                if duty_cycle > 100: duty_cycle = 100 #upper saturatino
                if duty_cycle < 10: duty_cycle = 10 #lower saturation
                self.la.ChangeDutyCycle(duty_cycle)

                #self.get_logger().info('%d, %d, %d, %d %f' % (pot_val, targetPos, abs_err, btn_val, duty_cycle))

                # Necessary to keep direction in loop, in case of overshoot
                # Configure hardware to move central shaft in desired direction
                if pot_val < targetPos and pot_val < self.max_extension and not btn_val:
                    GPIO.output(self.in1,GPIO.HIGH)
                    GPIO.output(self.in2,GPIO.LOW)

                elif pot_val > targetPos and pot_val > self.min_extension:
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.HIGH)
                    
                else:
                    GPIO.output(self.in1,GPIO.LOW)
                    GPIO.output(self.in2,GPIO.LOW)
                
                if abs_err <= threshold or (btn_val and targetPos >= pot_val): 
                    self.get_logger().info('Central shaft within threshold of target position')
                    self.reachedPos = True

                else: # Stop if past min/max extension
                    past_max = (pot_val >= self.max_extension - threshold and targetPos >= self.max_extension)
                    past_min = (pot_val <= self.min_extension + threshold and targetPos <= self.min_extension)
                    if past_max or past_min:
                        self.reachedPos = False
                        if past_min: self.get_logger().info('Central shaft at min extension')
                        if past_max: self.get_logger().info('Central shaft at max extension')

            result.movement_time_completed = self.reachedPos
            goal_handle.succeed()

        elif desired_pos == "probing_for_hole" and self.reachedPos:

            self.get_logger().info('Central shaft probe request in execution')

            self.la.ChangeDutyCycle(100)
            loose_threshold = 250

            moving_down = False
            self.reachedPos = False

            targetPos = self.positions['touching_farm']

            while not self.reachedPos:

                btn_val = self.button.is_pressed # Get once per loop, may change during loop
                pot_val = self.pot.value # Get pot val once per loop, since it will be changing small amounts throughout the loop
                
                #abs_err = abs(pot_val - targetPos)

                #self.get_logger().info('%d, %d, %d' % (pot_val, targetPos, btn_val))

                if moving_down:
                    if pot_val < self.max_extension and not btn_val:
                        GPIO.output(self.in1,GPIO.HIGH)
                        GPIO.output(self.in2,GPIO.LOW)
                    else:
                        GPIO.output(self.in1,GPIO.LOW)
                        GPIO.output(self.in2,GPIO.LOW)

                elif not moving_down:
                    if pot_val > self.min_extension:
                        GPIO.output(self.in1,GPIO.LOW)
                        GPIO.output(self.in2,GPIO.HIGH)
                    else:
                        GPIO.output(self.in1,GPIO.LOW)
                        GPIO.output(self.in2,GPIO.LOW)
                
                if moving_down and btn_val:
                    self.get_logger().info('Hit farm, going back up')
                    moving_down = False

                elif moving_down and pot_val >= self.positions['barely_in_hole']:
                    self.get_logger().info('Made it barely into hole')
                    self.reachedPos = True
                    result.movement_time_completed = True
                    
                elif not moving_down and pot_val <= targetPos:
                    self.get_logger().info('Moving down')
                    moving_down = True  

            result.movement_time_completed = self.reachedPos
            goal_handle.succeed()

        else:
            self.get_logger().info('Central shaft position request not in execution')
            result.movement_time_completed = False
            goal_handle.abort()

        # Stop movement
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)

        #self.get_logger().info('%d, %d' % (abs_err, btn_val))
        #self.get_logger().info('pos: %d, btn: %d' % (pot_val, btn_val))

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
# Stop if we have button press
"""
elif btn_val and self.targetPos >= pot_val: # Button pressed while going down
    if desired_pos == "touching_farm": # This is the success case for touching the farm
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        self.reachedPos = True
    else:
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        self.reachedPos = False # Any other case that ends in button press is a fail
        break
"""