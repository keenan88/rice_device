import rclpy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
from rclpy.node import Node
from time import sleep
from math import pi
from std_msgs.msg import String
from li_interface.action import MoveLIAction
from rclpy.action import ActionServer


class LinearActuator(Node):

    def __init__(self):
        super().__init__('Linear_Actuator')

        

        self.side_LI_movement_time_elapsed_s = 0
        self.center_LI_movement_time_elapsed_s = 0

        self.side_LI_in_motion = False
        self.center_LI_in_motion = False

        # LA actions
        self._action_server = ActionServer(
            self,
            MoveLIAction,
            'move_side_lis',
            self.execute_callback
        )
        
        timer_period_s = 1 / 1000
        self.movement_time_length_s = 6
        self.side_ms = 0
        self.center_ms = 0
        self.la_move_time_s = 6

        # Timers to give feedback after HE's have moved
        self.sideTimer = self.create_timer(timer_period_s, self.increment_side_ms)
        self.centerTimer = self.create_timer(timer_period_s, self.increment_center_ms)
        
        self.centerTimer.cancel()

        self.isCenterMoving = False

        self.dutyCycle = 100
        
        # Pin Setup
        self.in1 = 0
        self.in2 = 5
        self.right = 22
        self.center = 27
        self.left = 17

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

    def increment_side_ms(self):
        #self.side_ms += 1
        pass

    def increment_center_ms(self):
        self.center_ms += 1

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Start movement
        self.rightLA.ChangeDutyCycle(self.dutyCycle)
        self.leftLA.ChangeDutyCycle(self.dutyCycle)

        if goal_handle.request.desired_pos == "down":
            GPIO.output(self.in1,GPIO.HIGH)
            GPIO.output(self.in2,GPIO.LOW)
        elif goal_handle.request.desired_pos == "up":
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.HIGH)

        result = MoveLIAction.Result()

        if goal_handle.request.desired_pos in ["down", "up"]:
            feedback_msg = MoveLIAction.Feedback()
            feedback_msg.in_motion = True

            start_time_s = self.get_clock().now().nanoseconds / 1e9
            curr_time_s = start_time_s
            print(start_time_s)

            while curr_time_s - start_time_s < self.la_move_time_s:
                goal_handle.publish_feedback(feedback_msg)
                curr_time_s = self.get_clock().now().nanoseconds / 1e9
                print(curr_time_s - start_time_s)
            
            result.movement_time_completed = True
            goal_handle.succeed()

            # Stop after movement
            self.rightLA.ChangeDutyCycle(0)
            self.leftLA.ChangeDutyCycle(0)

        else:
            result.movement_time_completed = False
            goal_handle.abort()
        
        return result

    def start_center_LA_move(self, moveDown):
        self.centerTimer.reset()
        self.stop_all()
        self.centerLA.ChangeDutyCycle(self.dutyCycle)

        if moveDown.data:
            GPIO.output(self.in1,GPIO.HIGH)
            GPIO.output(self.in2,GPIO.LOW)
        else:
            GPIO.output(self.in1,GPIO.LOW)
            GPIO.output(self.in2,GPIO.HIGH)
        
    def stop_all(self):
        self.rightLA.ChangeDutyCycle(0)
        self.leftLA.ChangeDutyCycle(0)
        self.centerLA.ChangeDutyCycle(0)

    
        
def main(args=None):
    rclpy.init(args=args)

    linear_actuator = LinearActuator()

    rclpy.spin(linear_actuator)

    linear_actuator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# button, move to 3 positions, reach position