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

        self.movement_time_length_s = 6

        self.side_LI_movement_time_elapsed_s = 0
        self.center_LI_movement_time_elapsed_s = 0

        self.side_LI_in_motion = False
        self.center_LI_in_motion = False

        timer_period_s = 1

        # LA actions
        self._action_server = ActionServer(
            self,
            MoveLIAction,
            'move_side_lis',
            self.execute_callback
        )
        
        # Timers to give feedback after HE's have moved
        self.sideTimer = self.create_timer(timer_period_s, self.stop_all)
        self.centerTimer = self.create_timer(timer_period_s, self.stop_all)
        
        self.centerTimer.cancel()
        self.sideTimer.cancel()

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

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        result = MoveLIAction.Result()
        

        feedback_msg = MoveLIAction.Feedback()
        feedback_msg.in_motion = True

        for i in range(100):
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Sending feedback')

        result.movement_time_completed = True
        goal_handle.succeed()
        
        return result


    def start_center_LA_move(self, moveDown):
        self.centerTimer.reset()
        self.stop_all()
        self.centerLA.ChangeDutyCycle(self.dutyCycle) # Can these be moved to the constructor?

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