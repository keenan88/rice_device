import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from enum import Enum
import tf2_ros
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist
from li_interface.action import MoveLIAction
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry

ROBOT_STATE_INIT_DO_NOTHING = Int32(data = 16)
ROBOT_STATE_INIT_ALL_UP = Int32(data = 0)
ROBOT_STATE_INIT_CENTRAL_SHAFT_DOWN = Int32(data = 1)
ROBOT_STATE_INIT_OUTSIDE_RAILS_DOWN = Int32(data = 2)
ROBOT_STATE_INIT_CENTRAL_SHAFT_UP = Int32(data = 3)
ROBOT_STATE_INIT_CENTER_RAIL_DOWN = Int32(data = 4)

ROBOT_STATE_INITIALIZE = Int32(data = 5)

# LINEAR & LINEUP

ROBOT_STATE_FULL_SPEED = Int32(data = 6)
ROBOT_STATE_LINEUP_CENTRAL_SHAFT = Int32(data = 7)
ROBOT_STATE_STOP = Int32(data = 8)

# TURN PREP & TURN
ROBOT_STATE_RETRACT_CENTRAL_FLOPPER_RAIL = Int32(data = 9)
ROBOT_STATE_EXTEND_CENTRAL_SHAFT = Int32(data = 10)
ROBOT_STATE_RETRACT_SIDE_FLOPPER_RAILS = Int32(data = 11)
ROBOT_STATE_TURN_180 = Int32(data = 12)

# LINEAR PREP
ROBOT_STATE_EXTEND_SIDE_FLOPPER_RAILS = Int32(data = 13)
ROBOT_STATE_RETRACT_CENTRAL_SHAFT = Int32(data = 14)
ROBOT_STATE_EXTEND_CENTRAL_FLOPPER_RAIL = Int32(data = 15)


class Robot_Behaviour(Node):
    def __init__(self, robot_behaviour = ROBOT_STATE_INIT_ALL_UP):
        super().__init__('Robot_Behaviour')
        
        self.robot_behaviour_state = robot_behaviour

        self.behaviour_state_timer = self.create_timer(1, self.update_robot_state)

        # Publishers to various subsystems
        self.drive_publisher = self.create_publisher(String, '/robot_movement', 10)
        self.robot_zeroer = self.create_publisher(Int32, '/zero_robot', 10)

        # Subscribers to various subsystems feedback
        self.central_shaft_subscriber = self.create_subscription(
            Int32, '/central_shaft_status', self.central_shaft_callback, 10
        )   

        self.side_li_action_client = ActionClient(self, MoveLIAction, 'move_side_lis')

        self.center_li_action_client = ActionClient(self, MoveLIAction, 'move_center_li')
        

        self.human_input_subscriber = self.create_subscription(
            Int32, '/human_input', self.human_input_callback, 10
        )

        self.robot_odom_subscriber = self.create_subscription(
            Odometry, '/robot_odom', self.robot_odom_callback, 10
        )

        self.center_LI_in_motion = False
        self.center_LI_motion_initiated = False

        self.side_LI_in_motion = False
        self.side_LI_motion_initiated = False

        self.central_shaft_reached_goal = False
        self.human_input_received = False

        self.robot_odom = Odometry()

    def send_side_LI_goal(self, desired_pos):
        goal_msg = MoveLIAction.Goal()
        goal_msg.desired_pos = desired_pos

        self.side_LI_in_motion = True

        self.side_li_action_client.wait_for_server()

        self.send_goal_future = self.side_li_action_client.send_goal_async(goal_msg, feedback_callback = self.side_LI_feedback_cb)

        self.send_goal_future.add_done_callback(self.side_LI_response_cb)

    def side_LI_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Side Goal rejected :(')
            return

        self.get_logger().info('Side Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.side_LI_result_cb)

    def side_LI_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Side Result: {0}'.format(result.movement_time_completed))

        self.side_LI_in_motion = not result.movement_time_completed

    def side_LI_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.side_LI_in_motion = feedback.in_motion

    def send_center_LI_goal(self, desired_pos):
        goal_msg = MoveLIAction.Goal()
        goal_msg.desired_pos = desired_pos

        self.center_LI_in_motion = True

        self.center_li_action_client.wait_for_server()

        self.center_goal_future = self.center_li_action_client.send_goal_async(goal_msg, feedback_callback = self.center_LI_feedback_cb)

        self.center_goal_future.add_done_callback(self.center_LI_response_cb)

    def center_LI_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Center Goal rejected :(')
            return

        self.get_logger().info('Center Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.center_LI_result_cb)

    def center_LI_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Center Result: {0}'.format(result.movement_time_completed))

        self.center_LI_in_motion = not result.movement_time_completed

    def center_LI_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.center_LI_in_motion = feedback.in_motion

    def tf_callback(msg):
    # Callback function to handle TF messages
        print("Received TF message:")
        print(msg)

    def human_input_callback(self, msg: Int32):
        self.human_input_received = True

    def central_shaft_callback(self, shaft_reached_goal: Bool):
        self.central_shaft_reached_goal = shaft_reached_goal.data

    def robot_odom_callback(self, msg: Odometry):
        self.robot_odom = msg

    def update_robot_state(self):

        if self.robot_behaviour_state == ROBOT_STATE_INIT_DO_NOTHING:

            if self.human_input_received:
                self.robot_behaviour_state = ROBOT_STATE_INIT_ALL_UP
                self.human_input_received = False

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_ALL_UP:
            # Brake motors
            drive_msg = String()
            drive_msg.data = "Stop"
            self.drive_publisher.publish(drive_msg) # Assumed instantaneous change in wheel velocity

            # Bring all flopper rails up
            if not self.side_LI_motion_initiated:
                self.side_LI_motion_initiated = True
                self.send_side_LI_goal("up")

            if not self.center_LI_motion_initiated:
                self.center_LI_motion_initiated = True
                self.send_center_LI_goal("up")
        

            # Bring central shaft up
            #shaft_msg = String()
            #shaft_msg.data = "high"
            #self.central_shaft_publisher.publish(shaft_msg)

            # Wait until all subsystems confirm complete movement
            if not self.side_LI_in_motion:# and not self.center_LI_in_motion:
                #if self.central_shaft_reached_goal and self.human_input_received:
                print("DONE")
                self.side_LI_motion_initiated = False
                #self.robot_behaviour_state = ROBOT_STATE_INIT_CENTRAL_SHAFT_DOWN
                while 1: pass
                self.human_input_received = False
                self.central_shaft_reached_goal = False
                self.center_LI_reached_goal = False
                self.side_LI_reached_goal = False

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_CENTRAL_SHAFT_DOWN:
            
            shaft_msg = String()
            shaft_msg.data = "low"
            self.central_shaft_publisher.publish(shaft_msg)

            if self.human_input_received and self.central_shaft_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_INIT_OUTSIDE_RAILS_DOWN
                self.human_input_received = False
                self.central_shaft_reached_goal = False


        elif self.robot_behaviour_state == ROBOT_STATE_INIT_OUTSIDE_RAILS_DOWN:

            moveDown = False
            rail_msg = Bool()
            rail_msg.data = moveDown
            self.side_LI_publisher.publish(rail_msg)

            if self.human_input_received and self.side_LI_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_INIT_CENTRAL_SHAFT_UP
                self.human_input_received = False
                self.side_LI_reached_goal = False
            
        elif self.robot_behaviour_state == ROBOT_STATE_INIT_CENTRAL_SHAFT_UP:

            shaft_msg = String()
            shaft_msg.data = "high"
            self.central_shaft_publisher.publish(shaft_msg)
                
            if self.human_input_received and self.central_shaft_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_INIT_CENTER_RAIL_DOWN
                self.human_input_received = False
                self.central_shaft_reached_goal = False

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_CENTER_RAIL_DOWN:
            
            moveDown = False
            rail_msg = Bool()
            rail_msg.data = moveDown
            self.center_LI_publisher.publish(rail_msg)

            if self.human_input_received and self.center_LI_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_INITIALIZE
                self.human_input_received = False
                self.center_LI_reached_goal = False
            
        
        elif self.robot_behaviour_state == ROBOT_STATE_INITIALIZE:
            trigger_zero_msg = Int32()
            self.robot_zeroer.publish(trigger_zero_msg) # Zero robot

            if self.human_input_received:
                self.robot_behaviour_state = ROBOT_STATE_FULL_SPEED
                self.human_input_received = False

        elif self.robot_behaviour_state == ROBOT_STATE_FULL_SPEED:
            speed_msg = String()
            speed_msg.data = "full_speed"
            self.drive_publisher.publish(speed_msg)
            
            if self.robot_odom.pose.pose.position.x > 0.45: # Within 15 cm of pivot hole
                self.robot_behaviour_state = ROBOT_STATE_LINEUP_CENTRAL_SHAFT
            

        elif self.robot_behaviour_state == ROBOT_STATE_LINEUP_CENTRAL_SHAFT:
            speed_msg = String()
            speed_msg. data = "slow"
            self.drive_publisher.publish(speed_msg) 
            
            if self.robot_odom.pose.pose.position.x > 0.6 - 2 / 1000: # Within 2mm of pivot hole
                self.robot_behaviour_state = ROBOT_STATE_LINEUP_CENTRAL_SHAFT

        elif self.robot_behaviour_state == ROBOT_STATE_STOP:
            speed_msg = String()
            speed_msg. data = "stop"
            self.drive_publisher.publish(speed_msg)
            self.robot_behaviour_state == ROBOT_STATE_STOP

        elif self.robot_behaviour_state == ROBOT_STATE_RETRACT_CENTRAL_FLOPPER_RAIL:
            moveDown = False
            rail_msg = Bool()
            rail_msg.data = moveDown
            self.center_LI_publisher.publish(rail_msg)

            if self.center_LI_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_EXTEND_CENTRAL_SHAFT
                self.center_LI_reached_goal = False
            
        elif self.robot_behaviour_state == ROBOT_STATE_EXTEND_CENTRAL_SHAFT:
            shaft_msg = String()
            shaft_msg.data = "high"
            self.central_shaft_publisher.publish(shaft_msg)
                
            if self.central_shaft_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_RETRACT_SIDE_FLOPPER_RAILS
                self.central_shaft_reached_goal = False

        elif self.robot_behaviour_state == ROBOT_STATE_RETRACT_SIDE_FLOPPER_RAILS:  
            
            moveDown = False
            rail_msg = Bool()
            rail_msg.data = moveDown
            self.side_LI_publisher.publish(rail_msg)

            if self.side_LI_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_TURN_180
                self.side_LI_reached_goal = False  

        elif self.robot_behaviour_state == ROBOT_STATE_TURN_180:
            drive_msg = String()
            drive_msg.data = "turn"
            self.drive_publisher.publish(drive_msg)

            robot_quat = self.robot_odom.pose.pose.orientation
            robot_euler_orientation = tf.transformations.euler_from_quaternion([robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w])

            if robot_euler_orientation[2] > 3.14:
                self.robot_behaviour_state = ROBOT_STATE_EXTEND_SIDE_FLOPPER_RAILS

        elif self.robot_behaviour_state == ROBOT_STATE_EXTEND_SIDE_FLOPPER_RAILS:
            self.drive_publisher.publish(String("stop"))

            moveDown = True
            rail_msg = Bool()
            rail_msg.data = moveDown
            self.side_LI_publisher.publish(rail_msg)

            if self.side_LI_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_RETRACT_CENTRAL_SHAFT
                self.side_LI_reached_goal = False  

            pass

        elif self.robot_behaviour_state == ROBOT_STATE_RETRACT_CENTRAL_SHAFT:
            
            shaft_msg = String()
            shaft_msg.data = "high"
            self.central_shaft_publisher.publish(shaft_msg)
                
            if self.central_shaft_reached_goal:
                self.robot_behaviour_state = ROBOT_STATE_EXTEND_CENTRAL_FLOPPER_RAIL
                self.central_shaft_reached_goal = False

        elif self.robot_behaviour_state == ROBOT_STATE_EXTEND_CENTRAL_FLOPPER_RAIL:
            
            moveDown = True
            rail_msg = Bool()
            rail_msg.data = moveDown
            self.center_LI_publisher.publish(rail_msg)

            if self.center_LI_reached_goal:
                self.robot_zeroer.publish(trigger_zero_msg) # Zero robot
                self.robot_behaviour_state = ROBOT_STATE_INITIALIZE
                self.center_LI_reached_goal = False



def main(args=None):
    rclpy.init(args=args)
    robot_behaviour = Robot_Behaviour()
    rclpy.spin(robot_behaviour)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
