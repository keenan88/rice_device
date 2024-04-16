import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from enum import Enum
import tf2_ros
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist
from li_interface.action import MoveLIAction
from rclpy.action import ActionClient
from time import sleep
from std_srvs.srv import Trigger


ROBOT_STATE_INIT_DO_NOTHING = Int32(data = 0)
ROBOT_STATE_INIT_ALL_UP = Int32(data = 1)
ROBOT_STATE_INIT_CENTRAL_SHAFT_UP = Int32(data = 2)
ROBOT_STATE_INIT_CENTRAL_SHAFT_DOWN = Int32(data = 3)
ROBOT_STATE_INIT_OUTSIDE_RAILS_DOWN = Int32(data = 4)
ROBOT_STATE_INIT_CENTRAL_SHAFT_UP_2 = Int32(data = 5)
ROBOT_STATE_INIT_INNER_RAIL_DOWN = Int32(data = 16)

DEBUG_CONVENVIENCE_STATE = Int32(data=18)


# LINEAR & LINEUP

ROBOT_STATE_FULL_SPEED = Int32(data = 6)
ROBOT_STATE_LOWER_CENTRAL_SHAFT_TO_FARM = Int32(data = 17)
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
    def __init__(self, robot_behaviour = DEBUG_CONVENVIENCE_STATE):
        super().__init__('Robot_Behaviour')
        
        

        # Publishers to various subsystems
        self.drive_publisher = self.create_publisher(String, '/robot_movement', 10)
        self.robot_zeroer = self.create_publisher(Int32, '/zero_robot', 10)

    
        # Action clients for flopper rails and central shaft
        self.side_li_action_client = ActionClient(self, MoveLIAction, 'move_side_lis')
        self.center_li_action_client = ActionClient(self, MoveLIAction, 'move_center_li')
        self.central_shaft_action_client = ActionClient(self, MoveLIAction, 'move_central_shaft')

        self.toggle_action_client = ActionClient(self, MoveLIAction, 'toggle_drive')
        self.toggle_action_sent = False
        self.toggle_succeeded = False
        
        self.human_input_subscriber = self.create_subscription(
            Int32, '/human_input', self.human_input_callback, 10
        )

        # Subscriber and publisher to flopper localization
        self.last_hex_flag_subscriber = self.create_subscription(Int32, 'last_hex_flag', self.update_last_hex_flag, 10)
        self.flopper_localizer_publisher = self.create_publisher(String, '/flopper_localizer_state', 10)
        
        # Locmotion states
        self.drive_speed_sent = False


        # States of flopper LAs
        self.center_LI_in_motion = False
        self.center_LI_motion_initiated = False

        self.side_LI_in_motion = False
        self.side_LI_motion_initiated = False

        # Central shaft states
        self.central_shaft_motion_initiated = False
        self.central_shaft_in_motion = False
        self.central_shaft_reached_goal = False


        self.human_input_received = False

        # States of localization
        self.within_hole_threshold = False
        self.within_final_hex_transition = False
        self.within_turn_completion_threshold = False
        self.flopper_localizer_state_sent = False

        # Start robot state LAST, after all services started
        self.robot_behaviour_state = robot_behaviour
        self.last_state = self.robot_behaviour_state
        self.behaviour_state_timer = self.create_timer(0.1, self.update_robot_state)

        self.get_logger().info('Robot behaviour initialization complete! Continuing...')

    # Toggle action client
    def send_toggle_action(self):
        self.get_logger().info('Sending toggle action')
        self.toggle_action_sent = True
        self.toggle_succeeded = False
        goal_msg = MoveLIAction.Goal()

        self.toggle_action_client.wait_for_server()
        self.toggle_goal_future = self.toggle_action_client.send_goal_async(goal_msg)
        self.toggle_goal_future.add_done_callback(self.toggle_response_cb)

    def toggle_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Toggle goal rejected')
            self.toggle_action_sent = False
            return

        self.get_logger().info('Toggle goal accepted')
        self.toggle_goal_future = goal_handle.get_result_async()
        self.toggle_goal_future.add_done_callback(self.toggle_result_cb)

    def toggle_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Toggle succeeded? {0}'.format(result.movement_time_completed))
        self.toggle_succeeded = result.movement_time_completed
        




    # Side Flopper Rail Action Client
    def send_side_LI_goal(self, desired_pos, movement_time_s = 6):
        self.get_logger().info('Side LA goal sent')
        goal_msg = MoveLIAction.Goal()
        goal_msg.desired_pos = desired_pos
        goal_msg.movement_time_s = movement_time_s

        self.side_LI_in_motion = True
        self.side_li_action_client.wait_for_server()
        self.send_goal_future = self.side_li_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.side_LI_response_cb)

    def side_LI_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Side LA goal rejected')
            return

        self.get_logger().info('Side LA goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.side_LI_result_cb)

    def side_LI_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Side LA movement succeeded? {0}'.format(result.movement_time_completed))
        self.side_LI_in_motion = not result.movement_time_completed

    # Center Flopper Rail Action Client
    def send_center_LI_goal(self, desired_pos, movement_time_s = 6):
        goal_msg = MoveLIAction.Goal()
        goal_msg.desired_pos = desired_pos
        goal_msg.movement_time_s = movement_time_s

        self.center_LI_in_motion = True
        self.center_li_action_client.wait_for_server()
        self.center_goal_future = self.center_li_action_client.send_goal_async(goal_msg)
        self.center_goal_future.add_done_callback(self.center_LI_response_cb)

    def center_LI_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Center LA Goal rejected')
            return

        self.get_logger().info('Center LA Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.center_LI_result_cb)

    def center_LI_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Center LA movement succeeded? {0}'.format(result.movement_time_completed))
        self.center_LI_in_motion = not result.movement_time_completed
    
    # Central Shaft Action Client
    def send_central_shaft_goal(self, desired_pos):
        goal_msg = MoveLIAction.Goal()
        goal_msg.desired_pos = desired_pos

        self.central_shaft_in_motion = True
        self.central_shaft_reached_goal = False
        self.central_shaft_action_client.wait_for_server()
        self.center_goal_future = self.central_shaft_action_client.send_goal_async(goal_msg)
        self.center_goal_future.add_done_callback(self.central_shaft_response_cb)

    def central_shaft_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Central shaft goal rejected')
            return

        self.get_logger().info('Central shaft goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.central_shaft_result_cb)

    def central_shaft_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Central shaft movement succeeded? {0}'.format(result.movement_time_completed))
        self.central_shaft_in_motion = False
        self.central_shaft_reached_goal = result.movement_time_completed


    def human_input_callback(self, msg: Int32):
        self.human_input_received = True

    def update_last_hex_flag(self, msg: Int32):
        self.within_final_hex_transition = 1


    

    def update_robot_state(self):

        if self.robot_behaviour_state != self.last_state:
            self.get_logger().info("Robot state updated to: %d" % (self.robot_behaviour_state.data))

        self.last_state = self.robot_behaviour_state

        if self.robot_behaviour_state == ROBOT_STATE_INIT_DO_NOTHING:

            if self.human_input_received:
                self.robot_behaviour_state = ROBOT_STATE_INIT_ALL_UP
                self.human_input_received = False

        elif self.robot_behaviour_state == DEBUG_CONVENVIENCE_STATE:
            


            """
            if not self.toggle_action_sent:
                self.send_toggle_action()

            if self.toggle_succeeded:
                self.toggle_action_sent = False
                self.get_logger().info('Toggle succeeded')
                while 1: pass
            """
    

            
           
            

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_ALL_UP:
            if not self.drive_speed_sent: # Brake motors
                self.drive_speed_sent = True
                drive_msg = String()
                drive_msg.data = "Stop"
                self.drive_publisher.publish(drive_msg) # Assumed instantaneous change in wheel velocity, no feedback necessary
            
            # Bring all flopper rails up
            if not self.side_LI_motion_initiated:
                self.side_LI_motion_initiated = True
                self.send_side_LI_goal("up")

            if not self.center_LI_motion_initiated:
                self.center_LI_motion_initiated = True
                self.send_center_LI_goal("up")
            
            if not self.side_LI_in_motion and not self.center_LI_in_motion:

                # Reset central shaft and LA state flags
                self.side_LI_motion_initiated = False
                self.center_LI_motion_initiated = False

                self.get_logger().info("Floppers Retracted, ready to get central shaft into hole")
                self.robot_behaviour_state = ROBOT_STATE_INIT_CENTRAL_SHAFT_UP


        elif self.robot_behaviour_state == ROBOT_STATE_INIT_CENTRAL_SHAFT_UP:

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("above_wheels")

            if not self.central_shaft_in_motion:
                self.central_shaft_motion_initiated = False

                if self.central_shaft_reached_goal:
                    self.robot_behaviour_state = ROBOT_STATE_INIT_CENTRAL_SHAFT_DOWN
                    self.get_logger().info("Central shaft retracted, ready to insert into hole") 
                else:
                    self.get_logger().info("Central shaft could not be retracted, trying again")
                

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_CENTRAL_SHAFT_DOWN:

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("in_hole")

            if not self.central_shaft_in_motion:     
                self.central_shaft_motion_initiated = False

                if self.central_shaft_reached_goal:
                    self.robot_behaviour_state = ROBOT_STATE_INIT_OUTSIDE_RAILS_DOWN
                    self.get_logger().info("Central shaft in hole, ready to lower floppers") 

                else:     
                    self.get_logger().info("Central shaft did not get into hole, retracting and trying again.") 
                    self.robot_behaviour_state = ROBOT_STATE_INIT_CENTRAL_SHAFT_UP

                    
        elif self.robot_behaviour_state == ROBOT_STATE_INIT_OUTSIDE_RAILS_DOWN:

            if not self.side_LI_motion_initiated:
                self.side_LI_motion_initiated = True
                self.send_side_LI_goal("down")

            if not self.side_LI_in_motion:
                self.side_LI_motion_initiated = False

                self.robot_behaviour_state = ROBOT_STATE_INIT_CENTRAL_SHAFT_UP_2

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_CENTRAL_SHAFT_UP_2:

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("above_wheels")

            if not self.central_shaft_in_motion:
                self.central_shaft_motion_initiated = False

                if self.central_shaft_reached_goal:
                    self.robot_behaviour_state = ROBOT_STATE_INIT_INNER_RAIL_DOWN
                    self.get_logger().info("Central shaft retracted, ready to proceed autonomously") 
                else:
                    pass # Try retracting again

        elif self.robot_behaviour_state == ROBOT_STATE_INIT_INNER_RAIL_DOWN:

            if not self.center_LI_motion_initiated:
                self.center_LI_motion_initiated = True
                self.send_center_LI_goal("down", 3)

            if not self.center_LI_in_motion:
                self.center_LI_motion_initiated = False
                self.robot_behaviour_state = ROBOT_STATE_FULL_SPEED



        elif self.robot_behaviour_state == ROBOT_STATE_FULL_SPEED:

            if not self.center_LI_motion_initiated:
                self.get_logger().info('Center LA motion sent')
                self.center_LI_motion_initiated = True
                self.send_center_LI_goal("down", 3)

            if not self.drive_speed_sent:
                self.get_logger().info('Drive speed sent')
                self.drive_speed_sent = True
                speed_msg = String()
                speed_msg.data = "low_speed"
                self.drive_publisher.publish(speed_msg)

            if not self.flopper_localizer_state_sent:
                self.flopper_localizer_state_sent = True
                start_flopper_localization_msg = String()
                start_flopper_localization_msg.data = "on"
                self.flopper_localizer_publisher.publish(start_flopper_localization_msg)
                self.get_logger().info('Flopper localizer state sent')

            if self.within_final_hex_transition:
                speed_msg = String()
                speed_msg.data = "stop"
                self.drive_publisher.publish(speed_msg)

                self.center_LI_motion_initiated = False
                self.within_final_hex_transition = False
                self.drive_speed_sent = False
                self.flopper_localizer_state_sent = False
                self.robot_behaviour_state = ROBOT_STATE_LOWER_CENTRAL_SHAFT_TO_FARM

        elif self.robot_behaviour_state == ROBOT_STATE_LOWER_CENTRAL_SHAFT_TO_FARM:

            if not self.drive_speed_sent:
                self.drive_speed_sent = True
                speed_msg = String()
                speed_msg.data = "stop"
                self.drive_publisher.publish(speed_msg) 

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("touching_farm")
            
            if self.central_shaft_reached_goal:
                self.central_shaft_reached_goal = False
                self.central_shaft_motion_initiated = False
                self.robot_behaviour_state = ROBOT_STATE_LINEUP_CENTRAL_SHAFT
                self.drive_speed_sent = False

        elif self.robot_behaviour_state == ROBOT_STATE_LINEUP_CENTRAL_SHAFT:
            
            if not self.drive_speed_sent:
                self.drive_speed_sent = True
                speed_msg = String()
                speed_msg.data = "low_speed"
                self.drive_publisher.publish(speed_msg) 

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("probing_for_hole")
            
            if self.central_shaft_reached_goal:
                self.central_shaft_reached_goal = False
                self.central_shaft_motion_initiated = False
                self.drive_speed_sent = False
                self.robot_behaviour_state = ROBOT_STATE_STOP
                self.central_shaft_reached_goal = False

                


        elif self.robot_behaviour_state == ROBOT_STATE_STOP:
            if not self.drive_speed_sent:
                self.drive_speed_sent = True
                speed_msg = String()
                speed_msg.data = "stop"
                self.drive_publisher.publish(speed_msg)   

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("in_hole")
            
            if self.central_shaft_reached_goal:
                self.central_shaft_reached_goal = False
                self.central_shaft_motion_initiated = False
                self.robot_behaviour_state = ROBOT_STATE_LINEUP_CENTRAL_SHAFT
                self.drive_speed_sent = False         

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
                self.within_turn_completion_threshold = False
                self.robot_behaviour_state = ROBOT_STATE_TURN_180
                self.side_LI_reached_goal = False  

        elif self.robot_behaviour_state == -1:

            if not self.drive_speed_sent:
                self.drive_speed_sent = True
                speed_msg = String()
                speed_msg.data = "stop"
                self.drive_publisher.publish(speed_msg) 

            if not self.central_shaft_motion_initiated:
                self.central_shaft_motion_initiated = True
                self.send_central_shaft_goal("in_hole")

            # Bring all flopper rails up
            if not self.side_LI_motion_initiated:
                self.side_LI_motion_initiated = True
                self.send_side_LI_goal("up")

            if not self.center_LI_motion_initiated:
                self.center_LI_motion_initiated = True
                self.send_center_LI_goal("up")
                            
            if self.central_shaft_reached_goal:
                if not self.side_LI_in_motion and not self.center_LI_in_motion:
                    self.get_logger().info("Shaft extended floppers retracted, ready to turn")
                    self.side_LI_motion_initiated = False
                    self.center_LI_motion_initiated = False

                    self.central_shaft_reached_goal = False
                    self.central_shaft_motion_initiated = False
                    self.drive_speed_sent = False

                    self.robot_behaviour_state = ROBOT_STATE_TURN_180

                    while 1: pass

        elif self.robot_behaviour_state == ROBOT_STATE_TURN_180:
            drive_msg = String()
            drive_msg.data = "turn"

            self.drive_publisher.publish(drive_msg)

            robot_quat = self.robot_odom.pose.pose.orientation
            robot_euler_orientation = tf.transformations.euler_from_quaternion([robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w])

            if self.within_turn_completion_threshold:
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
