import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from math import pi
import sys
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from time import sleep
from li_interface.action import MoveLIAction # Scuff.
from rclpy.action import ActionServer

class Locomotion(Node):

    def __init__(self):
        super().__init__('Locomotion')

        self.got_new_behaviour_state = False

        self.high_speed_m_per_s = 0.05
        self.low_speed_m_per_s = 0.01
        self.toggle_ang_vel = 0.3
        self.current_velocity = 0
        self.toggle_motion_time_s = 0.25
        self.forward_disp = 0
        self.local_localization_enabled = 0

        self.current_vel_prof = "stop"

        self.turning_speed_rad_per_s = 2 * 3.1415 / 10

        self.robot_velocity_subscriber = self.create_subscription(
            String, '/robot_movement', self.set_robot_velocity, 10
        )

        self.robot_speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.local_localiztaion_period_s = 0.1
        self.local_localization_timer = self.create_timer(self.local_localiztaion_period_s, self.local_localization_cb)

        self.toggle_action_server = ActionServer(
            self,
            MoveLIAction,
            'toggle_drive',
            self.toggle_callback
        )
        

    def set_robot_velocity(self, vel_str: String):

            robot_vel = Twist() # Initialize 0-velocity twist message
            
            if vel_str.data == "stop":
                pass

            elif vel_str.data == "full_speed":
                robot_vel.linear.x = self.high_speed_m_per_s
                
                
            elif vel_str.data == "low_speed":
                robot_vel.linear.x = self.low_speed_m_per_s

            elif vel_str.data == "turn":
                robot_vel.angular.z = self.turning_speed_rad_per_s

            elif vel_str.data == "CW_toggle":
                robot_vel.angular.z = self.toggle_ang_vel

            elif vel_str.data == "CCW_toggle":
                robot_vel.angular.z = -self.toggle_ang_vel

            else:
                pass

            if vel_str.data == "full_speed":
                self.forward_disp = 0
                self.local_localization_enabled = 1
            else:
                self.local_localization_enabled = 0

            if vel_str.data not in ['CW_toggle', 'CCW_toggle']:
                self.current_velocity = robot_vel.linear.x
                self.current_vel_prof = vel_str

            """
            self.get_logger().info(
                'Robot Velocity Changed to: Vx: %.2f, Vy: %.2f, Vz: %.2f' % 
                (robot_vel.linear.x, robot_vel.linear.y, robot_vel.linear.z) 
                + ' ' + 'Wx: %.2f, Wy: %.2f, Wz: %.2f' % (robot_vel.angular.x, robot_vel.angular.y, robot_vel.angular.z)
            )
            """

            self.robot_speed_publisher.publish(robot_vel)

    def local_localization_cb(self):
        self.forward_disp += self.current_velocity * self.local_localiztaion_period_s

        if self.forward_disp >= 0.125 and self.local_localization_enabled:
            #stop_movement = String()
            #stop_movement.data = "stop"
            #self.set_robot_velocity(stop_movement)

            toggle_states = ['CW_toggle', 'CCW_toggle', 'CW_toggle', 'CCW_toggle']

            toggle_state_idx = 0

            while toggle_state_idx < len(toggle_states):
                vel_str = String()
                vel_str.data = toggle_states[toggle_state_idx]
                self.set_robot_velocity(vel_str)

                sleep(self.toggle_motion_time_s)
                toggle_state_idx += 1
            
            self.forward_disp = 0
            self.get_logger().info('Current velocity profile %s' % self.current_vel_prof.data)
            self.set_robot_velocity(self.current_vel_prof)


    def toggle_callback(self, goal_handle):
        self.get_logger().info('Executing toggle callback...')

        # Start movement
        stop_movement = String()
        stop_movement.data = "stop"
        self.set_robot_velocity(stop_movement)

        toggle_states = ['CW_toggle', 'CCW_toggle', 'CW_toggle', 'CCW_toggle']

        toggle_state_idx = 0

        while toggle_state_idx < len(toggle_states):
            vel_str = String()
            vel_str.data = toggle_states[toggle_state_idx]
            self.set_robot_velocity(vel_str)

            sleep(self.toggle_motion_time_s)
            toggle_state_idx += 1
        
        self.set_robot_velocity(stop_movement)

        result = MoveLIAction.Result()

        result.movement_time_completed = True
        goal_handle.succeed()
        
        return result

        
        

    
def main(args=None):
    rclpy.init(args=args)

    locomotion = Locomotion()

    rclpy.spin(locomotion)

    locomotion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()