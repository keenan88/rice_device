import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from math import pi
import sys

class Locomotion(Node):

    def __init__(self, robot_behaviour = Robot_Behaviour.STOP.value):
        super().__init__('Locomotion')

         # TODO - change this to pull robot state from a param file instead of hardcoded
        self.robot_behaviour_state = robot_behaviour

        self.got_new_behaviour_state = False

        # TODO - tune these parameters
        self.high_speed_m_per_s = 0.10
        self.low_speed_m_per_s = 0.05
        self.start_speed_m_per_s = 0.01

        self.angular_speed_rad_per_s = 0

        self.robot_behaviour_state_subscriber = self.create_subscription(
            Int32, '/robot_behaviour_state', self.robot_behaviour_state_callback, 10
        )

        self.robot_speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_velocity_update_period_s = 0.25

        self.robot_velocity_timer = self.create_timer(
            self.robot_velocity_update_period_s, 
            self.robot_velocity_callback
        )
        
    def robot_behaviour_state_callback(self, msg: Int32):
        if msg != self.robot_behaviour_state:
            self.got_new_behaviour_state = True

        self.robot_behaviour_state = msg
        

    def robot_velocity_callback(self):
        robot_vel = Twist() # Initialize 0-velocity twist message

        if self.robot_behaviour_state.data == 0: # 
            pass
            
        elif self.robot_behaviour_state.data == 1: # ROBOT_BEHAVIOUR_RAMP_TO_FULL_SPEED
            robot_vel.linear.x = self.high_speed_m_per_s

        elif self.robot_behaviour_state.data == 2:
            robot_vel.linear.x = self.low_speed_m_per_s

        elif self.robot_behaviour_state.data == 3:
            pass

        elif self.robot_behaviour_state.data == 4:
            robot_vel.angular.z = self.angular_speed_rad_per_s

        if self.got_new_behaviour_state:
            self.got_new_behaviour_state = False
            self.get_logger().info(
                'Robot Velocity Changed to: Vx: %.2f, Vy: %.2f, Vz: %.2f' % 
                (robot_vel.linear.x, robot_vel.linear.y, robot_vel.linear.z) 
                + ' ' + 'Wx: %.2f, Wy: %.2f, Wz: %.2f' % (robot_vel.angular.x, robot_vel.angular.y, robot_vel.angular.z)
                + ' ' + 'Robot Behaviour State: %d' % self.robot_behaviour_state.data
            )

        self.robot_speed_publisher.publish(robot_vel)
    
def main(args=None):
    rclpy.init(args=args)

    locomotion = Locomotion()

    rclpy.spin(locomotion)

    locomotion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()