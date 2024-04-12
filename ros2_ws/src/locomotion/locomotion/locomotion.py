import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from math import pi
import sys
from nav_msgs.msg import Odometry


class Locomotion(Node):

    def __init__(self):
        super().__init__('Locomotion')

        self.got_new_behaviour_state = False

        self.high_speed_m_per_s = 0.125
        self.low_speed_m_per_s = 0.01

        self.turning_speed_rad_per_s = 2 * 3.1415 / 10

        self.robot_velocity_subscriber = self.create_subscription(
            String, '/robot_movement', self.set_robot_velocity, 10
        )

        self.robot_speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

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
                
            else:
                pass

            self.get_logger().info(
                'Robot Velocity Changed to: Vx: %.2f, Vy: %.2f, Vz: %.2f' % 
                (robot_vel.linear.x, robot_vel.linear.y, robot_vel.linear.z) 
                + ' ' + 'Wx: %.2f, Wy: %.2f, Wz: %.2f' % (robot_vel.angular.x, robot_vel.angular.y, robot_vel.angular.z)
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