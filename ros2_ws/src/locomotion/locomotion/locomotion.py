import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from math import pi

class Locomotion(Node):

    def __init__(self):
        super().__init__('Locomotion')

        self.robot_behaviour_state = Int32(data = 0) # TODO - change this to pull robot state from a param file instead of hardcoded
        self.new_behaviour_state = False

        # TODO - tune these parameters
        self.high_speed_m_per_s = 0.15
        self.low_speed_m_per_s = 0.05
        self.angular_speed_rad_per_s = 2*pi / 5

        self.robot_behaviour_state_subscriber = self.create_subscription(
            Int32, '/robot_behaviour_state', self.robot_behaviour_state_callback, 10
        )

        self.robot_speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_velocity_update_period_s = 1/60
        self.robot_velocity_timer = self.create_timer(
            self.robot_velocity_update_period_s, 
            self.robot_velocity_callback
        )
        
    def robot_behaviour_state_callback(self, msg: Int32):
        if msg != self.robot_behaviour_state:
            self.new_behaviour_state = True
        self.robot_behaviour_state = msg
        

    def robot_velocity_callback(self):
        robot_vel = Twist()

        if self.robot_behaviour_state.data == 0:
            pass
            
        elif self.robot_behaviour_state.data == 1:
            robot_vel.linear.x = self.high_speed_m_per_s

        elif self.robot_behaviour_state.data == 2:
            robot_vel.linear.x = self.low_speed_m_per_s

        elif self.robot_behaviour_state.data == 3:
            pass

        elif self.robot_behaviour_state.data == 4:
            robot_vel.linear.z = self.angular_speed_rad_per_s

        if self.new_behaviour_state:
            self.new_behaviour_state = False
            self.get_logger().info(
                'Robot Velocity Changed to: Vx: %f, Vy: %f, Vz: %f' % 
                (round(robot_vel.linear.x, 2), round(robot_vel.linear.y, 2), round(robot_vel.linear.z, 2)) 
                + ' ' + 'Wx: %f, Wy: %f, Wz: %f' % (round(robot_vel.angular.x, 2), round(robot_vel.angular.y, 2), round(robot_vel.angular.z, 2))
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