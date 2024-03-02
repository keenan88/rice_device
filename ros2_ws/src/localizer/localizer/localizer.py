import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class Localizer(Node):

    def __init__(self):
        super().__init__('Localizer')

        self.wheel_odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.set_motors_degrees_per_s, 10)
        
        # TODO - read from flopper odometry once implemented
        #self.flopper_odom_sub = self.create_subscription(Odometry, '/odom_flopper', self.record_flopper_odom, 10)

        self.imu_sub = self.create_subscription(Imu, '/bno055/imu', self.record_imu_odom, 10)

        # TODO - implement custom type for button press with time stamp, or highjack an existing type
        #self.central_shaft_contact_sub = self.create_subscription(Imu, '/bno055/imu', self.set_motors_degrees_per_s, 10)
        
        self.positioning_period_sec = 0.1
        self.determine_position_timer = self.create_timer(self.positioning_period_sec, self.determine_position_callback)
        
        self.odom_publisher = self.create_publisher(Odometry, '/odom_combined', 10)

    def determine_position_callback(self):
        # TODO - implement
        pass

    




        


def main(args=None):
    rclpy.init(args=args)

    localizer = Localizer()

    rclpy.spin(localizer)

    localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()