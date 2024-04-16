import rclpy
from rclpy.node import Node


class RotationLocalizer(Node):

    def __init__(self):
        super().__init__('Rotation_Localizer')

        self.rotation = 0

        self.imu_subscription = self.create_subscription(
            Imu,
            '/bno055/imu_raw',
            self.incoming_imu_callback,
            10
        )



    def incoming_imu_callback(self, msg):
        self.rotation = msg.orientation.z

        self.get_logger().info('Rotation: %f' % self.rotation)

        

    
        
def main(args=None):
    rclpy.init(args=args)

    rotation_localizer = RotationLocalizer()

    rclpy.spin(rotation_localizer)

    rotation_localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()