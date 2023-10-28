import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math

class EulerAngleExtractor(Node):

    def __init__(self):
        super().__init__('EulerAngleExtractor')
        self.subscription = self.create_subscription(
            Imu,
            'bno055/imu',
            self.listener_callback,
            10 # QOS profile.
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.orientation)
        quat = msg.orientation
        roll_x, pitch_y, yaw_z = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        print(roll_x, pitch_y, yaw_z)

    
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = round(math.atan2(t0, t1), 2)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = round(math.asin(t2), 2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = round(math.atan2(t3, t4), 2)
     
        return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = EulerAngleExtractor()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()