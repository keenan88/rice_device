import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

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

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.orientation)
        quat = msg.orientation
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        print(roll_x, pitch_y, yaw_z)

        self.make_transforms(quat)

    def make_transforms(self, quaternion_angle):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'bno055'

        t.transform.translation.x = float(0)
        t.transform.translation.y = float(0)
        t.transform.translation.z = float(0)
        t.transform.rotation.x = float(quaternion_angle.x)
        t.transform.rotation.y = float(quaternion_angle.y)
        t.transform.rotation.z = float(quaternion_angle.z)
        t.transform.rotation.w = float(quaternion_angle.w)

        self.tf_static_broadcaster.sendTransform(t)

    
    # Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    # Alternative source: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
    def euler_from_quaternion(self, x, y, z, w):
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