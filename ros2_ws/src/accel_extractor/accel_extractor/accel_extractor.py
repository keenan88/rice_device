import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import Accel
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import csv

samples_folder = '/home/keenan/Documents/rice_device/ros2_ws/src/accel_extractor/samples/'

class AccelExtractor(Node):

    def __init__(self):
        super().__init__('AccelExtractor')
        self.subscription = self.create_subscription(
            Imu,
            'bno055/imu',
            self.listener_callback,
            10 # QOS profile.
        )
        self.subscription  # prevent unused variable warning

        self.total_samples_desired = 100
        self.samples = []
        self.samples_were_written = 0

    
    def listener_callback(self, msg: Imu):
        #self.get_logger().info('I heard: "%s"' % msg.orientation)
        accel = msg.linear_acceleration
        time = msg.header

        #self.get_logger().info('%f %f %f %f %f' % (accel.x, accel.y, accel.z, time.stamp.sec, time.stamp.nanosec))

        if not self.samples_were_written:

            self.samples.append([accel.x, accel.y, accel.z, time.stamp.sec, time.stamp.nanosec])

            if len(self.samples) > self.total_samples_desired:

                file_path = samples_folder + 'samples1.csv'
                with open(file_path, 'w', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter=' ',
                                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    self.samples.insert(0, ['accelX', 'accelY', 'accelZ', 'sec', 'nanosec'])
                    for sample in self.samples:
                        spamwriter.writerow(sample)
                self.get_logger().info('%f Samples written to %s' % (len(self.samples), file_path))
                self.samples_were_written = 1
                




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = AccelExtractor()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()