import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import Accel
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import json

samples_folder = '/home/keenan/Documents/rice_device/ros2_ws/src/accel_extractor/samples/'

class AccelExtractor(Node):

    def __init__(self):
        super().__init__('AccelExtractor')
        self.subscription = self.create_subscription(
            Imu,
            'bno055/imu_raw',
            self.listener_callback,
            10 # QOS profile.
        )
        self.subscription  # prevent unused variable warning

        self.total_samples_desired = 1000
        self.samples = {'seconds': [], 'nanosecs': [], 'accelXs': [], 'accelYs': [], 'accelZs': []}
        self.x_offset = 0#7.71
        self.y_offset = 0#-220.75
        self.z_offset = 0#0.47

    
    def listener_callback(self, msg: Imu):
        #self.get_logger().info('I heard: "%s"' % msg.orientation)
        accel = msg.linear_acceleration
        time = msg.header

        # self.get_logger().info('%f %f %f %f %f' % (accel.x+ self.x_offset, accel.y + self.y_offset, \
        #                                            accel.z + self.z_offset, time.stamp.sec, time.stamp.nanosec))

        self.samples['seconds'].append(time.stamp.sec)
        self.samples['nanosecs'].append(time.stamp.nanosec)
        self.samples['accelXs'].append(accel.x + self.x_offset)
        self.samples['accelYs'].append(accel.y + self.y_offset)
        self.samples['accelZs'].append(accel.z + self.z_offset)

        if len(self.samples['seconds']) >= self.total_samples_desired:

            file_path = samples_folder + str(time.stamp.sec) + '.json' # Using name as time stamp ensures unique filenames, no overwrites

            with open(file_path, "w") as outfile: 
                json.dump(self.samples, outfile)

            self.get_logger().info('%f Samples written to %s' % (len(self.samples['seconds']), file_path))
            self.samples = {'seconds': [], 'nanosecs': [], 'accelXs': [], 'accelYs': [], 'accelZs': []}

        else:
            pass
            #self.get_logger().info('%f IMU accel samples obtained' % (len(self.samples['seconds'])))
            




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