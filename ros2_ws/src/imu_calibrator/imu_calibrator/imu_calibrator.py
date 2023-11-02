import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Imu
import json
import numpy as np

samples_folder = '/home/keenan/Documents/rice_device/signal_analysis/database/imu/'

class IMU_Calibration(Node):

    def __init__(self):
        super().__init__('IMU_Calibration')
        self.subscription = self.create_subscription(
            Imu,
            'bno055/imu_raw',
            self.listener_callback,
            10 # QOS profile.
        )
        self.subscription  # prevent unused variable warning

        self.total_samples_desired = 100
        self.samples = {
            'seconds': [], 'nanosecs': [], 
            'accel': {'x': [], 'y': [], 'z': []},
            'gyro':  {'x': [], 'y': [], 'z': []}
        }

        self.acc_x_offset = -0.595 - 9.985 - 1.61
        self.acc_y_offset = -20.812 - 0.848 + 0.62
        self.acc_z_offset = 3.557 + 22.09 + 0.6

        self.acc_scalar = 0.01


    
    def listener_callback(self, msg: Imu):
        #self.get_logger().info('I heard: "%s"' % msg.orientation)

        # self.get_logger().info('%f %f %f %f %f' % (accel.x+ self.x_offset, accel.y + self.y_offset, \
        #                                            accel.z + self.z_offset, time.stamp.sec, time.stamp.nanosec))

        msg.linear_acceleration.x = (msg.linear_acceleration.x + self.acc_x_offset) * self.acc_scalar
        msg.linear_acceleration.y = (msg.linear_acceleration.y + self.acc_y_offset) * self.acc_scalar
        msg.linear_acceleration.z = (msg.linear_acceleration.z + self.acc_z_offset) * self.acc_scalar

        self.samples['seconds'].append(msg.header.stamp.sec)
        self.samples['nanosecs'].append(msg.header.stamp.nanosec)
        self.samples['accel']['x'].append(msg.linear_acceleration.x)
        self.samples['accel']['y'].append(msg.linear_acceleration.y)
        self.samples['accel']['z'].append(msg.linear_acceleration.z)
        self.samples['gyro']['x'].append(msg.angular_velocity.x)
        self.samples['gyro']['y'].append(msg.angular_velocity.y)
        self.samples['gyro']['z'].append(msg.angular_velocity.z)

        acc_mag = round(np.sqrt(msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 + msg.linear_acceleration.z ** 2), 2)
        self.get_logger().info('%f, %f, %f, %f' % (round(msg.linear_acceleration.x, 2), round(msg.linear_acceleration.y, 2), round(msg.linear_acceleration.z, 2), acc_mag))

        if len(self.samples['seconds']) >= self.total_samples_desired:

            #file_path = samples_folder + str(msg.header.stamp.sec) + '.json' # Using name as time stamp ensures unique filenames, no overwrites
            file_path = samples_folder + 'neg_z_on_table' + '.json' # Using name as time stamp ensures unique filenames, no overwrites

            with open(file_path, "w") as outfile: 
                json.dump(self.samples, outfile)

            self.get_logger().info('%f Samples written to %s' % (len(self.samples['seconds']), file_path))
            self.samples = {
                'seconds': [], 'nanosecs': [], 
                'accel': {'x': [], 'y': [], 'z': []},
                'gyro':  {'x': [], 'y': [], 'z': []}
            }          
        




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = IMU_Calibration()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()