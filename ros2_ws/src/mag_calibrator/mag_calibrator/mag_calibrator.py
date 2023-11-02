import rclpy
from rclpy.node import Node

from sensor_msgs.msg import MagneticField
import json
import numpy as np

samples_folder = '/home/keenan/Documents/rice_device/signal_analysis/database/magnetometer/'

class IMU_Calibration(Node):

    def __init__(self):
        super().__init__('IMU_Calibration')
        self.subscription = self.create_subscription(
            MagneticField,
            'bno055/mag',
            self.listener_callback,
            10 # QOS profile.
        )
        self.subscription  # prevent unused variable warning

        self.total_samples_desired = 100
        self.samples = {
            'seconds': [], 'nanosecs': [], 
            'mag': {'x': [], 'y': [], 'z': []}
        }

        self.mag_x_offset = 0
        self.mag_y_offset = 0
        self.mag_z_offset = 0


    
    def listener_callback(self, msg: MagneticField):
        
        msg.magnetic_field.x += self.mag_x_offset
        msg.magnetic_field.y += self.mag_y_offset
        msg.magnetic_field.z += self.mag_z_offset

        self.samples['seconds'].append(msg.header.stamp.sec)
        self.samples['nanosecs'].append(msg.header.stamp.nanosec)
        self.samples['mag']['x'].append(msg.magnetic_field.x)
        self.samples['mag']['y'].append(msg.magnetic_field.y)
        self.samples['mag']['z'].append(msg.magnetic_field.z)

        mag_field_magnitude = round(
            np.sqrt(msg.magnetic_field.x ** 2 + \
                    msg.magnetic_field.y ** 2 + \
                    msg.magnetic_field.z ** 2), 2)
        self.get_logger().info('%f, %f, %f, %f' % (msg.magnetic_field.x, 
                                                   msg.magnetic_field.y, 
                                                   msg.magnetic_field.z, 
                                                   mag_field_magnitude))

        if len(self.samples['seconds']) >= self.total_samples_desired:

            #file_path = samples_folder + str(msg.header.stamp.sec) + '.json' # Using name as time stamp ensures unique filenames, no overwrites
            file_path = samples_folder + 'neg_z_on_table' + '.json' # Using name as time stamp ensures unique filenames, no overwrites

            with open(file_path, "w") as outfile: 
                json.dump(self.samples, outfile)

            self.get_logger().info('%f Samples written to %s' % (len(self.samples['seconds']), file_path))
            self.samples = {
            'seconds': [], 'nanosecs': [], 
            'mag': {'x': [], 'y': [], 'z': []}
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