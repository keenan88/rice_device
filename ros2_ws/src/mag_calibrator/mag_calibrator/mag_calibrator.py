import rclpy
from rclpy.node import Node

from std_msgs.msg import String
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
        self.subscription_continue_calib = self.create_subscription(
            String,
            'bno055/continue_mag_calib',
            self.continue_calib_cb,
            10 # QOS profile.
        )
        self.continue_calib = 0

        self.subscription  # prevent unused variable warning
        self.subscription_continue_calib

        self.total_samples_desired = 100
        self.samples = {
            'seconds': [], 'nanosecs': [], 
            'mag': {'x': [], 'y': [], 'z': []}
        }

        self.mag_x_offset = -402.333
        self.mag_y_offset = -509.0
        self.mag_z_offset = 46.798

        self.mag_scalar_x = 0.062
        self.mag_scalar_y = -0.0617
        self.mag_scalar_z = 0.05915

        # self.mag_x_offset = 0
        # self.mag_y_offset = 0
        # self.mag_z_offset = 0

        # self.mag_scalar_x = 1
        # self.mag_scalar_y = 1
        # self.mag_scalar_z = 1

        self.last_n_xs = list(np.zeros(20))

        self.calib_file_names = []
            
        for major_ax in ['x', 'y', 'z']:
                self.calib_file_names.append(major_ax + 'east')

        self.calib_dir_idx = 0

        self.file_name = self.calib_file_names[self.calib_dir_idx]

        self.get_logger().info('Place sensor in position: %s' % (self.file_name))

    
    def listener_callback(self, msg: MagneticField):
        
        if self.continue_calib >= 1 and self.calib_dir_idx < len(self.calib_file_names):
            
            msg.magnetic_field.x += self.mag_x_offset
            msg.magnetic_field.y += self.mag_y_offset
            msg.magnetic_field.z += self.mag_z_offset

            msg.magnetic_field.x *= self.mag_scalar_x
            msg.magnetic_field.y *= self.mag_scalar_y
            msg.magnetic_field.z *= self.mag_scalar_z

            self.last_n_xs.pop(0)
            self.last_n_xs.append(msg.magnetic_field.x)

            self.samples['seconds'].append(msg.header.stamp.sec)
            self.samples['nanosecs'].append(msg.header.stamp.nanosec)
            self.samples['mag']['x'].append(msg.magnetic_field.x)
            self.samples['mag']['y'].append(msg.magnetic_field.y)
            self.samples['mag']['z'].append(msg.magnetic_field.z)

            mag_field_magnitude = round(
                np.sqrt(msg.magnetic_field.x ** 2 + \
                        msg.magnetic_field.y ** 2 + \
                        msg.magnetic_field.z ** 2), 2)
            self.get_logger().info('%f, %f, %f, %f' % (
                #sum(self.last_n_xs) / len(self.last_n_xs),
                msg.magnetic_field.x, 
                msg.magnetic_field.y, 
                msg.magnetic_field.z, 
                mag_field_magnitude)
            )

            # if len(self.samples['seconds']) >= self.total_samples_desired:

            #     file_path = samples_folder + self.file_name + '.json' # Using name as time stamp ensures unique filenames, no overwrites
            #     #file_path = samples_folder + 'neg_z_on_table' + '.json' # Using name as time stamp ensures unique filenames, no overwrites

            #     with open(file_path, "w") as outfile: 
            #         json.dump(self.samples, outfile)

            #     self.get_logger().info('%f Samples written to %s' % (len(self.samples['seconds']), file_path))
            #     self.samples = {
            #         'seconds': [], 'nanosecs': [], 
            #         'mag': {'x': [], 'y': [], 'z': []
            #         }
            #     }    

            #     self.calib_dir_idx += 1
            #     self.file_name = self.calib_file_names[self.calib_dir_idx]
            #     self.get_logger().info('Place sensor in position: %s' % (self.file_name))
            #     self.continue_calib -= 1

                
                
    def continue_calib_cb(self, msg):
        self.continue_calib += 1
            




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