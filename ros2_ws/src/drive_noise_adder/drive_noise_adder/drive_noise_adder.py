import rclpy
from rclpy.node import Node
from gpiozero import DigitalOutputDevice
from time import sleep
from geometry_msgs.msg import Twist
from math import pi
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import numpy as np



class DriveNoiseAdder(Node):

    def __init__(self):
        super().__init__('Drive_Noise_Adder')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.given_cmd_vel = Twist()

        self.noise_change_period = 0.05

        self.noised_cmd_vel_timer = self.create_timer(self.noise_change_period, self.noised_cmd_vel_callback)

        self.noise_cmd_vel_pub = self.create_publisher(Twist, '/noised_cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        self.given_cmd_vel = msg

    def noised_cmd_vel_callback(self):
        

        if self.given_cmd_vel.linear.x != 0:

            self.noised_cmd_vel = self.given_cmd_vel
            angle_noise = np.random.normal(0, 0.00)
            lim = 0.1
            if angle_noise > lim: angle_noise = lim
            if angle_noise < -lim: angle_noise = -lim

            self.noised_cmd_vel.angular.z = self.given_cmd_vel.angular.z + angle_noise
            self.noise_cmd_vel_pub.publish(self.noised_cmd_vel)

        else:
            stop_cmd_vel = Twist()
            self.noise_cmd_vel_pub.publish(stop_cmd_vel)




        



def main(args=None):
    rclpy.init(args=args)

    drive_noise_adder = DriveNoiseAdder()

    rclpy.spin(drive_noise_adder)

    drive_noise_adder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()