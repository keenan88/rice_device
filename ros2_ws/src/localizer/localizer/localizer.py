import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import sin, cos
from std_msgs.msg import Int32

class Localizer(Node):

    def __init__(self):
        super().__init__('Localizer')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initial position estimations, that central shaft is in hole and robot floppers are engaged
        # IE - robot is at (0,0,0), (0,0,0) (lin, ang)

        self.robot_odom = Odometry()
        robot_odom.header.frame_id = 'world'
        robot_odom.child_frame_id = 'base_link'

        self.drive_control_velocity = Twist()
        self.drive_control_sub = self.create_subscription(Twist, '/cmd_vel', self.record_drive_control_velocity, 10)

        self.zeroing_subscriber = self.create_subscription(Int32, '/zero_robot', self.zeroing_callback, 10)

        self.robot_odom_publisher = self.create_publisher(Odometry, '/robot_odom', 10)
        
        # Localization Algorithm
        self.localization_period = 0.1  # seconds
        self.localization_timer = self.create_timer(self.localization_period, self.localize)

    def zeroing_callback(self, dummy_msg):
        self.robot_odom.pose.pose.position.x = 0
        self.robot_odom.pose.pose.position.y = 0
        self.robot_odom.pose.pose.position.z = 0
        print("Zeroed Robot Position")
    

    def record_drive_control_velocity(self, robot_drive_twist):     
        self.drive_control_velocity = robot_drive_twist
        

    def quaternion_to_euler(self, quaternion):
        quat_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = tf_transformations.euler_from_quaternion(quat_list)
        return euler

    def localize(self):

        self.robot_odom.header.stamp = self.get_clock().now().to_msg()

        last_euler_angles = self.quaternion_to_euler(self.robot_odom.pose.pose.orientation)

        avgd_angle_over_last_step = last_euler_angles[2] + \
            self.drive_control_velocity.angular.z * self.localization_period / 2

        delta_x = self.drive_control_velocity.linear.x * self.localization_period * cos(avgd_angle_over_last_step)
        delta_y = self.drive_control_velocity.linear.y * self.localization_period * sin(avgd_angle_over_last_step)
        delta_theta = self.drive_control_velocity.angular.z * self.localization_period

        self.robot_odom.pose.pose.position.x += delta_x
        self.robot_odom.pose.pose.position.y += delta_y
        self.robot_odom.pose.pose.orientation.z += delta_theta # This may be wrong, supposed to be quat not euler

        self.robot_odom_publisher.publish(self.robot_odom)


def main(args=None):
    rclpy.init(args=args)

    localizer = Localizer()

    rclpy.spin(localizer)

    localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()