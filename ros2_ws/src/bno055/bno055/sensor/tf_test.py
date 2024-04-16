import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from math import sqrt
import tf2_py
import math

class Tf2Publisher(Node):
    def __init__(self):
        super().__init__('tf2_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        self.imu_subber = self.create_subscription(Imu, "/bno055/imu", self.publish_transform, 10)

        self.transform = geometry_msgs.msg.TransformStamped()

        self.transform.header.frame_id = 'world'  # Parent frame
        self.transform.child_frame_id = 'my_frame'  # Child frame

        self.rot_z = 0
        
        clock_msg = self.get_clock().now().to_msg()
        self.last_measurement_time = float(clock_msg.sec) + float(clock_msg.nanosec) / 10e8
        self.previous_ang_vels = [0, 0, 0, 0, 0]

    def quaternion_to_euler(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.pi / 2 if sinp > 0 else -math.pi / 2  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return 180/3.14*round(yaw, 2)
        

    def publish_transform(self, imu_msg:Imu):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        clock_time_s = float(imu_msg.header.stamp.sec) + float(imu_msg.header.stamp.nanosec) / 10e8
        
        q = imu_msg.orientation
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) 
        self.transform.transform.rotation.x = q.x / norm
        self.transform.transform.rotation.y = q.y / norm
        self.transform.transform.rotation.z = q.z / norm
        self.transform.transform.rotation.w = q.w / norm

        elapsed_time = clock_time_s - self.last_measurement_time
        #print(round(self.rot_z*180/3.14, 2))

        self.rot_z += sum(self.previous_ang_vels) / len(self.previous_ang_vels) * elapsed_time

        self.previous_ang_vels = self.previous_ang_vels[1:]
        self.previous_ang_vels.append(imu_msg.angular_velocity.z)
        
        self.last_measurement_time = clock_time_s

        print(self.quaternion_to_euler(q), elapsed_time)

        self.broadcaster.sendTransform(self.transform)

def main(args=None):
    rclpy.init(args=args)
    tf2_publisher = Tf2Publisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(tf2_publisher, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    tf2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
