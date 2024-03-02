import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odom

class Accel_Integrator(Node):
    def __init__(self):
        super().__init__('tf2_publisher')

        self.imu_subber = self.create_subscription(Imu, "/bno055/imu", self.integrate_acceleration, 10)

        self.odom_publisher = self.create_publisher(Odom, '/odom_from_imu', 10)

        last_imu_


    def integrate_acceleration(self, imu_msg: Imu):
        







    

def main(args=None):
    rclpy.init(args=args)
    accel_integrator = Accel_Integrator()

    try:
        while rclpy.ok():
            rclpy.spin_once(accel_integrator, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    accel_integrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
