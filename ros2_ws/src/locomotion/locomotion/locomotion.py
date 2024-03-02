import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from robot_behaviour_interface.srv import GetRobotBehaviourState
from time import sleep

class Locomotion(Node):

    def __init__(self):
        super().__init__('Localizer')

        self.cli = self.create_client(GetRobotBehaviourState, '/get_robot_behaviour_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = GetRobotBehaviourState.Request()

        self.future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)
        
        print(self.future.result())

    
def main(args=None):
    rclpy.init(args=args)

    locomotion = Locomotion()

    rclpy.spin(locomotion)

    locomotion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()