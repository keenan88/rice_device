import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Robot_Behaviour(Node):
    def __init__(self):
        super().__init__('Robot_Behaviour')

        self.ROBOT_BEHAIOUR_INITIALIZE = Int32(data = 0)
        self.ROBOT_BEHAIOUR_LINEAR_FULL_SPEED = Int32(data = 1)
        self.ROBOT_BEHAIOUR_LINEAR_SLOW_SPEED = Int32(data = 2)
        self.ROBOT_BEHAIOUR_STOP = Int32(data = 3)
        self.ROBOT_BEHAIOUR_TURN = Int32(data = 4)
        
        self.robot_behaviour_state = self.ROBOT_BEHAIOUR_INITIALIZE

        self.behaviour_state_publisher = self.create_publisher(Int32, '/robot_behaviour_state', 10)
        self.behaviour_state_publisher.publish(self.ROBOT_BEHAIOUR_INITIALIZE)
        self.behaviour_state_timer = self.create_timer(1, self.publisher_behaviour_state)

        self.behaviour_state_subscriber = self.create_subscription(
            Int32, '/set_robot_behaviour_state', self.behaviour_state_callback, 10
        )

    def behaviour_state_callback(self, msg: Int32):
        self.robot_behaviour_state = msg
        self.get_logger().info('Robot Behaviour State: %d' % self.robot_behaviour_state.data)
        self.behaviour_state_publisher.publish(self.robot_behaviour_state)

    def publisher_behaviour_state(self):
        self.behaviour_state_publisher.publish(self.robot_behaviour_state)



def main(args=None):
    rclpy.init(args=args)
    robot_behaviour = Robot_Behaviour()
    rclpy.spin(robot_behaviour)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
