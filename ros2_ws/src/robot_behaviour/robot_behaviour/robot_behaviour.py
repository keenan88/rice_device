import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from enum import Enum

class Robot_Behaviour(Enum):
    INITIALIZE = Int32(data = 0)

    RAMP_TO_HIGH_SPEED = Int32(data = 1)
    MAINTAIN_HIGH_SPEED = Int32(data = 2)

    RAMP_TO_LOW_SPEED = Int32(data = 3)
    MAINTAIN_LOW_SPEED = Int32(data = 4)

    STOP = Int32(data = 5)
    TURN = Int32(data = 6)



class Robot_Behaviour(Node):
    def __init__(self, robot_behaviour = Robot_Behaviour.INITIALIZE.value):
        super().__init__('Robot_Behaviour')
        
        self.robot_behaviour_state = robot_behaviour

        self.behaviour_state_publisher = self.create_publisher(Int32, '/robot_behaviour_state', 10)
        self.behaviour_state_publisher.publish(self.robot_behaviour_state)
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
