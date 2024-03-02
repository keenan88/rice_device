import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class Robot_Behaviour(Node):
    def __init__(self):
        super().__init__('Robot_Behaviour')

        self.ROBOT_BEHAIOUR_INITIALIZE = 0
        self.ROBOT_BEHAIOUR_LINEAR_FULL_SPEED = 0
        self.ROBOT_BEHAIOUR_LINEAR_SLOW_SPEED = 0
        self.ROBOT_BEHAIOUR_STOP = 0
        self.ROBOT_BEHAIOUR_TURN = 0
        
        self.robot_behaviour_state = self.ROBOT_BEHAIOUR_INITIALIZE

        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response



def main(args=None):
    rclpy.init(args=args)
    robot_behaviour = Robot_Behaviour()
    rclpy.spin(robot_behaviour)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
