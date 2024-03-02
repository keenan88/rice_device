from robot_behaviour_interface.msg import RobotBehaviourState
from robot_behaviour_interface.srv import GetRobotBehaviourState

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            GetRobotBehaviourState, 
            'get_robot_behaviour_state', 
            self.provide_behaviour_state
        )
        #self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def provide_behaviour_state(self, request, response):
        
        response.robot_behaviour_state = 1

        return response
    


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()