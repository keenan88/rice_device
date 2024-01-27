import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from he_sensor_interface.msg import HeSensorTriggeredStamped


class HESensors(Node):

    def __init__(self):
        super().__init__('he_sensors')
        self.publisher_ = self.create_publisher(HeSensorTriggeredStamped, 'he_sensors', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = HeSensorTriggeredStamped()
        msg.stamp = self.get_clock().now().to_msg()
        msg.triggered = True
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = HESensors()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()