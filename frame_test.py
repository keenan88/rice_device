import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros

def main():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('tf_listener')

    # Create a TF2 buffer
    tf_buffer = tf2_ros.Buffer(node)

    # Create a TF2 listener
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    try:
        # Spin the node to keep it running
        while rclpy.ok():
            # Lookup the transform between source_frame and target_frame
            try:
                transform = tf_buffer.lookup_transform('world', 'farm_hole_2', rclpy.time.Time())
                print("Received TF transform:")
                print(transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("Exception occurred:", e)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
