import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import tf_transformations as tf_trans

class TF2Publisher(Node):
    def __init__(self):
        super().__init__('tf2_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publish_timer = self.create_timer(1, self.publish_callback)

    def publish_callback(self):
        # Define the transformations
        translation_hole1 = [0.0, 0.0, 0.0]  # Translation for Frame 1
        rotation_hole1 = tf_trans.quaternion_from_euler(0, 0, 0)  # Rotation for Frame 1 (no rotation)
        translation_hole2 = [0.6, 0.0, 0.0]  # Translation for Frame 2
        rotation_hole2 = tf_trans.quaternion_from_euler(0, 0, 0)  # Rotation for Frame 2 (45 degrees around Z-axis)

        # Publish the transformations
        self.publish_transform("world", "hole1", translation_hole1, rotation_hole1)
        self.publish_transform("world", "hole2", translation_hole2, rotation_hole2)

    def publish_transform(self, parent_frame, child_frame, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = TF2Publisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
