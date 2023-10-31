from launch import LaunchDescription
from launch_ros.actions import Node

ros2_ws_path = '/home/keenan/Documents/rice_device/ros2_ws/'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='euler_angle_extractor',
            executable='euler_angle_extractor_exe',
        ),
        Node(
            package='bno055',
            executable='bno055',
        ),
        Node(
             package='accel_extractor',
             executable='accel_extractor_exe',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', [ros2_ws_path + 'src/rice_device_bringup/rviz_configs/rviz_bno055_config.rviz']]
        )
    ])