from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='central_shaft',
            executable='central_shaft',
        ),

        Node(
            package='linear_actuator',
            executable='linear_actuator',
        ),

        Node(
            package='stepper_driver',
            executable='stepper_driver_exe',
        ),

        Node(
            package='locomotion',
            executable = 'locomotion_exe'
        )

    ])