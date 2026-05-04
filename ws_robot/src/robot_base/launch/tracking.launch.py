from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_base',
            executable='tracking_node.py',
            name='tracking_node',
            output='screen'
        )
    ])
