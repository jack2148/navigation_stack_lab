import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_base',
            executable='robot_pose_tf_bridge.py',
            name='robot_pose_tf_bridge',
            output='screen',
            parameters=[
                {'output_topic': '/robot_pose'}
            ]
        )
    ])
