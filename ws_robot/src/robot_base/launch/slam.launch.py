from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_params_file = '/home/chan/navigation_stack_lab/ws_robot/src/robot_base/config/slam.yaml'

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output= 'screen',
        parameters=[
            slam_params_file    
        ]
    )

    return LaunchDescription([
        slam_node
    ])
    