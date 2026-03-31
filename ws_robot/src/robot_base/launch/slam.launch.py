import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    pkg_share = get_package_share_directory('robot_base')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam.yaml')

    slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[slam_params_file]
)

    return LaunchDescription([
        slam_node
    ])
    