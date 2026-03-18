from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': '/home/chan/maps/map_v1_debug.yaml',
                'use_sim_time': 'false',
                'params_file': '/home/chan/navigation_stack_lab/ws_robot/src/robot_base/config/nav2_params.yaml'
            }.items()
        )
    ])