import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    nav2_params_file = os.path.join(pkg_share, 'config', 'mppi_coordi.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'final_map11.yaml')

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    robot_pose_launch = os.path.join(
        pkg_share,
        'launch',
        'robot_pose.launch.py'
    )

    return LaunchDescription([
        SetRemap('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params_file,
                'map': map_file
                }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_pose_launch)
        )

    ])
