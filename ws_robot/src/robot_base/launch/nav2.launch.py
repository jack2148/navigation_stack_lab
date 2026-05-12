import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    nav2_params_file = os.path.join(pkg_share, 'config', 'mppi_coordi.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'center.yaml')
    keepout_mask_file = os.path.join(pkg_share, 'maps', 'keepoutmask.yaml')

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

    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': keepout_mask_file,
            'topic_name': 'keepout_filter_mask',
            'frame_id': 'map',
        }]
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'type': 0,
            'filter_info_topic': 'costmap_filter_info',
            'mask_topic': 'keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }]
    )

    return LaunchDescription([
        SetRemap('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),

        keepout_mask_server,
        costmap_filter_info_server,

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
