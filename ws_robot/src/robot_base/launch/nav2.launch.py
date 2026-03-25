import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
<<<<<<< HEAD
    map_file = os.path.join(os.path.expanduser('~'),
                            'ws_robot/src/robot base/maps/my_map.yaml')   # 실제 파일명에 맞게
=======
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')   # 실제 파일명에 맞게
>>>>>>> 0ead885 (change prams for queue error : fix scan range , odom_topic:diff~/odom)

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
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
        )
    ])