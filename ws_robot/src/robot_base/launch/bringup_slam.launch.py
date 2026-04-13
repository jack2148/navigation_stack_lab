import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')

    # ---------- 1. 하드웨어 레벨: 기본 센서 및 모터 통신 구동 ----------
    hardware_bringup_launch = os.path.join(
        pkg_share, 'launch', 'bringup.launch.py'
    )

    # ---------- 2. 기능 레벨: SLAM 스택 구동 ----------
    # SLAM Toolbox 구동 (map -> odom 생성)
    slam_launch = os.path.join(
        pkg_share, 'launch', 'slam.launch.py'
    )

    return LaunchDescription([
        # 1. 하위 하드웨어 활성화
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_bringup_launch)
        ),
        
        # 2. 매핑엔진 활성화
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch)
        ),
    ])
