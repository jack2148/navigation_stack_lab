import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')

    # ---------- 1. 하드웨어 레벨: 기본 센서 및 모터 통신 구동 ----------
    # existing bringup.launch.py (odom -> base_link 생성 및 tf 발행)
    hardware_bringup_launch = os.path.join(
        pkg_share, 'launch', 'bringup.launch.py'
    )

    # ---------- 2. 기능 레벨: 네비게이션 엔진 구동 ----------
    # Nav2 및 robot_pose.launch.py (map -> odom 포함)
    nav2_launch = os.path.join(
        pkg_share, 'launch', 'nav2.launch.py'
    )

    return LaunchDescription([
        # 1. 하위 하드웨어 활성화
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_bringup_launch)
        ),
        
        # 2. 맵 기반 내비게이션 활성화
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch)
        ),
    ])
