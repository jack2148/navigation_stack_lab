#!/usr/bin/python3
# Copyright 2020, EAIBOT (Modified for slam_chan package)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    try:
        share_dir = get_package_share_directory('robot_base')
    except Exception as e:
        print(f"Error finding package 'robot_base': {e}")
        # 만약 패키지를 못 찾으면 안전하게 종료하거나 기본 경로를 설정해야 하지만,
        # 여기서는 명확한 에러 확인을 위해 그대로 진행합니다.
        raise e

    # 2. 파라미터 파일 경로 설정
    #    (slam_chan/config/ydlidar.yaml 파일을 기본값으로 사용)
    parameter_file = LaunchConfiguration('params_file')
    
    # config 폴더 내의 ydlidar.yaml 경로를 정확히 지정
    default_params_file = os.path.join(share_dir, 'config', 'ydlidar.yaml')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file], # LaunchConfiguration을 통해 전달된 경로 사용
        namespace='/',
    )

    # 4. TF 퍼블리셔 (LiDAR 위치)
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        # arguments 순서: x y z yaw pitch roll frame_id child_frame_id
        # (쿼터니언 대신 오일러 각 사용 시 편의성 고려)
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])