from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("robot_base")

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")
    urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    # ---------- 로봇 상태(TF 트리) 퍼블리셔 영역 ----------
    # URDF에 선언된 링크들(base_footprint -> base_link -> laser_frame 등)의 
    # 위치 변환(TF)을 ROS 네트워크에 뿌려주는 필수 노드입니다.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Spawn controllers (you need these to get /odom and list_controllers)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",

    )

    # ---------- YDLIDAR 파라미터 파일 로드 ----------
    default_ydlidar_params_file = os.path.join(pkg_share, 'config', 'ydlidar.yaml')
    ydlidar_params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_ydlidar_params_file,
        description='Path to the ROS2 parameters file to use for YDLIDAR.'
    )

    # ---------- YDLIDAR 라이다 드라이버 (스캔 원본) ----------
    ydlidar_driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        namespace='/',
        remappings=[('/scan', '/scan_raw')]
    )

    # ---------- 레이저 컷오프(필터) 노드 ----------
    laser_filter_config = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan')
        ]
    )

    return LaunchDescription([
        ydlidar_params_declare,
        robot_state_pub_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        ydlidar_driver_node,
        laser_filter_node,
    ])