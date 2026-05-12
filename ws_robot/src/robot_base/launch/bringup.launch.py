from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_base")

    # ---------- 파일 경로 ----------
    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")
    urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf")
    default_ydlidar_params_file = os.path.join(pkg_share, "config", "ydlidar.yaml")
    laser_filter_config = os.path.join(pkg_share, "config", "laser_filter.yaml")
    ekf_yaml = os.path.join(pkg_share, "config", "ekf.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ---------- Launch Argument ----------
    params_file = LaunchConfiguration("params_file")
    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=default_ydlidar_params_file,
        description="Path to the ROS2 parameters file to use for YDLIDAR."
    )

    # ---------- ROS 2 Control ----------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    # ---------- Robot State Publisher ----------
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ---------- YDLIDAR Driver ----------
    ydlidar_driver_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
        namespace="/",
        remappings=[("/scan", "/scan_raw")],
    )

    # ---------- Laser Filter ----------
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        parameters=[laser_filter_config],
        remappings=[
            ("scan", "/scan_raw"),
            ("scan_filtered", "/scan_pre"),
        ],
        output="screen",
    )

    # ---------- Controller Spawners ----------
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

    # ---------- IMU Node ----------
    imu_node = Node(
        package="robot_base",
        executable="imu_node",
        name="ebimu_publisher",
        output="screen",
        parameters=[
            {"port": "/dev/ttyUSB1"},
            {"baudrate": 115200},
        ],
    )

    # ---------- EKF Node ----------
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml],
    )

    # ---------- IMU 로드 노드 ----------
    # 로직: 작성했던 imu.cpp를 빌드한 실행파일을 켭니다. 파라미터로 포트와 속도를 넘겨줍니다.
    imu_node = Node(
        package="robot_base",
        executable="imu_node", # CMakeLists.txt에 정의될/된 실행파일명에 맞추셔야 합니다. (보통 imu_node 등)
        name="ebimu_publisher",
        output="screen",
        parameters=[
            {"port": "/dev/ttyUSB1"},
            {"baudrate": 115200}
        ]
    )

    # ---------- EKF 노드 (robot_localization) ----------
    # 로직: 휠 오도메트리와 IMU 센서 데이터를 융합하여 base_footprint TF를 발행합니다.
    ekf_yaml_path = os.path.join(pkg_share, "config", "ekf.yaml")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml_path]
    )

    # ---------- Scan Restamper ----------
    # YDLidar 디바이스 클록 드리프트로 인한 TF lookup 실패 방지
    scan_restamper_node = Node(
        package="robot_base",
        executable="scan_restamper.py",
        name="scan_restamper",
        output="screen",
    )

    return LaunchDescription([
        params_declare,
        robot_state_pub_node,
        ros2_control_node,
        ydlidar_driver_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        laser_filter_node,
        imu_node,
        ekf_node,
        scan_restamper_node,
    ])