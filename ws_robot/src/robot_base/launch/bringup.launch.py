from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("robot_base")

    # ---------- 파라미터 및 파일 경로 설정 영역 ----------
    # 로봇 제어, URDF, 라이다 등 각종 설정 파일 경로를 지정합니다.
    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")
    urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf")
    ydlidar_params_file = os.path.join(pkg_share, "config", "ydlidar.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ---------- 런치 아규먼트 선언 영역 ----------
    # 스크립트 외부(명령줄 등)에서 파라미터 파일 경로를 쉽게 재정의할 수 있게 합니다.
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=ydlidar_params_file,
        description='Path to the ROS2 parameters file to use.'
    )

    # ---------- ROS 2 Control 노드 영역 ----------
    # 하드웨어 인터페이스와 컨트롤러 매니저를 연결하여 로봇 바퀴를 직접 제어합니다.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    # ---------- Robot State TF -----------------
    # URDF에 선언된 링크들(base_footprint -> base_link -> laser_frame 등)의 
    # 위치 변환(TF)을 ROS 네트워크에 뿌려주는 필수 노드입니다.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ---------- Lidar driver Node
    ydlidar_driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        namespace='/',
    )

    # ---------- 컨트롤러 Spawner 파트 ----------
    # 각 컨트롤러 활성화를 위해 controller_manager에 시작 요청을 보냅니다.
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

    # ---------- IMU 로드 노드 ----------
    # 로직: 작성했던 imu.cpp를 빌드한 실행파일을 켭니다. 파라미터로 포트와 속도를 넘겨줍니다.
    imu_node = Node(
        package="robot_base",
        executable="imu_node", # CMakeLists.txt에 정의될/된 실행파일명에 맞추셔야 합니다. (보통 imu_node 등)
        name="ebimu_publisher",
        output="screen",
        parameters=[
            {"port": "/dev/ttyUSB0"},
            {"baudrate": 115200}
        ]
    )

    # ---------- EKF 노드 (robot_localization) ----------
    # 로직: 휠 오도메트리(Wheel)와 IMU 센서 데이터를 융합(Fusion)하여 더 정확한 Odom을 계산합니다.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"frequency": 30.0},
            {"sensor_timeout": 0.1},
            {"two_d_mode": True}, # 2D 평면을 주행하므로 True
            {"publish_tf": True},
            {"map_frame": "map"},
            {"odom_frame": "odom"},
            {"base_link_frame": "base_footprint"}, # 로봇 URDF의 최하단 링크
            {"world_frame": "odom"},

            # 1. 휠 오도메트리 파라미터 연동
            # diff_drive_controller에서 나오는 초기 odom 값 사용
            {"odom0": "/diff_drive_controller/odom"},
            # [X, Y, Z, Roll, Pitch, Yaw, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]
            # X, Y 위치와 V_x, V_yaw 속도만 주로 신뢰합니다.
            {"odom0_config": [True,  True,  False,
                              False, False, True,
                              True,  False, False,
                              False, False, True,
                              False, False, False]},
                              
            # 2. IMU 데이터 파라미터 연동
            # 센서에서 직접 퍼블리시한 IMU 데이터 경로
            {"imu0": "/imu/data"},
            # IMU는 주로 방향(Yaw)과 각속도(Vyaw)에 강점이 있으므로 그 부분을 신뢰합니다.
            {"imu0_config": [False, False, False,
                             False, False, True,
                             False, False, False,
                             False, False, True,
                             False, False, False]},
        ]
    )

    return LaunchDescription([
        params_declare,
        ydlidar_params_declare,
        robot_state_pub_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        ydlidar_driver_node,
        laser_filter_node,
        imu_node,
        ekf_node,
    ])