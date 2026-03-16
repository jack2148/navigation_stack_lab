from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
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

    return LaunchDescription([
        params_declare,
        robot_state_pub_node,
        ros2_control_node,
        ydlidar_driver_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])