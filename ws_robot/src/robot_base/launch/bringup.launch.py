from launch import LaunchDescription
from launch_ros.actions import Node
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
        remappings=[
            ("/diff_drive_controller/odom", "/odom"),
            ("/diff_drive_controller/cmd_vel", "/cmd_vel")
        ]
    )

    return LaunchDescription([
        robot_state_pub_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])