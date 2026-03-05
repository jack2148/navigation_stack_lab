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

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])