from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lidar_stub = LaunchConfiguration('use_lidar_stub')
    slam_params_file = LaunchConfiguration('slam_params_file')

    lidar_params = PathJoinSubstitution([FindPackageShare('robot_bringup'), 'config', 'lidar.yaml'])
    base_launch = PathJoinSubstitution([FindPackageShare('robot_bringup'), 'launch', 'base.launch.py'])

    slam_toolbox_launch = PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_lidar_stub', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('robot_bringup'), 'config', 'slam_toolbox.yaml'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Lidar stub (for pipeline testing). Replace with ydlidar_ros2_driver later.
        Node(
            package='robot_sensors',
            executable='lidar_stub_node',
            name='lidar_stub',
            output='screen',
            parameters=[lidar_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_lidar_stub),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={'slam_params_file': slam_params_file,
                              'use_sim_time': use_sim_time}.items()
        ),
    ])
