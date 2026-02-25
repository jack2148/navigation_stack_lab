from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    simulate_can = LaunchConfiguration('simulate_can')
    can_interface = LaunchConfiguration('can_interface')
    encoder_rate_hz = LaunchConfiguration('encoder_rate_hz')

    base_params = PathJoinSubstitution([FindPackageShare('robot_bringup'), 'config', 'base.yaml'])

    # URDF(xacro) -> robot_description parameter
    xacro_file = PathJoinSubstitution([FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro'])
    robot_description = {
        'robot_description': Command([FindExecutable(name='xacro'), ' ', xacro_file])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('simulate_can', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('encoder_rate_hz', default_value='50.0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_base',
            executable='can_bridge_node',
            name='can_bridge',
            output='screen',
            parameters=[
                base_params,
                {'use_sim_time': use_sim_time},
                {'simulate': simulate_can},
                {'can_interface': can_interface},
                {'publish_rate_hz': encoder_rate_hz},
            ],
        ),

        Node(
            package='robot_base',
            executable='wheel_odometry_node',
            name='wheel_odometry',
            output='screen',
            parameters=[
                base_params,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
