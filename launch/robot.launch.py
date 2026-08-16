from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('ddsm115_controller'),
        'config',
        'robot.yaml',
    ])
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Shared motor and robot parameter file',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='ddsm115_controller',
            executable='velocity_control',
            name='velocity_control_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='ddsm115_controller',
            executable='two_wheels_robot',
            name='two_wheels_robot_node',
            output='screen',
            parameters=[config],
        ),
    ])
