from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_dev',
            default_value='/dev/ttyUSB0',
            description='RS-485 serial device',
        ),
        DeclareLaunchArgument(
            'max_check',
            default_value='1',
            description='Highest motor ID to scan',
        ),
        DeclareLaunchArgument(
            'command_timeout',
            default_value='2.0',
            description='Seconds without an RPM command before stopping',
        ),
        Node(
            package='ddsm115_controller',
            executable='velocity_control',
            name='velocity_control_node',
            output='screen',
            parameters=[{
                'usb_dev': LaunchConfiguration('usb_dev'),
                'max_check': LaunchConfiguration('max_check'),
                'command_timeout': LaunchConfiguration('command_timeout'),
            }],
        ),
    ])
