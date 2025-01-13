from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    target_ip_arg = DeclareLaunchArgument('ip', default_value='192.168.137.101', description='host ip')

    # Node definition
    controller_node = Node(
        package='onrobot_2fg7',
        executable='onrobot_server_node',
        output='screen',
        emulate_tty=True, 
        parameters=[{
            'ip': LaunchConfiguration('ip'),
        }]
    )

    return LaunchDescription([
        target_ip_arg,
        controller_node,
    ])