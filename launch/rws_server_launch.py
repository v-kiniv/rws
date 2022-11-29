import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_port_param = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port to use for websocket server'
    )

    declare_rosbridge_compat_param = DeclareLaunchArgument(
        'rosbridge_compatible',
        default_value='True',
        description='Enable compatibility with rosbridge protocol'
    )

    declare_watchgod_param = DeclareLaunchArgument(
        'watchdog',
        default_value='True',
        description='Use ping/pong to detect and drop unresponsive clients that keep TCP connection open'
    )

    rws_server_node = Node(
        package='rws',
        executable='rws_server',
        name='rws_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'rosbridge_compatible': LaunchConfiguration('rosbridge_compatible'),
            'watchdog': LaunchConfiguration('watchdog'),
        }]
    )

    return LaunchDescription([
        declare_port_param,
        declare_rosbridge_compat_param,
        declare_watchgod_param,
        rws_server_node
    ])
