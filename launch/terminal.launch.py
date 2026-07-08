"""
一键启动 aider_terminal 节点。

用法:
    ros2 launch aiderminal terminal.launch.py
    ros2 launch aiderminal terminal.launch.py robot_type:=aider
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='aider'),
        DeclareLaunchArgument('no_sim', default_value='false'),
        DeclareLaunchArgument('no_viz', default_value='false'),
        DeclareLaunchArgument('no_vr', default_value='false'),
        DeclareLaunchArgument('no_keyboard', default_value='false'),
        DeclareLaunchArgument('autoconnect', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='warning'),
        DeclareLaunchArgument('env_dev', default_value='false'),
        DeclareLaunchArgument('server_host', default_value=''),
        DeclareLaunchArgument('api_host', default_value=''),
        DeclareLaunchArgument('ws_port', default_value='8442'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),

        Node(
            package='aiderminal',
            executable='terminal_node',
            name='aider_terminal',
            output='screen',
            parameters=[{
                'robot_type': LaunchConfiguration('robot_type'),
                'no_sim': LaunchConfiguration('no_sim'),
                'no_viz': LaunchConfiguration('no_viz'),
                'no_vr': LaunchConfiguration('no_vr'),
                'no_keyboard': LaunchConfiguration('no_keyboard'),
                'autoconnect': LaunchConfiguration('autoconnect'),
                'log_level': LaunchConfiguration('log_level'),
                'env_dev': LaunchConfiguration('env_dev'),
                'server_host': LaunchConfiguration('server_host'),
                'api_host': LaunchConfiguration('api_host'),
                'ws_port': LaunchConfiguration('ws_port'),
                'host': LaunchConfiguration('host'),
            }],
        ),
    ])
