"""Gazebo Harmonic 仿真启动（空世界）。

用于后续强化学习与物理仿真。与 PyBullet 节点可并存，
两套仿真通过 ROS_DOMAIN_ID 或节点命名区分。

前置: 需在支持图形的环境运行（Gazebo 需要 GPU/OpenGL）。

用法:
    ros2 launch robot_sim gazebo.launch.py
    ros2 launch robot_sim gazebo.launch.py headless:=true    # 无 GUI（服务器/RL 训练）
    ros2 launch robot_sim gazebo.launch.py world:=empty
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim')
    default_world = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Gazebo 世界文件路径'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='无 GUI 模式（服务器/RL 训练用）'),

        # Gazebo Harmonic (gz-sim)
        # -r: 启动即运行仿真
        # -s: server-only 无 GUI（headless，服务器/RL 训练用）
        Node(
            package='ros_gz_sim',
            executable='gz_sim',
            name='gazebo',
            output='screen',
            arguments=[
                '-r',
                '-s' if os.environ.get('GZ_HEADLESS', '0') == '1' else '',
                LaunchConfiguration('world'),
            ],
        ),

        # ROS-Gazebo 桥接（时钟同步是刚需，否则 ROS 节点时间不推进）
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
    ])
