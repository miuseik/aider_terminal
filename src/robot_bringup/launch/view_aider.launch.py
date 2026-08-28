"""在 RViz 中查看 Aider 机器人（新家验收演示）。

启动链路:
    joint_wave (发布关节角)
        → robot_state_publisher (URDF → TF)
            → rviz2 (可视化)

用法:
    ros2 launch robot_bringup view_aider.launch.py
    ros2 launch robot_bringup view_aider.launch.py use_rviz:=false   # 不起 RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'aider', 'aider.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz 可视化'),
        DeclareLaunchArgument(
            'use_joint_wave', default_value='true',
            description='是否用关节摆动驱动模型（否则静止）'),
        DeclareLaunchArgument(
            'wave_amplitude', default_value='0.3',
            description='关节摆动幅度 (rad)'),

        # 关节角发布（驱动模型运动）
        Node(
            package='robot_control',
            executable='joint_wave',
            name='joint_wave',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joint_wave')),
            parameters=[{
                'wave_amplitude': LaunchConfiguration('wave_amplitude'),
            }],
        ),

        # URDF → TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_bringup'),
                'rviz', 'aider.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
