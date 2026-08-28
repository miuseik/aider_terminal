"""可视化机器人模型（标准 ROS2 做法：模型包自带 display launch）。

通过 xacro 加载 URDF，支持 robot_type 参数选择机型。
这是 robot_description 包的"自检"入口，不依赖 robot_bringup。

用法:
    ros2 launch robot_description display.launch.py
    ros2 launch robot_description display.launch.py robot_type:=aider
    ros2 launch robot_description display.launch.py use_joint_state_gui:=true
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    default_xacro = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'aider.rviz')

    # 用 xacro 实时展开（而非直接读 .urdf），符合 ROS2 标准做法。
    # 必须用 ParameterValue(value_type=str) 包裹：Command 输出会被
    # 当作 YAML 解析，而 URDF 以 <?xml 开头会触发 YAML 语法错误。
    robot_description_content = ParameterValue(
        Command([
            'xacro ', default_xacro,
            ' robot_type:=', LaunchConfiguration('robot_type'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type', default_value='aider',
            description='机型: aider / aloha / so100'),
        DeclareLaunchArgument(
            'use_joint_state_gui', default_value='false',
            description='是否启动 joint_state_publisher_gui 手动拖关节'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz'),

        # 关节状态发布器（GUI 版可手动拖动关节）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('use_joint_state_gui'))
        ),

        # URDF(xacro展开) → TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
