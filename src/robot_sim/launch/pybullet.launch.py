"""PyBullet 仿真启动（老系统仿真器移植到 ROS 2）。

用法:
    ros2 launch robot_sim pybullet.launch.py
    ros2 launch robot_sim pybullet.launch.py use_gui:=false    # 无 X11 环境
    ros2 launch robot_sim pybullet.launch.py with_wave:=true   # 同时启动关节摆动

说明:
    use_gui=false 时使用 PyBullet DIRECT 模式（无窗口，仅计算），
    适用于无 X11 的服务器环境或 CI。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='PyBullet GUI 窗口（需要 X11）'),
        DeclareLaunchArgument(
            'with_wave', default_value='false',
            description='同时启动 joint_wave 驱动关节摆动'),

        # 关节摆动源（可选）
        Node(
            package='robot_control',
            executable='joint_wave',
            name='joint_wave',
            output='screen',
            condition=IfCondition(LaunchConfiguration('with_wave')),
        ),

        # PyBullet 仿真节点
        Node(
            package='robot_sim',
            executable='pybullet_node',
            name='pybullet_node',
            output='screen',
            parameters=[{
                'use_gui': LaunchConfiguration('use_gui'),
                'self_drive': PythonExpression([
                    'not ', LaunchConfiguration('with_wave')
                ]),
                'step_rate': 60.0,
            }],
        ),
    ])
