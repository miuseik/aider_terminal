"""真实 IMU 实况演示 —— 转动陀螺仪，RViz 里的机器人跟着转。

链路:
    串口 IMU (Hiwonder, /dev/ttyUSB0 @460800)
        → imu_hiwonder_node (发布 sensor_msgs/Imu + TF: odom→base_link)
            → robot_state_publisher (URDF 其余关节 → TF)
                → rviz2 (显示)

RViz 的 Fixed Frame 需设为 odom（因 TF 根为 odom）。

用法:
    ros2 launch robot_bringup imu_live.launch.py
    ros2 launch robot_bringup imu_live.launch.py port:=/dev/ttyUSB1

验证:
    ros2 topic echo /sensor/imu
    ros2 topic hz /sensor/imu
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_desc = get_package_share_directory('robot_description')
    pkg_bringup = get_package_share_directory('robot_bringup')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'aider', 'aider.urdf')
    rviz_cfg = os.path.join(pkg_bringup, 'rviz', 'imu_live.rviz')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='IMU 串口设备（默认 by-id 稳定路径，插拔不变）'),
        DeclareLaunchArgument(
            'baud', default_value='460800',
            description='IMU 波特率 (实测 460800)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'wave_amplitude', default_value='0.3',
            description='关节摆动幅度 (rad)，0 则关节静止'),

        # 关节角源 —— robot_state_publisher 缺少 joint_states 时
        # 不会发布 URDF 关节 TF，RViz 中将只剩空的 base_link（黑屏）。
        # 本节点按 URDF 关节名持续发布，使完整模型可见。
        Node(
            package='robot_control',
            executable='joint_wave',
            name='joint_wave',
            output='screen',
            parameters=[{
                'wave_amplitude': LaunchConfiguration('wave_amplitude'),
                'publish_rate': 30.0,
            }],
        ),

        # 真实 IMU 驱动（含 TF 发布，使机器人跟随姿态）
        Node(
            package='robot_sensors',
            executable='imu_hiwonder_node',
            name='imu_hiwonder_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'frame_id': 'imu_link',
                'parent_frame': 'odom',
                'child_frame': 'base_link',
                'publish_tf': True,
                'publish_rate': 100.0,
            }],
        ),

        # URDF 其余关节 → TF（依赖上面的 joint_wave 提供 joint_states）
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
            arguments=['-d', rviz_cfg],
            condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(
                LaunchConfiguration('use_rviz')),
        ),
    ])
