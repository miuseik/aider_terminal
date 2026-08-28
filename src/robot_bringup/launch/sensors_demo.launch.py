"""传感器数据链路演示（新家第一个真节点）。

启动 IMU 发布器，验证 ROS topic 通信链路。

用法:
    ros2 launch robot_bringup sensors_demo.launch.py

验证:
    ros2 topic echo /sensor/imu
    ros2 topic hz /sensor/imu
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='使用模拟数据（未接硬件）'),
        DeclareLaunchArgument(
            'publish_rate', default_value='50.0',
            description='IMU 发布频率 (Hz)'),

        Node(
            package='robot_sensors',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'use_sim': LaunchConfiguration('use_sim'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),
    ])
