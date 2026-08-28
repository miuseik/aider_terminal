"""最小系统 —— 只启动 RViz 显示网格(Grid)，不带任何传感器/机器人节点。

目的: 隔离验证 RViz 的 OpenGL 渲染链路是否正常。
  - Grid 不依赖 TF / URDF / 任何 topic，只要渲染管线通就能看到网格。
  - Fixed Frame 用 map（RViz 内置虚拟 frame，永远存在），确保 Grid 必定渲染。

用法:
    ros2 launch robot_bringup grid_only.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    rviz_cfg = os.path.join(pkg_bringup, 'rviz', 'grid_only.rviz')

    return LaunchDescription([
        # 真实陀螺仪 IMU 节点（只输出角速度时，内部积分得到姿态）
        Node(
            package='robot_sensors',
            executable='imu_hiwonder_node',
            name='imu_hiwonder_node',
            output='screen',
            parameters=[{
                'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baud': 460800,
                'publish_rate': 100.0,
            }],
        ),
        # 测试方块：跟随 /sensor/imu 姿态旋转
        Node(
            package='robot_bringup',
            executable='test_box_node',
            name='test_box_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])

