#!/usr/bin/env python3
"""PyBullet 仿真节点 —— 将老系统的 PyBullet 可视化器接入 ROS 2。

从 aiderminal/robots/aider/visualizer.py 移植，通过订阅 /joint_states
驱动 PyBullet 中的机器人模型，实现与 RViz 并行的第二套可视化。

两种驱动方式:
    1. 订阅 /joint_states（由 robot_control/joint_wave 或其他控制器发布）
    2. 无外部输入时，自行产生正弦摆动（self_drive:=true）

用法:
    ros2 launch robot_sim pybullet.launch.py
    ros2 run robot_sim pybullet_node --ros-args -p use_gui:=false -p self_drive:=true

参数:
    use_gui      是否用 PyBullet GUI（需要 X11；无显示用 DIRECT）
    self_drive   是否自行产生摆动（无外部 joint_states 时）
    urdf_package URDF 所在包（默认 robot_description）
    urdf_path    包内相对路径（默认 urdf/aider/aider.urdf）
    step_rate    仿真步进频率 (Hz)
"""

import math
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .aider_visualizer import AiderVisualizer


class PybulletNode(Node):
    """PyBullet 仿真 ROS 节点。"""

    def __init__(self):
        super().__init__('pybullet_node')

        self.declare_parameter('use_gui', True)
        self.declare_parameter('self_drive', False)
        self.declare_parameter('urdf_package', 'robot_description')
        self.declare_parameter('urdf_path', 'urdf/aider/aider.urdf')
        self.declare_parameter('step_rate', 60.0)
        self.declare_parameter('joint_topic', 'joint_states')

        self._self_drive = self.get_parameter('self_drive').value
        rate = self.get_parameter('step_rate').value

        # 解析 URDF 路径（用 package:// 机制，而非老系统的相对路径）
        pkg = self.get_parameter('urdf_package').value
        rel = self.get_parameter('urdf_path').value
        urdf_path = os.path.join(get_package_share_directory(pkg), rel)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f'URDF 不存在: {urdf_path}')
            raise FileNotFoundError(urdf_path)

        self.get_logger().info(f'加载 URDF: {urdf_path}')

        # 初始化 PyBullet 可视化器（老系统代码，原样复用）
        self._viz = AiderVisualizer(
            urdf_path=urdf_path,
            use_gui=self.get_parameter('use_gui').value,
        )

        if not self._viz.setup():
            self.get_logger().error('PyBullet 初始化失败')
            raise RuntimeError('PyBullet setup failed')

        self.get_logger().info('PyBullet 仿真已就绪')

        # 订阅关节状态
        self._joint_sub = self.create_subscription(
            JointState,
            self.get_parameter('joint_topic').value,
            self._on_joint_state,
            10,
        )

        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self._latest_angles = None
        self._start_time = self.get_clock().now()
        self._frames = 0

    def _on_joint_state(self, msg: JointState):
        """接收关节角（度→弧度转换后存入；visualizer 内部用弧度）。"""
        if not msg.name:
            return
        self._latest_angles = dict(zip(msg.name, msg.position))

    def _on_timer(self):
        if self._self_drive or self._latest_angles is None:
            # 无外部输入：自行摆动
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            angles = {}
            for arm in ('left', 'right'):
                for i in range(1, 9):
                    name = f'{arm}_arm{i}'
                    angles[name] = math.sin(
                        elapsed * 0.5 + i * 0.3 + (0 if arm == 'left' else 1.5)
                    ) * 0.3
            self._latest_angles = angles

        # 更新 PyBullet 中的关节（名称 → visualizer 映射）
        for name, angle_rad in self._latest_angles.items():
            try:
                self._viz.update_body_joint(name, float(angle_rad))
            except Exception:
                pass  # URDF 中不存在的关节名，忽略

        try:
            self._viz.step_simulation()
        except Exception as e:
            self.get_logger().error(f'仿真步进失败: {e}')
            return

        self._frames += 1
        if self._frames % 600 == 0:
            self.get_logger().info(f'仿真运行中 | {self._frames} 帧')

    def destroy_node(self):
        try:
            self._viz.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PybulletNode()
    except Exception as e:
        print(f'PyBullet 节点启动失败: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
