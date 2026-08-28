#!/usr/bin/env python3
"""关节摆动节点 —— 让 RViz 里的机器人动起来。

按 URDF 关节名发布 sensor_msgs/JointState，驱动 robot_state_publisher
生成 TF，从而在 RViz 中看到机器人做正弦摆动。

这是「新家能跑」的可视化验证节点，后续由真实控制器替代。

用法:
    ros2 run robot_control joint_wave
    ros2 run robot_control joint_wave --ros-args -p wave_amplitude:=0.5
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Aider URDF 中的关节名（与 robot_description 保持一致）
JOINT_NAMES = [
    # 躯干
    'lift_Link', 'waist_Link', 'head_Link', 'head_Link2',
    # 右臂 (arm1..arm8 + 夹爪 arm12)
    'right_arm1', 'right_arm2', 'right_arm3', 'right_arm4',
    'right_arm5', 'right_arm6', 'right_arm7', 'right_arm8', 'right_arm12',
    # 左臂
    'left_arm1', 'left_arm2', 'left_arm3', 'left_arm4',
    'left_arm5', 'left_arm6', 'left_arm7', 'left_arm8', 'left_arm12',
    # 底盘四轮
    'whel_Link1', 'whel_Link2', 'whel_Link3', 'whel_Link4',
]


class JointWave(Node):
    """发布正弦摆动关节角，驱动 RViz 中的模型运动。"""

    def __init__(self):
        super().__init__('joint_wave')

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('wave_amplitude', 0.35)
        self.declare_parameter('wave_frequency', 0.4)
        self.declare_parameter('topic', 'joint_states')
        self.declare_parameter('wave_joints', True)

        rate = self.get_parameter('publish_rate').value
        self._amp = self.get_parameter('wave_amplitude').value
        self._freq = self.get_parameter('wave_frequency').value
        self._wave = self.get_parameter('wave_joints').value
        topic = self.get_parameter('topic').value

        self._publisher = self.create_publisher(JointState, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self._start_time = self.get_clock().now()

        self.get_logger().info(
            f'关节摆动节点已启动 | {len(JOINT_NAMES)} 个关节 | '
            f'{rate}Hz | 幅度={self._amp} 频率={self._freq} | wave={self._wave}'
        )

    def _on_timer(self):
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds / 1e9

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(JOINT_NAMES)

        positions = []
        for i, _name in enumerate(JOINT_NAMES):
            if not self._wave:
                positions.append(0.0)
            else:
                # 每个关节相位错开，形成波浪式摆动
                phase = elapsed * self._freq * 2.0 * math.pi + i * 0.25
                positions.append(math.sin(phase) * self._amp)

        msg.position = positions
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointWave()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
