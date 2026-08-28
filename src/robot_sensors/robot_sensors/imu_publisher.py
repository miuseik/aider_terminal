#!/usr/bin/env python3
"""IMU 数据发布节点。

新家第一个真 ROS 节点：以固定频率发布 sensor_msgs/Imu。
未接硬件时输出模拟数据（缓慢摆动），用于验证链路与 RViz 显示。

用法:
    ros2 run robot_sensors imu_publisher
    ros2 run robot_sensors imu_publisher --ros-args -p use_sim:=false
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPublisher(Node):
    """以固定频率发布 IMU 数据。"""

    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('use_sim', True)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('topic', 'sensor/imu')

        self._use_sim = self.get_parameter('use_sim').value
        self._frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('topic').value

        self._publisher = self.create_publisher(Imu, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self._start_time = self.get_clock().now()
        self._seq = 0

        self.get_logger().info(
            f'IMU 发布器已启动 | topic={topic} | rate={rate}Hz | '
            f'frame_id={self._frame_id} | sim={self._use_sim}'
        )

    def _on_timer(self):
        msg = Imu()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id

        elapsed = (now - self._start_time).nanoseconds / 1e9

        if self._use_sim:
            # 模拟数据：姿态缓慢摆动，模拟机器人轻微晃动
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = math.sin(elapsed * 0.5) * 0.1
            msg.orientation.w = math.cos(elapsed * 0.5) * 0.1 + 0.9

            msg.angular_velocity.x = math.sin(elapsed) * 0.02
            msg.angular_velocity.y = math.cos(elapsed * 1.3) * 0.02
            msg.angular_velocity.z = math.sin(elapsed * 0.7) * 0.03

            # 静止时加速度计应指向重力方向 (约 9.81 m/s^2)
            msg.linear_acceleration.x = math.sin(elapsed * 0.8) * 0.05
            msg.linear_acceleration.y = math.cos(elapsed * 1.1) * 0.05
            msg.linear_acceleration.z = 9.81
        else:
            # 真实硬件接入点：在这里读取实际 IMU 驱动
            # 例: data = self._driver.read()
            #     msg.linear_acceleration.x = data['accel']['x'] * 9.80665
            msg.orientation.w = 1.0

        # 协方差：0 表示未知/不可用
        for cov in (msg.orientation_covariance,
                    msg.angular_velocity_covariance,
                    msg.linear_acceleration_covariance):
            for i in range(len(cov)):
                cov[i] = 0.0

        self._publisher.publish(msg)
        self._seq += 1

        # 每 5 秒打印一次，避免刷屏
        if self._seq % int(5.0 * self.get_parameter('publish_rate').value) == 0:
            self.get_logger().info(
                f'已发布 {self._seq} 条 IMU 数据 '
                f'(az={msg.linear_acceleration.z:.2f} m/s²)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
