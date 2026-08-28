#!/usr/bin/env python3
"""真实 IMU 节点 —— 读取 Hiwonder 串口陀螺仪并发布 ROS 消息。

功能:
    1. 发布 sensor_msgs/Imu（含姿态四元数、角速度、加速度）
    2. 发布 TF: <parent_frame> -> <frame_id>，姿态跟随陀螺仪

发布 TF 是为了在 RViz 中直接看到机器人的姿态随陀螺仪变化——
转动陀螺仪，RViz 里的机器人跟着转。

用法:
    ros2 run robot_sensors imu_hiwonder_node
    ros2 run robot_sensors imu_hiwonder_node --ros-args -p port:=/dev/ttyUSB1

参数:
    port         串口设备 (默认 /dev/ttyUSB0)
    baud         波特率 (默认 460800，实测值)
    frame_id     IMU 坐标系名 (默认 imu_link)
    parent_frame TF 父坐标系 (默认 odom)
    publish_tf   是否发布 TF (默认 true)
    topic        IMU 话题 (默认 sensor/imu)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from .imu_hiwonder import HiwonderImu, euler_to_quat


class ImuHiwonderNode(Node):
    """真实 IMU 驱动节点。"""

    def __init__(self):
        super().__init__('imu_hiwonder_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 100.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self._frame_id = self.get_parameter('frame_id').value
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value
        self._pub_tf = self.get_parameter('publish_tf').value
        rate = self.get_parameter('publish_rate').value

        self._publisher = self.create_publisher(Imu, 'sensor/imu', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 连接硬件
        self._imu = HiwonderImu(port, baud)
        try:
            self._imu.connect()
            self.get_logger().info(f'IMU 已连接: {port} @{baud}')
        except Exception as e:
            self.get_logger().error(f'IMU 连接失败 ({port}): {e}')
            self.get_logger().error('将退出。请检查设备是否存在及权限(dialout 组)')
            raise

        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self._count = 0
        self._no_data_warned = False

    def _on_timer(self):
        data = self._imu.read()
        if data is None:
            if not self._no_data_warned:
                self.get_logger().warn('等待 IMU 数据...')
                self._no_data_warned = True
            return

        now = self.get_clock().now()

        # ── 构造 Imu 消息 ──
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id

        if data['angle']:
            q = euler_to_quat(
                data['angle']['roll'],
                data['angle']['pitch'],
                data['angle']['yaw'],
            )
            msg.orientation.x, msg.orientation.y = q[0], q[1]
            msg.orientation.z, msg.orientation.w = q[2], q[3]
        else:
            msg.orientation.w = 1.0

        if data['gyro']:
            # °/s -> rad/s
            k = math.pi / 180.0
            msg.angular_velocity.x = data['gyro']['x'] * k
            msg.angular_velocity.y = data['gyro']['y'] * k
            msg.angular_velocity.z = data['gyro']['z'] * k

        if data['accel']:
            # g -> m/s²
            k = 9.80665
            msg.linear_acceleration.x = data['accel']['x'] * k
            msg.linear_acceleration.y = data['accel']['y'] * k
            msg.linear_acceleration.z = data['accel']['z'] * k

        # 协方差未知
        for cov in (msg.orientation_covariance,
                    msg.angular_velocity_covariance,
                    msg.linear_acceleration_covariance):
            for i in range(len(cov)):
                cov[i] = 0.0

        self._publisher.publish(msg)

        # ── 发布 TF（让 RViz 中机器人跟随陀螺仪姿态）──
        # 注意: 发布 <parent> -> <child_frame(默认 base_link)>，
        # 使整个机器人跟随陀螺仪旋转。Imu 消息本身仍标记 imu_link。
        if self._pub_tf and data['angle']:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._parent
            t.child_frame_id = self._child
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = msg.orientation.x
            t.transform.rotation.y = msg.orientation.y
            t.transform.rotation.z = msg.orientation.z
            t.transform.rotation.w = msg.orientation.w
            self._tf_broadcaster.sendTransform(t)

        # 降频日志
        self._count += 1
        if self._count % 200 == 0 and data['angle']:
            a = data['angle']
            tag = ' (加速度由姿态反推)' if data.get('accel_derived') else ''
            self.get_logger().info(
                f"已发布 {self._count} 条 | "
                f"R={a['roll']:7.2f}° P={a['pitch']:7.2f}° Y={a['yaw']:7.2f}°"
                f"{tag}"
            )

    def destroy_node(self):
        self._imu.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuHiwonderNode()
    except Exception:
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
