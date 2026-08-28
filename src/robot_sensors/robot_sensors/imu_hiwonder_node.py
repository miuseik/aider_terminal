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
from geometry_msgs.msg import TransformStamped, Quaternion

from .imu_hiwonder import HiwonderImu, euler_to_quat


class ImuHiwonderNode(Node):
    """真实 IMU 驱动节点。

    姿态来源（按优先级）:
        1. 设备输出 0x53 姿态角帧 → 直接用欧拉角转四元数（无漂移）
        2. 设备只输出 0x52 角速度帧 → 对角速度数值积分得到四元数
           （会有零偏累积漂移，但足以直观验证陀螺仪方向映射）
    """

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

        # 角速度积分得到的姿态四元数（仅当设备无 0x53 帧时使用）
        self._integ_quat = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w
        self._last_integ_time = None

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

    def _integrate_gyro(self, gx, gy, gz, now):
        """对角速度(rad/s)数值积分更新 self._integ_quat。

        采用一阶近似: q_{k+1} = q_k + 0.5 * (Ω ⊗ q_k) * dt，
        其中 Ω = (gx,gy,gz,0)。积分后归一化。
        """
        if self._last_integ_time is None:
            self._last_integ_time = now
            return
        dt = (now - self._last_integ_time).nanoseconds * 1e-9
        self._last_integ_time = now
        if dt <= 0.0:
            return

        qx, qy, qz, qw = self._integ_quat
        # Ω ⊗ q = (wx,qw 形式)
        # Ω = (gx, gy, gz, 0)
        ox = gx
        oy = gy
        oz = gz
        ow = 0.0
        # 四元数乘法 Ω * q
        nx = ow * qx + ox * qw + oy * qz - oz * qy
        ny = ow * qy - ox * qz + oy * qw + oz * qx
        nz = ow * qz + ox * qy - oy * qx + oz * qw
        nw = ow * qw - ox * qx - oy * qy - oz * qz
        # 半角增量
        self._integ_quat[0] = qx + 0.5 * nx * dt
        self._integ_quat[1] = qy + 0.5 * ny * dt
        self._integ_quat[2] = qz + 0.5 * nz * dt
        self._integ_quat[3] = qw + 0.5 * nw * dt
        # 归一化
        n = math.sqrt(sum(c * c for c in self._integ_quat))
        if n > 0.0:
            self._integ_quat = [c / n for c in self._integ_quat]

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

        # 角速度 (°/s -> rad/s)
        gx = gy = gz = 0.0
        if data['gyro']:
            k = math.pi / 180.0
            gx = data['gyro']['x'] * k
            gy = data['gyro']['y'] * k
            gz = data['gyro']['z'] * k
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # 姿态四元数: 优先用 0x53 帧；无则用角速度积分
        if data['angle']:
            a = data['angle']
            # 诊断: 打印原始角度，确认是否稳定
            if self._count % 50 == 0:
                self.get_logger().info(
                    f"[diag] angle= R={a['roll']:.2f} P={a['pitch']:.2f} Y={a['yaw']:.2f}")
            q = euler_to_quat(a['roll'], a['pitch'], a['yaw'])
            msg.orientation.x, msg.orientation.y = q[0], q[1]
            msg.orientation.z, msg.orientation.w = q[2], q[3]
            # 有绝对姿态时，重置积分器基准，避免漂移累积
            self._integ_quat = [q[0], q[1], q[2], q[3]]
            self._last_integ_time = None
        else:
            # 仅角速度：积分得到姿态
            self._integrate_gyro(gx, gy, gz, now)
            msg.orientation.x = self._integ_quat[0]
            msg.orientation.y = self._integ_quat[1]
            msg.orientation.z = self._integ_quat[2]
            msg.orientation.w = self._integ_quat[3]

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
        # 发布 <parent> -> <child_frame(默认 base_link)>，使方块/机器人跟随姿态。
        if self._pub_tf:
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
        if self._count % 200 == 0:
            tag = ' (角速度积分)' if not data['angle'] else ''
            self.get_logger().info(
                f"已发布 {self._count} 条 | "
                f"陀螺仪 ω=({gx:.3f},{gy:.3f},{gz:.3f}) rad/s"
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
