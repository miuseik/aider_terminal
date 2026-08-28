"""测试方块节点 —— 通过一个 TF (box_frame) 跟随陀螺仪(IMU)姿态旋转。

用途: 在没有完整机器人 URDF 的情况下，用简单方块验证 IMU → RViz 姿态链路。
  方案: 发布 TF  map -> box_frame (姿态 = IMU orientation)，并发布一个
        frame_id=box_frame 的立方体 Marker + 在 RViz 用 Axes 显示 box_frame。
        方块姿态完全由 TF 决定，不依赖 Marker 消息里的 orientation 字段，
        避免 Marker/MakerArray 类型不匹配等坑。

用法:
    ros2 run robot_bringup test_box_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class TestBoxNode(Node):
    def __init__(self):
        super().__init__('test_box_node')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('box_size', 0.3)
        self.declare_parameter('publish_rate', 30.0)

        self.parent_frame = self.get_parameter('frame_id').value
        self.box_frame = 'box_frame'  # 方块所在坐标系
        self.box_size = float(self.get_parameter('box_size').value)
        rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # 发布 TF: parent_frame -> box_frame
        self.tf_broadcaster = TransformBroadcaster(self)

        # 发布立方体 Marker (frame_id = box_frame, 姿态由 TF 决定)
        self.marker_pub = self.create_publisher(Marker, '/test_box', 10)

        # 订阅 IMU 姿态
        self.latest_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.imu_sub = self.create_subscription(
            Imu, '/sensor/imu', self.imu_callback, qos)

        self.timer = self.create_timer(1.0 / rate, self.publish_all)

        self.get_logger().info(
            f'测试方块节点已启动: 发布 TF {self.parent_frame}->{self.box_frame}, '
            f'订阅 /sensor/imu')

    def imu_callback(self, msg: Imu):
        self.latest_orientation = msg.orientation

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        q = self.latest_orientation

        # 1) TF: parent -> box_frame
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.box_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.box_size / 2.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 2) 立方体 Marker (frame_id=box_frame, 不写 orientation)
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = self.box_frame
        m.ns = 'test_box'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = self.box_size
        m.scale.y = self.box_size
        m.scale.z = self.box_size
        m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TestBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
