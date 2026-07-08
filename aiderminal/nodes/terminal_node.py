#!/usr/bin/env python3
"""
ROS2 节点入口 —— 包装现有 aider_terminal 逻辑。
启动后 rclpy 在主线程 spin，原有 asyncio 逻辑在后台线程运行。

用法:
    ros2 run aiderminal terminal_node --ros-args -p robot_type:=aider
"""

import sys
import os
import asyncio
import logging
import threading

import rclpy
from rclpy.node import Node

# 确保项目根目录在 sys.path 中
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from aiderminal.config.settings import TelegripConfig, get_config_data, set_robot_type, get_robot_urdf_path


def _ros_param_to_config(node: Node) -> TelegripConfig:
    """从 ROS 参数直接构造 TelegripConfig，不走 argparse。"""
    config_data = get_config_data()
    config = TelegripConfig()

    # 控制标志
    config.enable_pybullet = not node.get_parameter('no_sim').value
    config.enable_pybullet_gui = config.enable_pybullet and not node.get_parameter('no_viz').value
    config.enable_vr = not node.get_parameter('no_vr').value
    config.enable_keyboard = not node.get_parameter('no_keyboard').value
    config.autoconnect = node.get_parameter('autoconnect').value
    config.log_level = node.get_parameter('log_level').value

    # 机器人类型
    robot_type = node.get_parameter('robot_type').value
    config.robot_type = robot_type
    node.get_logger().info(f'机器人类型: {robot_type}')

    # 设置 URDF 路径
    set_robot_type(robot_type)
    config.urdf_path = get_robot_urdf_path()
    if hasattr(config, 'aloha_urdf_path'):
        from aiderminal.config.settings import get_robot_aloha_urdf_path
        config.aloha_urdf_path = get_robot_aloha_urdf_path()

    # 网络设置
    config.websocket_port = node.get_parameter('ws_port').value
    config.host_ip = node.get_parameter('host').value

    server_host = node.get_parameter('server_host').value
    api_host = node.get_parameter('api_host').value
    env_dev = node.get_parameter('env_dev').value

    if env_dev:
        config.server_host = server_host if server_host else 'localhost'
        config.api_host = api_host if api_host else 'localhost'
    else:
        if server_host:
            config.server_host = server_host
        if api_host:
            config.api_host = api_host

    return config


class TerminalNode(Node):
    """ROS2 节点 —— 在后台线程运行原有 asyncio 遥操作逻辑。"""

    def __init__(self):
        super().__init__('aider_terminal')

        # 声明所有参数
        self.declare_parameter('robot_type', 'aider')
        self.declare_parameter('no_sim', False)
        self.declare_parameter('no_viz', False)
        self.declare_parameter('no_vr', False)
        self.declare_parameter('no_keyboard', False)
        self.declare_parameter('autoconnect', False)
        self.declare_parameter('log_level', 'warning')
        self.declare_parameter('env_dev', False)
        self.declare_parameter('server_host', '')
        self.declare_parameter('api_host', '')
        self.declare_parameter('ws_port', 8442)
        self.declare_parameter('host', '0.0.0.0')

        # 设置日志
        log_level = getattr(logging, self.get_parameter('log_level').value.upper(), logging.WARNING)
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            force=True,
        )
        logging.getLogger('websockets').setLevel(logging.WARNING)

        # 构造配置
        self._config = _ros_param_to_config(self)

        self._running = False
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

        self.get_logger().info('Aider Terminal ROS2 节点已初始化')

    def start_background(self):
        """在后台线程启动原有的 asyncio 逻辑。"""
        self._running = True

        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop
            try:
                from aiderminal.app import TelegripSystem

                system = TelegripSystem(self._config)
                # 不安装 signal handler —— ROS2 已处理 SIGINT
                try:
                    loop.run_until_complete(system.start())
                except (KeyboardInterrupt, SystemExit):
                    pass
                finally:
                    loop.run_until_complete(system.stop())
            except Exception as e:
                self.get_logger().error(f'应用异常: {e}')
            finally:
                loop.close()
                self._running = False

        self._thread = threading.Thread(target=_run, name='app-thread', daemon=True)
        self._thread.start()
        self.get_logger().info('后台应用线程已启动')

    def shutdown(self):
        """关闭节点。"""
        self.get_logger().info('正在关闭...')
        self._running = False
        # 取消后台事件循环
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(lambda: [
                t.cancel() for t in asyncio.all_tasks(self._loop)
            ])


def main():
    rclpy.init()
    node = TerminalNode()
    node.start_background()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
