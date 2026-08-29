"""
统一遥操作系统的主入口点。
协调 WebSocket 服务器、机器人接口和输入提供者。
"""

import asyncio
import argparse
import logging
import signal
import sys
import os
import socket
import contextlib
from typing import Optional
import queue  # 添加常规队列用于线程安全通信


def get_local_ip():
    """获取本机的本地 IP 地址。"""
    try:
        # 连接到远程地址以确定本地 IP
        # 这实际上不会发送任何数据
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # 回退：获取主机名 IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # 最终回退
            return "localhost"


# 在函数定义后导入模块
from src.config.settings import TelegripConfig, get_config_data, update_config_data
from src.core.control_loop import ControlLoop
from src.inputs.vr_handler import VRHandler
from src.inputs.exo_handler import ExoHandler
from src.comm.websocket.client import VRWebSocketClient
from src.inputs.keyboard_handler import WebKeyboardHandler
from src.inputs.base import ControlGoal
from src.drivers.webrtc.streamer import WebRTCStreamer
from src.utils.can_setup import setup_can

# Logger will be configured in main() based on command line arguments
logger = logging.getLogger(__name__)


# Note: APIHandler and HTTPSServer classes have been removed.
# Web UI has been migrated to external Vue project (aider_ui).
# All communication now goes through WebSocket via aider_server.


class TelegripSystem:
    """协调所有组件的主遥操作系统。"""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        
        # 命令队列
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)  # 线程安全队列
        
        # 组件
        self.vr_handler = VRHandler(self.command_queue, config)
        # 构造 server API URL（用于拉取外骨骼校准数据）
        _api_host = getattr(config, 'api_host', None) or getattr(config, 'server_host', 'localhost')
        _api_url = f"https://{_api_host}:{config.websocket_port}"
        self.exo_handler = ExoHandler(self.command_queue, server_url=_api_url)
        
        # 初始化 API 路由器
        from src.router.actuator_router import ActuatorRouter
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)
        self.actuator_router = ActuatorRouter(control_loop=self.control_loop)
        
        self.ws_client = VRWebSocketClient(config, self.vr_handler, self.actuator_router, self.control_loop, self.exo_handler)
        
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)

        # WebRTC 推流器（按需启动）
        self.webrtc_streamer = WebRTCStreamer(config)

        # 设置交叉引用
        self.vr_handler.web_keyboard_handler = self.web_keyboard_handler
        self.vr_handler.control_loop = self.control_loop  # ← 注入 control_loop 引用(VR 接管必需)
        self.exo_handler.control_loop = self.control_loop  # ← 注入 control_loop 引用(外骨骼接管必需)
        self.web_keyboard_handler.control_loop = self.control_loop  # ← 注入 control_loop 引用(键盘模式检查)
        self.control_loop.exo_handler = self.exo_handler  # ← 注入 exo_handler 引用(ControlLoop 查询 exo 控制的关节)
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.control_loop.main_app = self  # ← 添加 main_app 引用

        # 为 ESC 键设置断开连接回调
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")
        
        # 任务
        self.tasks = []
        self.is_running = False

    def add_control_command(self, action: str):
        """添加控制命令到队列进行处理。"""
        try:
            command = {"action": action}
            self.control_commands_queue.put_nowait(command)
        except queue.Full:
            print(f"Control commands queue is full, dropping command: {action}")
        except Exception as e:
            print(f"🔌 Error queuing command: {e}")
    
    def add_keypress_command(self, command: dict):
        """添加按键命令到队列进行处理。"""
        try:
            self.control_commands_queue.put_nowait(command)
        except queue.Full:
            print(f"Control commands queue is full, dropping keypress command: {command}")
        except Exception as e:
            print(f"🎮 Error queuing keypress command: {e}")
    
    async def process_control_commands(self):
        """处理来自线程安全队列的控制命令。"""
        try:
            # 从线程安全队列获取所有可用命令
            commands_to_process = []
            while True:
                try:
                    command = self.control_commands_queue.get_nowait()
                    commands_to_process.append(command)
                except queue.Empty:
                    break
            
            # 处理每个命令
            for command in commands_to_process:
                if self.ws_client:
                    await self.ws_client._handle_command(command)
                    
        except Exception as e:
            print(f"处理控制命令时出错: {e}")
    
    async def start(self):
        """启动所有系统组件。"""
        try:
            self.is_running = True

            # 启动时确保 CAN 接口配置正确
            try:
                setup_can()
            except Exception as e:
                logger.warning("CAN setup failed (non-fatal): %s", e)
            
            # HTTPS 服务器已禁用 - UI 已迁移到外部 Vue 项目
            # await self.https_server.start()
            
            # 启动 VR 处理器（无服务器，仅处理器）
            await self.vr_handler.start()

            # 启动外骨骼处理器
            await self.exo_handler.start()

            # 通过 WebSocket 客户端连接到 Aider Server
            # 后台启动：连不上/服务器离线时，重连在后台循环进行，
            # 不阻塞控制循环、键盘、状态推送等其余组件的启动。
            ws_task = asyncio.create_task(self.ws_client.connect())
            self.tasks.append(ws_task)

            # WebRTC 视频推流
            if getattr(self.config, 'enable_webrtc', False):
                print("📹 启动 WebRTC 视频推流...")
                self.webrtc_streamer.set_transport(self.ws_client.transport)
                webrtc_task = asyncio.create_task(self.webrtc_streamer.run())
                self.tasks.append(webrtc_task)
                # 等待 WebRTC 加入房间完成，再启动控制循环（PyBullet 会阻塞事件循环）
                try:
                    await asyncio.wait_for(self.webrtc_streamer._joined.wait(), timeout=30)
                except asyncio.TimeoutError:
                    print("⚠️ WebRTC 加入房间超时，继续启动...")

            # 启动 Web 键盘处理器
            await self.web_keyboard_handler.start()

            # 启动控制循环
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # 启动硬件状态周期推送（业务归属 WebSocket 客户端，由客户端自身维护）
            status_pusher_task = asyncio.create_task(self.ws_client.run_status_pusher())
            self.tasks.append(status_pusher_task)

            # 启动控制命令处理器
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            print("所有系统组件启动成功")

            # 如果请求则自动连接机器人
            if self.config.autoconnect:
                print("🔌 自动连接机器人电机...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
            # 主循环处理重启
            while self.is_running:
                try:
                    # 等待任务完成
                    await asyncio.gather(*self.tasks)
                    # 如果到达这里，所有任务正常完成（在正常操作中不应发生）
                    break
                except asyncio.CancelledError:
                    # 任务被取消 - 检查是否由于重启
                    if self.is_running:
                        # 系统正在重启，等待重启完成
                        await asyncio.sleep(1)
                        # 继续循环等待新任务
                        continue
                    else:
                        # 正常关闭
                        break
                except Exception as e:
                    print(f"主任务循环中出错: {e}")
                    break
            
        except OSError as e:
            if e.errno == 98:  # 地址已被占用
                print(f"启动遥操作系统时出错: {e}")
                print(f"要查找并终止使用该端口的进程，请运行:")
                print(f"  kill -9 $(lsof -t -i:{self.config.websocket_port})")
            else:
                print(f"启动遥操作系统时出错: {e}")
            await self.stop()
            raise
        except Exception as e:
            print(f"启动遥操作系统时出错: {e}")
            await self.stop()
            raise
    
    async def _run_command_processor(self):
        """运行控制命令处理器循环。"""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)  # 每 50ms 检查一次命令

    async def stop(self):
        """停止所有系统组件。"""
        print("正在关闭遥操作系统...")
        self.is_running = False

        # 首先停止 VR 服务器以关闭 websocket 连接（解除任何等待的处理程序的阻塞）
        try:
            await asyncio.wait_for(self.ws_client.disconnect(), timeout=2.0)
        except asyncio.TimeoutError:
            print("VR WebSocket 客户端断开超时")
        except Exception as e:
            print(f"断开 VR WebSocket 客户端时出错: {e}")
        
        try:
            await asyncio.wait_for(self.vr_handler.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            print("VR 处理器停止超时")
        except Exception as e:
            print(f"停止 VR 处理器时出错: {e}")

        try:
            await asyncio.wait_for(self.exo_handler.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            print("外骨骼处理器停止超时")
        except Exception as e:
            print(f"停止外骨骼处理器时出错: {e}")

        # 取消所有任务
        for task in self.tasks:
            task.cancel()

        # 等待任务完成并设置超时
        if self.tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*self.tasks, return_exceptions=True),
                    timeout=2.0
                )
            except asyncio.TimeoutError:
                print("部分任务未在超时时间内完成")

        # 停止剩余组件
        try:
            await asyncio.wait_for(self.control_loop.stop(), timeout=3.0)
        except asyncio.TimeoutError:
            print("控制循环停止超时")
        except Exception as e:
            print(f"停止控制循环时出错: {e}")

        try:
            await asyncio.wait_for(self.web_keyboard_handler.stop(), timeout=1.0)
        except asyncio.TimeoutError:
            print("Web 键盘处理器停止超时")
        except Exception as e:
            print(f"停止 Web 键盘处理器时出错: {e}")

        print("遥操作系统关闭完成")


def create_signal_handler(system: 'TelegripSystem', loop: asyncio.AbstractEventLoop):
    """创建一个正确停止系统的信号处理器。"""
    def signal_handler(signum, frame):
        """处理关闭信号。"""
        print(f"收到信号 {signum}")
        system.is_running = False
        # 从事件循环中取消所有任务
        for task in system.tasks:
            loop.call_soon_threadsafe(task.cancel)
        # 抛出 SystemExit 以跳出阻塞操作
        raise SystemExit(0)
    return signal_handler


def parse_arguments():
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")
    
    # 控制标志
    parser.add_argument("--no-sim", action="store_true", help="Disable PyBullet simulation and inverse kinematics")
    parser.add_argument("--no-viz", action="store_true", help="Disable PyBullet visualization (headless mode)")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR WebSocket server")
    parser.add_argument("--no-keyboard", action="store_true", help="Disable keyboard input")
    parser.add_argument("--autoconnect", action="store_true", help="Automatically connect to robot motors on startup")
    parser.add_argument("--log-level", default="warning", 
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="Set logging level (default: warning)")
    
    # 网络设置
    parser.add_argument("--ws-port", type=int, default=8442, help="WebSocket server port")
    parser.add_argument("--host", default="0.0.0.0", help="Host IP address")
    parser.add_argument("--server-host", default=None, help="WebSocket Server host (overrides config)")
    parser.add_argument("--api-host", default=None, help="API Server host (overrides config)")
    parser.add_argument("--env-dev", action="store_true", help="Use development environment (192.168.0.106)")
    
    # 路径
    parser.add_argument("--urdf", default="URDF/SO100/so100.urdf", help="Path to robot URDF file")
    
    # 机器人设置
    parser.add_argument("--config", default="config/config.yaml", help="Path to config file")
    parser.add_argument("--left-port", help="Left arm serial port (overrides config file)")
    parser.add_argument("--right-port", help="Right arm serial port (overrides config file)")
    parser.add_argument("--robot-type", default=None, choices=["aider", "aloha", "openarmx", "custom"],
                       help="Robot type to control (overrides config file)")
    parser.add_argument("--role", default=None, choices=["aider", "aloha", "openarmx", "custom"],
                       help="Alias for --robot-type (which robot to control)")
    parser.add_argument("--role-aider", dest="role_aider", action="store_true",
                       help="Shortcut for --role aider")
    parser.add_argument("--role-aloha", dest="role_aloha", action="store_true",
                       help="Shortcut for --role aloha")
    
    return parser.parse_args()


def create_config_from_args(args) -> TelegripConfig:
    """从命令行参数创建配置对象。"""
    config_data = get_config_data()
    config = TelegripConfig()
    
    # 应用命令行覆盖
    config.enable_pybullet = not args.no_sim
    config.enable_pybullet_gui = config.enable_pybullet and not args.no_viz
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    config.autoconnect = args.autoconnect
    config.log_level = args.log_level
    
    config.websocket_port = args.ws_port
    config.host_ip = args.host
    
    if args.server_host:
        config.server_host = args.server_host
    if args.api_host:
        config.api_host = args.api_host
    
    # 处理机器人类型 - 优先级: --role > --role-aider/--role-aloha > --robot-type > 配置文件
    if args.role:
        config.robot_type = args.role
        print(f"🤖 机器人类型: {args.role} (--role 指定)")
    elif getattr(args, 'role_aider', False):
        config.robot_type = "aider"
        print(f"🤖 机器人类型: aider (--role-aider)")
    elif getattr(args, 'role_aloha', False):
        config.robot_type = "aloha"
        print(f"🤖 机器人类型: aloha (--role-aloha)")
    elif args.robot_type:
        config.robot_type = args.robot_type
        print(f"🤖 机器人类型: {args.robot_type} (--robot-type 指定)")
    else:
        config.robot_type = config_data.get("robot", {}).get("type", "aider")
        print(f"🤖 机器人类型: {config.robot_type} (配置文件)")
    
    # 根据机器人类型设置 URDF 路径
    from src.config.settings import set_robot_type, get_robot_urdf_path, get_robot_aloha_urdf_path
    set_robot_type(config.robot_type)
    config.urdf_path = args.urdf if args.urdf != "URDF/SO100/so100.urdf" else get_robot_urdf_path()
    if hasattr(config, 'aloha_urdf_path'):
        config.aloha_urdf_path = get_robot_aloha_urdf_path()
    
    # 处理端口配置
    if args.left_port or args.right_port:
        config.follower_ports = {
            "left": args.left_port if args.left_port else config_data["robot"]["left_arm"]["port"],
            "right": args.right_port if args.right_port else config_data["robot"]["right_arm"]["port"]
        }
    
    return config


async def main():
    """主入口点。"""
    # 首先解析参数以检查日志级别
    args = parse_arguments()
    
    # 根据日志级别设置日志记录
    log_level = getattr(logging, args.log_level.upper())
    
    # 统一使用详细的日志格式，确保所有 logger 输出都能看到
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        force=True  # 强制重新配置，覆盖之前的设置
    )

    # 抑制嘈杂的 websockets 库日志（对 WS 端口的无效 HTTP 请求）
    logging.getLogger('websockets').setLevel(logging.WARNING)

    config = create_config_from_args(args)

    # SSL 证书检查已删除 - HTTPS 服务器已禁用（UI 已迁移到外部 Vue 项目）

    # 记录配置（仅在 INFO 级别或更详细时）
    if log_level <= logging.INFO:
        print("使用以下配置启动:")
        print(f"  机器人类型: {config.robot_type}")
        print(f"  PyBullet: {'启用' if config.enable_pybullet else '禁用'}")
        print(f"  无头模式: {'启用' if not config.enable_pybullet_gui and config.enable_pybullet else '禁用'}")
        print(f"  VR: {'启用' if config.enable_vr else '禁用'}")
        print(f"  键盘: {'启用' if config.enable_keyboard else '禁用'}")
        print(f"  自动连接: {'启用' if config.autoconnect else '禁用'}")
        print(f"  WebSocket 端口: {config.websocket_port}")
        print(f"  机器人端口: {config.follower_ports}")
    else:
        # 显示干净的启动消息
        print(f"🤖 aider_terminal 启动中... (机器人类型: {config.robot_type})")
        print(f"💡 使用 --log-level info 查看详细输出")
        print()
    
    # 创建并启动遥操作系统
    system = TelegripSystem(config)

    # 设置信号处理器，引用系统和事件循环
    loop = asyncio.get_event_loop()
    signal_handler = create_signal_handler(system, loop)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        await system.start()
    except (KeyboardInterrupt, SystemExit):
        if log_level <= logging.INFO:
            print("收到中断信号")
        else:
            print("\n🛑 正在关闭...")
    except asyncio.CancelledError:
        # 处理取消错误（通常来自重启场景）
        if log_level <= logging.INFO:
            print("系统任务已取消")
    except Exception as e:
        if log_level <= logging.INFO:
            print(f"系统错误: {e}")
        else:
            print(f"❌ 错误: {e}")
    finally:
        try:
            await system.stop()
        except (asyncio.CancelledError, SystemExit):
            # 在关闭期间忽略取消/退出错误
            pass

        # 在事件循环清理期间抑制 SSL 传输错误
        def ignore_ssl_errors(loop, context):
            # 在关闭期间忽略"错误的文件描述符"和"事件循环已关闭"错误
            if 'exception' in context:
                exc = context['exception']
                if isinstance(exc, (OSError, RuntimeError)):
                    return
            # 正常记录其他错误
            loop.default_exception_handler(context)

        loop.set_exception_handler(ignore_ssl_errors)

        if log_level > logging.INFO:
            print("✅ 关闭完成。")


def main_cli():
    """pip 安装包的控制台脚本入口点。"""
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, SystemExit):
        print("\n关闭完成。")
    except asyncio.CancelledError:
        # 处理重启场景中的取消错误
        pass
    except Exception as e:
        print(f"致命错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli()