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
import threading


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


@contextlib.contextmanager
def suppress_stdout_stderr():
    """上下文管理器，在文件描述符级别抑制标准输出和错误输出。"""
    # 保存原始文件描述符
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # 保存原始文件描述符
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # 打开 /dev/null
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # 将标准输出和错误重定向到 /dev/null
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # 恢复原始文件描述符
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # 关闭保存的文件描述符
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


# 在函数定义后导入 telegrip 模块
from .config import TelegripConfig, get_config_data, update_config_data
from .control_loop import ControlLoop
from .inputs.vr_handler import VRHandler
from .inputs.ws_client import VRWebSocketClient
from .inputs.webrtc_streamer import WebRTCStreamer
from .inputs.web_keyboard import WebKeyboardHandler
from .inputs.base import ControlGoal

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
        
        # 初始化 API 路由器
        from router.motor_router import MotorRouter
        self.motor_router = MotorRouter(control_loop=None)  # control_loop 稍后设置
        
        self.vr_ws_client = VRWebSocketClient(config, self.vr_handler, self.motor_router)
        MotorRouter.set_ws_client(self.vr_ws_client)  # 设置 ws_client 引用
        
        # 初始化 WebRTC 推流器
        self.webrtc_streamer = WebRTCStreamer(self.vr_ws_client, config)
        self.vr_ws_client.webrtc_streamer = self.webrtc_streamer  # 关联到 ws_client
        
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)
        
        # 设置 control_loop 到 motor_router
        self.motor_router.control_loop = self.control_loop

        # 设置交叉引用
        self.vr_handler.web_keyboard_handler = self.web_keyboard_handler
        self.vr_handler.control_loop = self.control_loop  # ← 添加 control_loop 引用
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.control_loop.main_app = self  # ← 添加 main_app 引用

        # 为 ESC 键设置断开连接回调
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")
        
        # 任务
        self.tasks = []
        self.is_running = False
        self.main_loop = None  # 系统启动时将设置
    
    def add_control_command(self, action: str):
        """添加控制命令到队列进行处理。"""
        try:
            command = {"action": action}
            logger.info(f"🔌 Queueing control command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🔌 Command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping command: {action}")
        except Exception as e:
            logger.error(f"🔌 Error queuing command: {e}")
    
    def add_keypress_command(self, command: dict):
        """添加按键命令到队列进行处理。"""
        try:
            logger.info(f"🎮 Queueing keypress command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🎮 Keypress command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping keypress command: {command}")
        except Exception as e:
            logger.error(f"🎮 Error queuing keypress command: {e}")
    
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
                if self.control_loop:
                    await self.control_loop._handle_command(command)
                    
        except Exception as e:
            logger.error(f"处理控制命令时出错: {e}")
    
    def restart(self):
        """重启遥操作系统。"""
        def do_restart():
            try:
                logger.info("正在启动系统重启...")
                # 使用存储的主事件循环引用来调度软重启
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    # 等待重启完成
                    future.result(timeout=30.0)
                else:
                    logger.error("主事件循环不可用，无法重启")
            except Exception as e:
                logger.error(f"重启过程中出错: {e}")
        
        # 在单独的线程中运行重启以避免阻塞 HTTP 响应
        restart_thread = threading.Thread(target=do_restart, daemon=True)
        restart_thread.start()
    
    async def _soft_restart_sequence(self):
        """通过重新初始化组件执行软重启，而不退出进程。"""
        try:
            logger.info("开始软重启序列...")
            
            # 等待片刻让 HTTP 响应发送
            await asyncio.sleep(1)
            
            # 取消所有任务
            for task in self.tasks:
                task.cancel()
            
            # 等待任务完成并设置超时
            if self.tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*self.tasks, return_exceptions=True), 
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    logger.warning("部分任务未在超时时间内完成")
            
            # 按相反顺序停止组件
            await self.control_loop.stop()
            await self.web_keyboard_handler.stop()
            await self.vr_ws_client.disconnect()
            await self.vr_handler.stop()
            # HTTPS server disabled - UI migrated to external Vue project

            # 等待清理
            await asyncio.sleep(1)

            # 从文件重新加载配置但保留命令行覆盖
            from .config import get_config_data
            file_config = get_config_data()
            logger.info("已从文件重新加载配置")

            # 保留现有的配置对象以保持命令行参数
            # 只更新配置文件中可能已更改的特定值

            # 使用现有配置重新创建组件
            self.command_queue = asyncio.Queue()
            self.control_commands_queue = queue.Queue(maxsize=10)

            # 创建新组件
            self.vr_handler = VRHandler(self.command_queue, self.config)
            
            # 重新初始化 API 路由器
            from router.motor_router import MotorRouter
            self.motor_router = MotorRouter(control_loop=None)
            
            self.vr_ws_client = VRWebSocketClient(self.config, self.vr_handler, self.motor_router)
            MotorRouter.set_ws_client(self.vr_ws_client)
            
            # 重新初始化 WebRTC 推流器
            self.webrtc_streamer = WebRTCStreamer(self.vr_ws_client, self.config)
            self.vr_ws_client.webrtc_streamer = self.webrtc_streamer
            
            self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, self.config)
            self.control_loop = ControlLoop(self.command_queue, self.config, self.control_commands_queue)
            
            # 设置 control_loop 到 motor_router
            self.motor_router.control_loop = self.control_loop

            # 设置交叉引用
            self.vr_handler.web_keyboard_handler = self.web_keyboard_handler
            self.vr_handler.control_loop = self.control_loop  # ← 添加 control_loop 引用
            self.control_loop.web_keyboard_handler = self.web_keyboard_handler
            self.control_loop.main_app = self  # ← 添加 main_app 引用

            # 为 ESC 键设置断开连接回调
            self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

            # 清除旧任务
            self.tasks = []

            # 启动 VR 处理器（无服务器，仅处理器）
            await self.vr_handler.start()

            # 通过 WebSocket 客户端连接到 Aider Server
            await self.vr_ws_client.connect()
            
            # WebRTC 视频推流改为按需启动(前端请求时才开启)
            # if getattr(self.config, 'enable_webrtc', False):
            #     logger.info("📹 启动 WebRTC 视频推流...")
            #     await self.webrtc_streamer.start_streaming()

            # 启动 Web 键盘处理器
            await self.web_keyboard_handler.start()

            # 启动控制循环
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # 启动控制命令处理器
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("系统重启成功完成")

            # 如果请求则自动连接机器人（在重启后保留自动连接行为）
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 重启后自动连接机器人电机...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
        except Exception as e:
            logger.error(f"软重启序列期间出错: {e}")
            raise
    
    async def start(self):
        """启动所有系统组件。"""
        try:
            self.is_running = True
            
            # 存储主事件循环引用以用于重启功能
            self.main_loop = asyncio.get_event_loop()
            
            # HTTPS 服务器已禁用 - UI 已迁移到外部 Vue 项目
            # await self.https_server.start()
            
            # 启动 VR 处理器（无服务器，仅处理器）
            await self.vr_handler.start()

            # 通过 WebSocket 客户端连接到 Aider Server
            await self.vr_ws_client.connect()

            # WebRTC 视频推流改为按需启动(前端请求时才开启)
            # if getattr(self.config, 'enable_webrtc', False):
            #     logger.info("📹 启动 WebRTC 视频推流...")
            #     await self.webrtc_streamer.start_streaming()

            # 启动 Web 键盘处理器
            await self.web_keyboard_handler.start()

            # 启动控制循环
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # 启动控制命令处理器
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("所有系统组件启动成功")

            # 如果请求则自动连接机器人
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 自动连接机器人电机...")
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
                    logger.error(f"主任务循环中出错: {e}")
                    break
            
        except OSError as e:
            if e.errno == 98:  # 地址已被占用
                logger.error(f"启动遥操作系统时出错: {e}")
                logger.error(f"要查找并终止使用该端口的进程，请运行:")
                logger.error(f"  kill -9 $(lsof -t -i:{self.config.websocket_port})")
            else:
                logger.error(f"启动遥操作系统时出错: {e}")
            await self.stop()
            raise
        except Exception as e:
            logger.error(f"启动遥操作系统时出错: {e}")
            await self.stop()
            raise
    
    async def _run_command_processor(self):
        """运行控制命令处理器循环。"""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)  # 每 50ms 检查一次命令
    
    async def stop(self):
        """停止所有系统组件。"""
        logger.info("正在关闭遥操作系统...")
        self.is_running = False

        # 首先停止 VR 服务器以关闭 websocket 连接（解除任何等待的处理程序的阻塞）
        try:
            await asyncio.wait_for(self.vr_ws_client.disconnect(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("VR WebSocket 客户端断开超时")
        except Exception as e:
            logger.warning(f"断开 VR WebSocket 客户端时出错: {e}")
        
        # 停止 WebRTC 推流
        if self.webrtc_streamer:
            try:
                await self.webrtc_streamer.stop_streaming()
            except Exception as e:
                logger.warning(f"停止 WebRTC 推流器时出错: {e}")

        try:
            await asyncio.wait_for(self.vr_handler.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("VR 处理器停止超时")
        except Exception as e:
            logger.warning(f"停止 VR 处理器时出错: {e}")

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
                logger.warning("部分任务未在超时时间内完成")

        # 停止剩余组件
        try:
            await asyncio.wait_for(self.control_loop.stop(), timeout=3.0)
        except asyncio.TimeoutError:
            logger.warning("控制循环停止超时")
        except Exception as e:
            logger.warning(f"停止控制循环时出错: {e}")

        try:
            await asyncio.wait_for(self.web_keyboard_handler.stop(), timeout=1.0)
        except asyncio.TimeoutError:
            logger.warning("Web 键盘处理器停止超时")
        except Exception as e:
            logger.warning(f"停止 Web 键盘处理器时出错: {e}")

        logger.info("遥操作系统关闭完成")


def create_signal_handler(system: 'TelegripSystem', loop: asyncio.AbstractEventLoop):
    """创建一个正确停止系统的信号处理器。"""
    def signal_handler(signum, frame):
        """处理关闭信号。"""
        logger.info(f"收到信号 {signum}")
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
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
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
    parser.add_argument("--env-dev", action="store_true", help="Use development environment (localhost)")
    
    # 路径
    parser.add_argument("--urdf", default="URDF/SO100/so100.urdf", help="Path to robot URDF file")
    
    # 机器人设置
    parser.add_argument("--config", default="config.yaml", help="Path to config file")
    parser.add_argument("--left-port", help="Left arm serial port (overrides config file)")
    parser.add_argument("--right-port", help="Right arm serial port (overrides config file)")
    
    return parser.parse_args()


def create_config_from_args(args) -> TelegripConfig:
    """从命令行参数创建配置对象。"""
    # 首先加载配置文件
    config_data = get_config_data()
    config = TelegripConfig()
    
    # 应用命令行覆盖
    config.enable_robot = not args.no_robot
    config.enable_pybullet = not args.no_sim
    config.enable_pybullet_gui = config.enable_pybullet and not args.no_viz
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    config.autoconnect = args.autoconnect
    config.log_level = args.log_level
    
    config.websocket_port = args.ws_port
    config.host_ip = args.host
    
    # 处理环境配置
    if args.env_dev:
        # 开发环境：使用 localhost（除非命令行明确指定了其他地址）
        config.server_host = args.server_host if args.server_host else 'localhost'
        config.api_host = args.api_host if args.api_host else 'localhost'
    else:
        # 生产环境：使用命令行参数或保持默认值
        if args.server_host:
            config.server_host = args.server_host
        if args.api_host:
            config.api_host = args.api_host
    
    config.urdf_path = args.urdf
    
    # 处理端口配置 - 如果提供则使用命令行参数，否则使用配置文件值
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
    
    # 在非详细模式下抑制 PyBullet 的原生输出
    if log_level > logging.INFO:
        os.environ['PYBULLET_SUPPRESS_CONSOLE_OUTPUT'] = '1'
        os.environ['PYBULLET_SUPPRESS_WARNINGS'] = '1'
    
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
        logger.info("使用以下配置启动:")
        logger.info(f"  机器人: {'启用' if config.enable_robot else '禁用'}")
        logger.info(f"  PyBullet: {'启用' if config.enable_pybullet else '禁用'}")
        logger.info(f"  无头模式: {'启用' if not config.enable_pybullet_gui and config.enable_pybullet else '禁用'}")
        logger.info(f"  VR: {'启用' if config.enable_vr else '禁用'}")
        logger.info(f"  键盘: {'启用' if config.enable_keyboard else '禁用'}")
        logger.info(f"  自动连接: {'启用' if config.autoconnect else '禁用'}")
        logger.info(f"  WebSocket 端口: {config.websocket_port}")
        logger.info(f"  机器人端口: {config.follower_ports}")
    else:
        # 显示干净的启动消息
        print(f"🤖 telegrip 启动中...")
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
            logger.info("收到中断信号")
        else:
            print("\n🛑 正在关闭...")
    except asyncio.CancelledError:
        # 处理取消错误（通常来自重启场景）
        if log_level <= logging.INFO:
            logger.info("系统任务已取消")
    except Exception as e:
        if log_level <= logging.INFO:
            logger.error(f"系统错误: {e}")
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
            # 在关闭期间忽略“错误的文件描述符”和“事件循环已关闭”错误
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
        logger.error(f"致命错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli() 
