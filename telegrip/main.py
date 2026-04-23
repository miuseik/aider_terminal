"""
统一遥操作系统的主入口。
协调 HTTPS 服务器、WebSocket 服务器、机器人接口和输入提供者。
"""

import asyncio
import argparse
import logging
import signal
import sys
import os
import http.server
import ssl
import socket
import json
import urllib.parse
import time
import contextlib
from typing import Optional, Dict, Any
import queue  # Add regular queue for thread-safe communication
import threading
from pathlib import Path
import weakref


def get_local_ip():
    """获取本机的本地 IP 地址。"""
    try:
        # Connect to a remote address to determine the local IP
        # This doesn't actually send any data
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback: get hostname IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # Final fallback
            return "localhost"


@contextlib.contextmanager
def suppress_stdout_stderr():
    """上下文管理器，在文件描述符级别抑制 stdout 和 stderr 输出。"""
    # 保存原始文件描述符
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # 保存原始文件描述符
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # 打开 /dev/null
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # 将 stdout 和 stderr 重定向到 /dev/null
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # 恢复原始文件描述符
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # 关闭已保存的文件描述符
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


# 在函数定义之后导入 telegrip 模块
from .config import TelegripConfig, get_config_data, update_config_data
from .control_loop import ControlLoop
from .inputs.vr_ws_server import VRWebSocketServer
from .inputs.vr_ws_client import VRWebSocketClient
from .inputs.web_keyboard import WebKeyboardHandler
from .inputs.base import ControlGoal
from .api_routes import APIRoutes
from .static_files import StaticFileServer

# 日志记录器将在 main() 中根据命令行参数进行配置
logger = logging.getLogger(__name__)


class APIHandler(http.server.BaseHTTPRequestHandler):
    """遥操作 API 的 HTTP 请求处理器。"""
    
    def __init__(self, *args, **kwargs):
        # 为所有请求设置 CORS 头
        super().__init__(*args, **kwargs)
    
    def end_headers(self):
        """为所有响应添加 CORS 头。"""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            # 客户端断开连接或 SSL 错误 - 静默忽略
            pass
    
    def do_OPTIONS(self):
        """处理预检 CORS 请求。"""
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        """重写以减少 HTTP 请求日志噪音。"""
        pass  # 禁用默认 HTTP 日志
    
    def do_GET(self):
        """处理 GET 请求 - 路由到对应的处理方法。"""
        # 获取 API 路由处理器
        routes = self._get_api_routes()
        if not routes:
            self.send_error(500, "System not available")
            return
        
        # 静态文件服务器
        static_server = self._get_static_server()
        
        # 路由分发
        if self.path == '/api/status':
            self._send_json_response(routes.get_status())
        elif self.path == '/api/config':
            self._send_json_response(routes.get_config())
        elif static_server and static_server.route_static_file(self, self.path):
            # 静态文件已处理
            pass
        else:
            self.send_error(404, "Not found")
    
    def do_POST(self):
        """处理 POST 请求 - 路由到对应的处理方法。"""
        # 获取 API 路由处理器
        routes = self._get_api_routes()
        if not routes:
            self.send_error(500, "System not available")
            return
        
        # 读取请求体
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length == 0:
            self.send_error(400, "No request body")
            return
        
        try:
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
            return
        
        # 路由分发
        if self.path == '/api/keyboard':
            action = data.get('action')
            result = routes.handle_keyboard_action(action)
            self._send_json_response(result)
        elif self.path == '/api/robot':
            action = data.get('action')
            result = routes.handle_robot_action(action)
            self._send_json_response(result)
        elif self.path == '/api/keypress':
            key = data.get('key')
            action = data.get('action')
            result = routes.handle_keypress(key, action)
            self._send_json_response(result)
        elif self.path == '/api/config':
            result = routes.update_config(data)
            self._send_json_response(result)
        elif self.path == '/api/restart':
            result = routes.restart_system()
            self._send_json_response(result)
        else:
            self.send_error(404, "Not found")
    
    def _get_static_server(self) -> Optional['StaticFileServer']:
        """获取静态文件服务器实例（懒加载）"""
        if hasattr(self.server, 'api_handler') and self.server.api_handler:
            system = self.server.api_handler
            if not hasattr(system, '_static_server'):
                system._static_server = StaticFileServer()
            return system._static_server
        return None
    
    def _get_api_routes(self) -> Optional['APIRoutes']:
        """获取 API 路由处理器实例（懒加载）"""
        if hasattr(self.server, 'api_handler') and self.server.api_handler:
            system = self.server.api_handler
            if not hasattr(system, '_api_routes'):
                system._api_routes = APIRoutes(system)
            return system._api_routes
        return None
    
    def _send_json_response(self, data: Dict[str, Any]):
        """发送 JSON 响应"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = json.dumps(data)
        self.wfile.write(response.encode('utf-8'))


class HTTPSServer:
    """遥操作 API 的 HTTPS 服务器。"""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None  # 主系统的直接引用
    
    def set_system_ref(self, system_ref):
        """设置主遥操作系统的引用。"""
        self.system_ref = system_ref
    
    async def start(self):
        """启动 HTTPS 服务器。"""
        try:
            # 创建服务器 - 直接使用 APIHandler 类
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), APIHandler)
            
            # 设置 API 处理器引用以进行命令排队
            self.httpd.api_handler = self.system_ref
            
            # 设置 SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            # 获取 SSL 证书的绝对路径
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            context.load_cert_chain(cert_path, key_path)
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
            
            # 在单独的线程中启动服务器
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            # 仅在 INFO 级别或更详细时记录日志
            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                host_display = get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
                logger.info(f"HTTPS server started on https://{host_display}:{self.config.https_port}")
            
        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            raise
    
    async def stop(self):
        """停止 HTTPS 服务器。"""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")


class TelegripSystem:
    """主遥操作系统，协调所有组件。"""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        
        # 任务 - 必须最先初始化
        self.tasks = []
        self.is_running = False
        self.main_loop = None  # 将在系统启动时设置
        
        # 命令队列
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)  # 线程安全队列
        
        # 组件
        self.https_server = HTTPSServer(config)
        
        # VR 组件 - 支持服务端和客户端模式
        self.vr_server = None
        self.vr_client = None
        self.webrtc_streamer = None  # WebRTC 视频推流器（ECS 模式）
        
        if not config.ecs_enabled:
            # 仅本地服务端模式
            logger.info("🏠 Local server mode enabled")
            self.vr_server = VRWebSocketServer(self.command_queue, config)
        else:
            # ECS 客户端模式
            logger.info("🌐 ECS client mode enabled - connecting to remote server")
            
            # 创建共享的 WebSocket 客户端
            from .inputs.websocket_client import WebSocketClient
            shared_ws_client = WebSocketClient(config)
            
            # 启动共享的 WebSocket 客户端（在后台任务中）
            ws_client_task = asyncio.create_task(shared_ws_client.start())
            self.tasks.append(ws_client_task)
            
            # 创建 VR 客户端（注入共享的 ws_client）
            self.vr_client = VRWebSocketClient(self.command_queue, config, shared_ws_client)
            # 设置 system 引用,用于获取状态
            self.vr_client.set_system_ref(lambda: self)
            
            # 根据配置决定是否启动本地服务端
            if config.local_ws_enabled:
                logger.info("🏠 Also starting local WebSocket server for LAN access")
                self.vr_server = VRWebSocketServer(self.command_queue, config)
            else:
                logger.info("⚠️ Local WebSocket server disabled")
        
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)

        # 为 API 调用设置系统引用
        self.https_server.set_system_ref(self)

        # Set up cross-references
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler

        # Set up disconnect callback for ESC key
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")
    
    def add_control_command(self, action: str):
        """向队列添加控制命令以供处理。"""
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
        """向队列添加按键命令以供处理。"""
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
            logger.error(f"Error processing control commands: {e}")
    
    def restart(self):
        """重启遥操作系统。"""
        def do_restart():
            try:
                logger.info("Initiating system restart...")
                # 使用存储的主事件循环引用来调度软重启
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    # 等待重启完成
                    future.result(timeout=30.0)
                else:
                    logger.error("Main event loop not available for restart")
            except Exception as e:
                logger.error(f"Error during restart: {e}")
        
        # 在单独的线程中运行重启以避免阻塞 HTTP 响应
        restart_thread = threading.Thread(target=do_restart, daemon=True)
        restart_thread.start()
    
    async def _soft_restart_sequence(self):
        """执行软重启，通过重新初始化组件而不退出进程。"""
        try:
            logger.info("Starting soft restart sequence...")
            
            # 稍等片刻让 HTTP 响应被发送
            await asyncio.sleep(1)
            
            # 取消所有任务
            for task in self.tasks:
                task.cancel()
            
            # 等待任务完成（带超时）
            if self.tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*self.tasks, return_exceptions=True), 
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    logger.warning("Some tasks did not complete within timeout")
            
            # 按相反顺序停止组件
            await self.control_loop.stop()
            await self.web_keyboard_handler.stop()
            if self.vr_client:
                await self.vr_client.stop()
            if self.vr_server:
                await self.vr_server.stop()
            # 不要停止 HTTPS 服务器 - 保持其运行以供 UI 使用

            # 稍等片刻进行清理
            await asyncio.sleep(1)

            # 从文件重新加载配置但保留命令行覆盖
            from .config import get_config_data
            file_config = get_config_data()
            logger.info("Configuration reloaded from file")

            # 保留现有配置对象以保持命令行参数
            # 只更新配置文件中可能已更改的特定值

            # 使用现有配置重新创建组件
            self.command_queue = asyncio.Queue()
            self.control_commands_queue = queue.Queue(maxsize=10)

            # 创建新组件（保留 ECS 模式）
            if self.config.ecs_enabled:
                from .inputs.websocket_client import WebSocketClient
                from .inputs.webrtc_streamer import WebRTCVideoStreamer
                
                shared_ws_client = WebSocketClient(self.config)
                # 启动共享的 WebSocket 客户端（在后台任务中）
                ws_client_task = asyncio.create_task(shared_ws_client.start())
                self.tasks.append(ws_client_task)
                
                self.vr_client = VRWebSocketClient(self.command_queue, self.config, shared_ws_client)
                # 设置 system 引用,用于获取状态
                self.vr_client.set_system_ref(lambda: self)
                
                # 创建独立的 WebRTC 视频推流器
                self.webrtc_streamer = WebRTCVideoStreamer(self.config, shared_ws_client)
            else:
                self.vr_client = None
                self.webrtc_streamer = None
            
            self.vr_server = VRWebSocketServer(self.command_queue, self.config)
            self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, self.config)
            self.control_loop = ControlLoop(self.command_queue, self.config, self.control_commands_queue)

            # 设置交叉引用
            self.control_loop.web_keyboard_handler = self.web_keyboard_handler

            # 为 ESC 键设置断开回调
            self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

            # 清除旧任务
            self.tasks = []

            # Start VR WebSocket server and/or client
            if self.vr_server:
                await self.vr_server.start()
            if self.vr_client:
                # Start client in background task
                client_task = asyncio.create_task(self.vr_client.start())
                self.tasks.append(client_task)
            
            # Start WebRTC video streamer (ECS mode only)
            if self.webrtc_streamer:
                webrtc_task = asyncio.create_task(self.webrtc_streamer.start())
                self.tasks.append(webrtc_task)

            # Start web keyboard handler
            await self.web_keyboard_handler.start()

            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("System restart completed successfully")
            
            # 如果请求则自动连接到机器人（重启后保留自动连接行为）
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors after restart...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
        except Exception as e:
            logger.error(f"Error during soft restart sequence: {e}")
            raise
    
    async def start(self):
        """启动所有系统组件。"""
        try:
            self.is_running = True
            
            # 存储主事件循环的引用以供重启功能使用
            self.main_loop = asyncio.get_event_loop()
            
            # 启动 HTTPS 服务器
            await self.https_server.start()
            
            # Start VR WebSocket server and/or client
            if self.vr_server:
                await self.vr_server.start()
            if self.vr_client:
                # Start client in background task
                client_task = asyncio.create_task(self.vr_client.start())
                self.tasks.append(client_task)
            
            # Start WebRTC video streamer (ECS mode only)
            if self.webrtc_streamer:
                webrtc_task = asyncio.create_task(self.webrtc_streamer.start())
                self.tasks.append(webrtc_task)

            # Start web keyboard handler
            await self.web_keyboard_handler.start()

            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("All system components started successfully")
            
            # 如果请求则自动连接到机器人
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
            # 处理重启的主循环
            while self.is_running:
                try:
                    # 等待任务完成
                    await asyncio.gather(*self.tasks)
                    # 如果到这里，所有任务正常完成（正常操作中不应该发生）
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
                    logger.error(f"Error in main task loop: {e}")
                    break
            
        except OSError as e:
            if e.errno == 98:  # Address already in use
                logger.error(f"Error starting teleoperation system: {e}")
                logger.error(f"To find and kill the process using these ports, run:")
                logger.error(f"  kill -9 $(lsof -t -i:{self.config.https_port} -i:{self.config.websocket_port})")
            else:
                logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
        except Exception as e:
            logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
    
    async def _run_command_processor(self):
        """运行控制命令处理器循环。"""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)  # 每 50ms 检查一次命令
    
    async def stop(self):
        """停止所有系统组件。"""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False

        # 首先停止 VR 服务器以关闭 WebSocket 连接（解除任何等待的处理程序阻塞）
        try:
            if self.vr_client:
                await asyncio.wait_for(self.vr_client.stop(), timeout=2.0)
            if self.vr_server:
                await asyncio.wait_for(self.vr_server.stop(), timeout=2.0)
            if self.webrtc_streamer:
                await asyncio.wait_for(self.webrtc_streamer.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("VR/WebRTC stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping VR/WebRTC: {e}")

        # 取消所有任务
        for task in self.tasks:
            task.cancel()

        # 等待任务完成（带超时）
        if self.tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*self.tasks, return_exceptions=True),
                    timeout=2.0
                )
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")

        # 停止剩余组件
        try:
            await asyncio.wait_for(self.control_loop.stop(), timeout=3.0)
        except asyncio.TimeoutError:
            logger.warning("Control loop stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping control loop: {e}")

        try:
            await asyncio.wait_for(self.web_keyboard_handler.stop(), timeout=1.0)
        except asyncio.TimeoutError:
            logger.warning("Web keyboard handler stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping web keyboard handler: {e}")

        try:
            await asyncio.wait_for(self.https_server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("HTTPS server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping HTTPS server: {e}")

        logger.info("Teleoperation system shutdown complete")


def create_signal_handler(system: 'TelegripSystem', loop: asyncio.AbstractEventLoop):
    """创建正确停止系统的信号处理器。"""
    def signal_handler(signum, frame):
        """处理关闭信号。"""
        logger.info(f"Received signal {signum}")
        system.is_running = False
        # 从事件循环中取消所有任务
        for task in system.tasks:
            loop.call_soon_threadsafe(task.cancel)
        # 引发 SystemExit 以跳出阻塞操作
        raise SystemExit(0)
    return signal_handler


def parse_arguments():
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")
    
    # 控制标志
    parser.add_argument("--no-robot", action="store_true", help="禁用机器人连接（仅可视化）")
    parser.add_argument("--no-sim", action="store_true", help="禁用 PyBullet 仿真和逆运动学")
    parser.add_argument("--no-viz", action="store_true", help="禁用 PyBullet 可视化（无头模式）")
    parser.add_argument("--no-vr", action="store_true", help="禁用 VR WebSocket 服务器")
    parser.add_argument("--no-keyboard", action="store_true", help="禁用键盘输入")
    parser.add_argument("--no-https", action="store_true", help="禁用 HTTPS 服务器")
    parser.add_argument("--autoconnect", action="store_true", help="启动时自动连接到机器人电机")
    parser.add_argument("--log-level", default="warning", 
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="设置日志级别（默认：warning）")
    
    # 网络设置
    parser.add_argument("--https-port", type=int, default=8443, help="HTTPS 服务器端口")
    parser.add_argument("--ws-port", type=int, default=8442, help="WebSocket 服务器端口")
    parser.add_argument("--host", default="0.0.0.0", help="主机 IP 地址")
    
    # 路径
    parser.add_argument("--urdf", default="URDF/SO100/so100.urdf", help="机器人 URDF 文件路径")
    parser.add_argument("--webapp", default="webapp", help="webapp 目录路径")
    parser.add_argument("--cert", default="cert.pem", help="SSL 证书路径")
    parser.add_argument("--key", default="key.pem", help="SSL 私钥路径")
    
    # 机器人设置
    parser.add_argument("--config", default="config.yaml", help="配置文件路径")
    parser.add_argument("--left-port", help="左臂串口（覆盖配置文件）")
    parser.add_argument("--right-port", help="右臂串口（覆盖配置文件）")
    
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
    
    config.https_port = args.https_port
    config.websocket_port = args.ws_port
    config.host_ip = args.host
    
    config.urdf_path = args.urdf
    config.webapp_dir = args.webapp
    config.certfile = args.cert
    config.keyfile = args.key
    
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
    
    if log_level <= logging.INFO:
        # 详细模式 - 显示带时间戳的详细日志
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        # 安静模式 - 仅显示警告和错误，使用简单格式
        logging.basicConfig(
            level=log_level,
            format='%(message)s'
        )

    # 抑制嘈杂的 websockets 库日志记录（对 WS 端口的无效 HTTP 请求）
    logging.getLogger('websockets').setLevel(logging.WARNING)

    config = create_config_from_args(args)

    # 确保 SSL 证书存在（首次启动时如需则生成）
    if not config.ensure_ssl_certificates():
        logger.error("Failed to ensure SSL certificates are available")
        sys.exit(1)

    # 记录配置（仅在 INFO 级别或更详细时）
    if log_level <= logging.INFO:
        logger.info("Starting with configuration:")
        logger.info(f"  Robot: {'enabled' if config.enable_robot else 'disabled'}")
        logger.info(f"  PyBullet: {'enabled' if config.enable_pybullet else 'disabled'}")
        logger.info(f"  Headless mode: {'enabled' if not config.enable_pybullet_gui and config.enable_pybullet else 'disabled'}")
        logger.info(f"  VR: {'enabled' if config.enable_vr else 'disabled'}")
        logger.info(f"  Keyboard: {'enabled' if config.enable_keyboard else 'disabled'}")
        logger.info(f"  Auto-connect: {'enabled' if config.autoconnect else 'disabled'}")
        logger.info(f"  HTTPS Port: {config.https_port}")
        logger.info(f"  WebSocket Port: {config.websocket_port}")
        logger.info(f"  Robot Ports: {config.follower_ports}")
    else:
        # 显示带有 HTTPS URL 的干净启动消息
        host_display = get_local_ip() if config.host_ip == "0.0.0.0" else config.host_ip
        print(f"🤖 telegrip starting...")
        print(f"📱 Open the UI in your browser on:")
        print(f"   https://{host_display}:{config.https_port}")
        print(f"📱 Then go to the same address on your VR headset browser")
        print(f"💡 Use --log-level info to see detailed output")
        print()
    
    # 创建并启动遥操作系统
    system = TelegripSystem(config)

    # 使用系统和事件循环的引用设置信号处理器
    loop = asyncio.get_event_loop()
    signal_handler = create_signal_handler(system, loop)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        await system.start()
    except (KeyboardInterrupt, SystemExit):
        if log_level <= logging.INFO:
            logger.info("Received interrupt signal")
        else:
            print("\n🛑 Shutting down...")
    except asyncio.CancelledError:
        # 处理取消的错误（通常来自重启场景）
        if log_level <= logging.INFO:
            logger.info("System tasks cancelled")
    except Exception as e:
        if log_level <= logging.INFO:
            logger.error(f"System error: {e}")
        else:
            print(f"❌ Error: {e}")
    finally:
        try:
            await system.stop()
        except (asyncio.CancelledError, SystemExit):
            # Ignore cancelled/exit errors during shutdown
            pass

        # 在事件循环清理期间抑制 SSL 传输错误
        def ignore_ssl_errors(loop, context):
            # 忽略关闭期间的“Bad file descriptor”和“Event loop is closed”错误
            if 'exception' in context:
                exc = context['exception']
                if isinstance(exc, (OSError, RuntimeError)):
                    return
            # 正常记录其他错误
            loop.default_exception_handler(context)

        loop.set_exception_handler(ignore_ssl_errors)

        if log_level > logging.INFO:
            print("✅ Shutdown complete.")


def main_cli():
    """pip 安装包的控制台脚本入口点。"""
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, SystemExit):
        print("\nShutdown complete.")
    except asyncio.CancelledError:
        # 处理来自重启场景的取消错误
        pass
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli() 
