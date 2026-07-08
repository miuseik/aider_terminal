"""WebSocket 传输层 - 负责连接管理、重连、消息收发"""
import asyncio
import ssl
import websockets
import logging
from typing import Optional, Callable
from pathlib import Path

logger = logging.getLogger(__name__)


class WSTransport:
    """WebSocket 传输层 - 纯连接管理,不关心业务逻辑"""
    
    def __init__(self, config):
        self.config = config
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.is_connected = False
        self.on_message_callback: Optional[Callable] = None
        self._extra_handlers: list = []  # 额外的消息处理器
        
        # SSL 上下文
        self.ssl_context = None
    
    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """设置 WebSocket 客户端连接的 SSL 上下文。"""
        ssl_context = ssl.create_default_context()
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE
        return ssl_context
    
    async def connect(self):
        """连接到服务器，支持自动重连。"""
        self.ssl_context = self.setup_ssl()
        if self.ssl_context is None:
            print("SSL 设置失败")
            return False
        
        host = self.config.server_host
        port = self.config.websocket_port
        ws_url = f"wss://{host}:{port}/ws/terminal"
        
        try:
            print(f"🔌 正在连接服务器: {ws_url}")
            self.websocket = await websockets.connect(
                ws_url,
                ssl=self.ssl_context,
                ping_interval=20,       # 每20秒发一次 ping
                ping_timeout=90,        # 等90秒超时（容忍高负载/慢网络）
                close_timeout=10,       # 关闭握手超时
                max_size=10 * 1024 * 1024,  # 最大消息10MB
            )
            self.is_connected = True
            print(f"✅ WebSocket 已连接: {ws_url}")
            
            # 启动消息接收任务
            asyncio.create_task(self._receive_loop())
            return True
            
        except Exception as e:
            print(f"❌ ws 连接失败: {e}")
            # 3秒后自动重连
            print(f"🔄 3秒后重连...")
            await asyncio.sleep(3)
            return await self.connect()
    
    async def disconnect(self):
        """断开与服务器的连接。"""
        self.is_connected = False
        if self.websocket:
            try:
                await self.websocket.close()
            except Exception:
                pass
            print("🔴 已断开与服务器的连接")
    
    async def _receive_loop(self):
        """消息接收循环。"""
        try:
            async for message in self.websocket:
                if not self.is_connected:
                    break
                
                # 主回调
                if self.on_message_callback:
                    await self.on_message_callback(message)
                
                # 额外处理器（如 WebRTC 信令）
                for handler in self._extra_handlers:
                    try:
                        await handler(message)
                    except Exception:
                        pass
        
        except websockets.exceptions.ConnectionClosedOK:
            print("❌ 连接已关闭（正常）")
            self.is_connected = False
            await self._reconnect()
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"❌ 连接已关闭（错误）: {e}")
            self.is_connected = False
            await self._reconnect()
        except Exception as e:
            print(f"❌ 接收错误: {e}")
            self.is_connected = False
            await self._reconnect()
    
    async def _reconnect(self):
        """自动重连。"""
        print(f"🔄 3秒后重连...")
        await asyncio.sleep(3)
        await self.connect()
    
    async def send_raw(self, data: str):
        """发送原始字符串消息。"""
        if not self.is_connected or not self.websocket:
            print("⚠️ 未连接，无法发送消息")
            return False
        
        try:
            await self.websocket.send(data)
            return True
        except Exception as e:
            print(f"❌ 发送失败: {e}")
            return False
    
    def on_message(self, callback: Callable):
        """注册消息回调。"""
        self.on_message_callback = callback
    
    def add_handler(self, callback: Callable):
        """注册额外的消息处理器（如 WebRTC 信令）。"""
        self._extra_handlers.append(callback)
