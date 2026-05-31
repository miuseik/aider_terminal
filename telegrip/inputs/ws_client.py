"""
WebSocket 客户端 - 连接到 Aider Server。
处理 VR 客户端和服务器之间的双向消息转发。
"""

import asyncio
import json
import logging
from typing import Optional

from .socket.ws_transport import WSTransport
from .socket.ws_protocol import encode_message, decode_message

logger = logging.getLogger(__name__)


class VRWebSocketClient:
    """WebSocket 业务层 - 处理 VR 数据和 API 命令"""
    
    def __init__(self, config, vr_handler, motor_router=None):
        self.config = config
        self.vr_handler = vr_handler
        self.motor_router = motor_router
        self.transport = WSTransport(config)
        self.client_id = "terminal"  # Terminal 始终使用此 ID
        
        # 注册消息回调
        self.transport.on_message(self._handle_message)
    
    async def connect(self):
        """连接到 Aider Server。"""
        return await self.transport.connect()
    
    async def disconnect(self):
        """断开与 Aider Server 的连接。"""
        await self.transport.disconnect()
    
    async def _handle_message(self, raw_message: str):
        """处理来自传输层的传入消息。"""
        try:
            data = decode_message(raw_message)
            
            # 检查是否为 API 命令
            if data.get('type') == 'api_command':
                await self.handle_api_command(data)
            else:
                # 转发到 VR 处理器进行处理
                await self.vr_handler.process_message(raw_message)
            
        except json.JSONDecodeError:
            print(f"⚠️ 收到非 JSON 消息")
        except Exception as e:
            print(f"❌ 处理消息错误: {e}")
    
    async def send_vr_data(self, vr_data: dict):
        """发送 VR 控制器数据到服务器。"""
        return await self.transport.send_raw(encode_message(vr_data))
    
    async def send_message(self, data: dict):
        """发送消息到服务器(用于 WebRTC 信令)。"""
        return await self.transport.send_raw(encode_message(data))
    
    async def send_command(self, action: str, **kwargs):
        """发送命令到服务器。"""
        command = {"action": action, **kwargs}
        return await self.transport.send_raw(encode_message(command))
    
    async def handle_api_command(self, data: dict):
        """处理来自服务器的 API 命令。"""
        category = data.get('category')
        print(f"😃来活了",data)
        
        if category == 'motor':
            print("处理 motor 数据", data)
            self.motor_router.route(data)
        elif hasattr(self.vr_handler, 'process_message'):
            print("处理 键盘 数据", data)
            await self.vr_handler.process_message(json.dumps(data))
        else:
            print(f"❌ 都 没有 这个 方法")
