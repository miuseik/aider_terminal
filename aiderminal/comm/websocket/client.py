"""
WebSocket 客户端 - 连接到 Aider Server。
处理 VR 客户端和服务器之间的双向消息转发。
"""

import asyncio
import json
import logging
import numpy as np
from typing import Optional

from .transport import WSTransport
from .protocol import encode_message, decode_message

logger = logging.getLogger(__name__)


class VRWebSocketClient:
    """WebSocket 业务层 - 处理 VR 数据和 API 命令"""
    
    def __init__(self, config, vr_handler, actuator_router=None, control_loop=None):
        self.config = config
        self.vr_handler = vr_handler
        self.actuator_router = actuator_router
        self.control_loop = control_loop
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

    async def run_status_pusher(self, interval: float = 1.0):
        """周期推送硬件状态到 Server（供 /api/status 使用）。业务归属：WebSocket 客户端。

        作为独立后台任务运行（由主入口 create_task 启动），不侵入控制循环。
        """
        while True:
            await asyncio.sleep(interval)
            if not self.transport.is_connected or not self.control_loop:
                continue
            try:
                status = self.control_loop.status
                status["type"] = "hardware_status"
                await self.transport.send_raw(encode_message(status))
            except Exception:
                pass
    
    async def handle_api_command(self, data: dict):
        """处理来自服务器的 API 命令。"""
        category = data.get('category')
        action = data.get('action')

        if category == 'motor':
            # 舵机管理命令 → ActuatorRouter
            await self.actuator_router.route_and_respond(data, self.transport)
        elif action and self.control_loop:
            # 机器人/系统命令 → 在本类中处理
            await self._handle_command(data)
        elif hasattr(self.vr_handler, 'process_message'):
            # VR 控制器数据
            await self.vr_handler.process_message(json.dumps(data))
        else:
            print(f"❌ 无法路由的消息: {data}")

    async def _handle_command(self, command):
        """处理单个命令。"""
        cl = self.control_loop
        action = command.get('action', '')

        if action == 'enable_keyboard':
            if cl and cl.web_keyboard_handler:
                # 重连后 is_engaged 可能为 False，自动使能
                if cl.robot_interface and cl.robot_interface.is_connected and not cl.robot_interface.is_engaged:
                    cl.robot_interface.engage()
                await cl.web_keyboard_handler.start()
                print("🎮 键盘控制已启用")
        elif action == 'disable_keyboard':
            if cl and cl.web_keyboard_handler:
                await cl.web_keyboard_handler.stop()
                print("🎮 键盘控制已禁用")
        elif action in ('web_keypress', 'keypress'):
            key = command.get('key')
            event = command.get('event')  # 'press' or 'release'
            if cl and cl.web_keyboard_handler and cl.web_keyboard_handler.is_enabled:
                if event == 'press':
                    cl.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    cl.web_keyboard_handler.on_key_release(key)
        elif action == 'robot_connect':
            if cl and cl.robot_interface:
                ri = cl.robot_interface
                # 用户点"连接"时总是重新扫描硬件，确保新插上的舵机也能被发现
                print(f"🔍 用户触发连接，重新扫描硬件 "
                      f"(is_connected={ri.is_connected}, online_servos现有={len(ri.online_servos)}个: {sorted(ri.online_servos.keys())})")
                ri.is_connected = False  # 重置以允许 connect() 重新执行扫描
                # connect() 包含 HTTP 请求 + 串口扫描，在独立线程中执行，不阻塞事件循环
                loop = asyncio.get_event_loop()
                executor = cl.motor_controller._executor if cl.motor_controller else None
                if executor:
                    success = await loop.run_in_executor(executor, ri.connect, True)
                else:
                    success = ri.connect(force_scan=True)
                # 不论成功与否都推送硬件信息，让前端拿到明确的连接状态（避免静默卡死）
                from aiderminal.utils.hardware_info import push_robot_hardware_info
                asyncio.create_task(push_robot_hardware_info(self.transport, ri))
                if not success:
                    print("❌ 机器人连接失败")
                    await self.transport.send_raw(encode_message({
                        "type": "robot_connect_response",
                        "success": False,
                        "message": "硬件扫描未找到足够在线舵机，连接失败",
                    }))
                    return
                print(f"🟢 扫描完成，在线舵机: {sorted(ri.online_servos.keys())}")
                # 连接成功后绑定 ServoConfigManager
                if self.actuator_router and hasattr(ri, 'servo_config_manager'):
                    self.actuator_router.bind_servo_config(
                        ri.servo_config_manager
                    )
                # 不自动使能——用户从前端动作列表选择姿态后才使能
                print("🔌 机器人已连接，等待选择姿态…")
                # 明确告诉前端连接成功（前端用于提示 + 驱动按钮状态）
                await self.transport.send_raw(encode_message({
                    "type": "robot_connect_response",
                    "success": True,
                    "message": "机器人连接成功",
                }))
            else:
                print("❌ 无机器人接口")
                await self.transport.send_raw(encode_message({
                    "type": "robot_connect_response",
                    "success": False,
                    "message": "无机器人接口",
                }))
        elif action == 'goto_pose':
            if cl and cl.robot_interface:
                arm = command.get('arm', 'both')
                pose_name = command.get('pose_name', 'safe')

                # 防止控制循环的 IK 覆盖姿态角度：
                # 1. 重置 ArmState → IDLE，让 _update_robot 跳过 IK
                # 2. 排空命令队列中目标手臂的 POSITION_CONTROL 指令，防止重新激活
                arms = ["left", "right"] if arm == "both" else [arm]
                for a in arms:
                    (cl.left_arm if a == "left" else cl.right_arm).reset()
                kept = []
                while not cl.command_queue.empty():
                    g = cl.command_queue.get_nowait()
                    if not (g.arm in arms and g.mode and g.mode.name == "POSITION_CONTROL"):
                        kept.append(g)
                for g in kept:
                    cl.command_queue.put_nowait(g)

                result = await cl.robot_interface.goto_pose(arm, pose_name)
                print(f"goto_pose result: {result}")
                # 推送结果给前端
                await self.transport.send_raw(encode_message({
                    "type": "goto_pose_response",
                    "success": result.get("success", False),
                    "message": result.get("message", ""),
                }))
            else:
                print("❌ goto_pose: 无机器人接口")
        elif action == 'list_poses':
            if cl and cl.robot_interface:
                poses = cl.robot_interface.list_poses()
                print(f"📋 可用姿态: {list(poses.keys())}")
                await self.transport.send_raw(encode_message({
                    "type": "list_poses_response",
                    "poses": poses,
                }))
            else:
                print("❌ list_poses: 无机器人接口")
        elif action == 'robot_disconnect':
            if cl and cl.robot_interface:
                ri = cl.robot_interface

                # 防止控制循环的 IK 覆盖姿态角度：
                # 在 disengage() 前重置 ArmState → IDLE，排空命令队列
                cl.left_arm.reset()
                cl.right_arm.reset()
                kept = []
                while not cl.command_queue.empty():
                    g = cl.command_queue.get_nowait()
                    if not (g.mode and g.mode.name == "POSITION_CONTROL"):
                        kept.append(g)
                for g in kept:
                    cl.command_queue.put_nowait(g)

                success = await ri.disengage()
                if success:
                    ri.is_connected = False  # 标记已断开，推送的 hardware_info 会反映此状态
                    print("🔌 机器人已断开")
                    if cl.visualizer:
                        for arm in ["left", "right"]:
                            cl.visualizer.hide_marker(f"{arm}_goal")
                            cl.visualizer.hide_frame(f"{arm}_goal_frame")
                            cl.visualizer.hide_marker(f"{arm}_target")
                            cl.visualizer.hide_frame(f"{arm}_target_frame")
                    # 通知前端机器人已断开连接（后台执行，不阻塞）
                    from aiderminal.utils.hardware_info import push_robot_hardware_info
                    asyncio.create_task(push_robot_hardware_info(self.transport, ri))
                else:
                    print("❌ 禁能失败")
            else:
                print("❌ 无机器人接口")
        elif action == 'restart':
            print("🔄 收到重启命令")
            if cl and hasattr(cl, 'main_app'):
                cl.main_app.restart()
                # 超时保护：10秒后强制硬重启
                import threading, os, sys
                def force_restart():
                    import time
                    time.sleep(10)
                    print("⚠️ 软重启超时，执行强制硬重启")
                    os.execv(sys.executable, [sys.executable] + sys.argv)
                threading.Thread(target=force_restart, daemon=True).start()
        elif action.startswith('control_') or action == 'calibrate_motor':
            if not self.actuator_router:
                print("⚠️ API命令路由器未初始化")
                return
            self.actuator_router.route(command)
        # 其余命令静默处理（web_keypress 等高频命令不打印）
