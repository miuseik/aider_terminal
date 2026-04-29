"""
VR 处理器，用于处理来自 WebSocket 客户端的控制器数据。
处理 VR 控制器状态跟踪和控制目标生成。
"""

import asyncio
import json
import numpy as np
import math
import logging
from typing import Dict, Optional
from scipy.spatial.transform import Rotation as R

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig
from ..core.kinematics import compute_relative_position

logger = logging.getLogger(__name__)


class VRControllerState:
    """VR 控制器的状态跟踪。"""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # 相对移动的位置跟踪
        self.origin_position = None
        self.origin_rotation = None
        
        # 基于四元数的旋转跟踪(比欧拉角更稳定)
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None  # 累积旋转(四元数)
        
        # 腕部控制的旋转跟踪
        self.z_axis_rotation = 0.0  # 用于 wrist_roll
        self.x_axis_rotation = 0.0  # 用于 wrist_flex (俯仰)
        
        # 位置跟踪
        self.current_position = None
        
        # 旋转跟踪
        self.origin_wrist_angle = 0.0
    
    def reset_grip(self):
        """重置握把状态但保留扳机状态。"""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0


class VRHandler(BaseInputProvider):
    """VR 控制器数据处理器 - 处理 VR 数据并生成控制目标。"""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        
        # 控制器状态
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # 机器人状态跟踪(用于相对位置计算)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None

    async def start(self):
        """启动 VR 处理器(无需服务器)。"""
        self.is_running = True
        logger.info("✅ VR 处理器已启动")

    async def stop(self):
        """停止 VR 处理器。"""
        self.is_running = False
        logger.info("🛑 VR 处理器已停止")
    
    async def process_message(self, message: str):
        """处理来自 WebSocket 客户端的 VR 控制器数据。"""
        try:
            data = json.loads(message)
            
            # 处理 API 类命令
            if 'action' in data:
                await self.handle_api_command(data)
            else:
                # 处理 VR 控制器数据
                await self.process_controller_data(data)
        except json.JSONDecodeError:
            logger.warning(f"⚠️ 收到非 JSON 消息: {message}")
        except Exception as e:
            logger.error(f"❌ 处理数据错误: {e}")
    
    async def handle_api_command(self, data: Dict):
        """处理 API 类命令。"""
        action = data.get('action')
        print(f"📡 VR API 命令: {action}")
        
        if action == 'get_status':
            # 如果有回调，通过回调返回当前状态
            status = {
                "type": "status_response",
                "robotEngaged": False,
                "keyboardEnabled": self.is_running,
                "vrConnected": True
            }
            if hasattr(self, 'on_status_callback'):
                await self.on_status_callback(status)
        
        elif action == 'enable_keyboard':
            print("🎮 键盘控制已启用")
            await self.control_loop._handle_command({'action': 'enable_keyboard'})
        
        elif action == 'disable_keyboard':
            print("🎮 键盘控制已禁用")
            await self.control_loop._handle_command({'action': 'disable_keyboard'})
        
        elif action == 'robot_connect':
            print("🔌 收到机器人连接命令")
            await self.control_loop._handle_command({'action': 'robot_connect'})
        
        elif action == 'robot_disconnect':
            print("🔌 收到机器人断开命令")
            await self.control_loop._handle_command({'action': 'robot_disconnect'})
        
        elif action == 'restart':
            print("🔄 收到重启命令")
            # 调用主应用的软重启方法
            self.control_loop.main_app.restart()
            
            # 设置超时保护：10秒后强制硬重启
            import threading
            import os
            import sys
            
            def force_restart():
                import time
                time.sleep(10)
                print("⚠️ 软重启超时，执行强制硬重启")
                os.execv(sys.executable, [sys.executable] + sys.argv)
            
            restart_thread = threading.Thread(target=force_restart, daemon=True)
            restart_thread.start()
        
        elif action == 'keypress':
            key = data.get('key')
            event = data.get('event')
            print(f"⌨️ 按键 {event}: {key}")
            if hasattr(self, 'web_keyboard_handler') and self.web_keyboard_handler:
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
        
        else:
            logger.warning(f"⚠️ 未知 VR 命令: {action}")
    
    async def process_controller_data(self, data: Dict):
        """处理传入的 VR 控制器数据。"""
        
        # 处理新的双控制器格式
        if 'leftController' in data and 'rightController' in data:
            left_data = data['leftController']
            right_data = data['rightController']
            # 处理左控制器
            if left_data.get('position') and (left_data.get('gripActive', False) or left_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('left', left_data)
            elif not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            # 处理右控制器
            if right_data.get('position') and (right_data.get('gripActive', False) or right_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('right', right_data)
            elif not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return
        
        # 处理旧版单控制器格式
        hand = data.get('hand')
        
        # 处理显式释放消息
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
            
        # 处理单控制器数据
        if hand and data.get('position') and (data.get('gripActive', False) or data.get('trigger', 0) > 0.5):
            await self.process_single_controller(hand, data)
    
    async def process_single_controller(self, hand: str, data: Dict):
        """处理单个控制器的数据。"""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        joystick = data.get('joystick', {'x': 0, 'y': 0})
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # 存储摇杆和扳机原始数据(供 control_loop 的 _update_mobile_base() 使用)
        # 注意: 不通过 ControlGoal 传递,避免干扰机械臂的 POSITION_CONTROL 模式
        controller_key = f"{hand}Controller"
        if controller_key in self.control_loop.vr_raw_data:
            self.control_loop.vr_raw_data[controller_key]['joystick'] = joystick
            self.control_loop.vr_raw_data[controller_key]['trigger'] = trigger

        # Handle trigger for gripper control (线性控制)
        # 每帧都发送 trigger_value,实现 0-1 连续映射到夹爪角度 90°-0°
        if grip_active:
            gripper_goal = ControlGoal(
                arm=hand,
                gripper_closed=False,  # 这个参数会被忽略
                metadata={"trigger_value": trigger}  # 传递完整的 0-1 值
            )
            await self.send_goal(gripper_goal)

        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # 握把刚激活 - 设置原点并重置目标位置
                controller.grip_active = True
                controller.origin_position = position.copy()
                
                # 如果有四元数数据则直接使用
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                    controller.origin_rotation = controller.origin_quaternion
                else:
                    # 回退到欧拉角转换
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                    controller.origin_rotation = controller.origin_quaternion
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                controller.z_axis_rotation = 0.0
                controller.x_axis_rotation = 0.0
                
                # 向控制循环发送重置信号
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=None,
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                        "reset_target_to_current": True
                    }
                )
                await self.send_goal(reset_goal)
                
                logger.info(f"🔒 {hand.upper()} 握把已激活 - 控制 {hand} 机械臂")
            
            # 计算目标位置
            if controller.origin_position:
                relative_delta = compute_relative_position(
                    position, 
                    controller.origin_position, 
                    self.config.vr_to_robot_scale
                )
                
                # 计算 Z 轴旋转用于 wrist_roll 控制
                # 计算 X 轴旋转用于 wrist_flex 控制
                if controller.origin_quaternion is not None:
                    # 更新基于四元数的旋转跟踪
                    if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                        # 直接使用四元数数据
                        current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                        self.update_quaternion_rotation_direct(controller, current_quat)
                    else:
                        # 回退到欧拉角转换
                        self.update_quaternion_rotation(controller, rotation)
                    
                    # 从四元数获取累积旋转
                    controller.z_axis_rotation = self.extract_roll_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                    controller.x_axis_rotation = self.extract_pitch_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                
                # 创建位置控制目标
                # 注意：这里发送相对位置，control_loop 会处理将其添加到机器人当前位置
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,
                    wrist_roll_deg=-controller.z_axis_rotation,
                    wrist_flex_deg=-controller.x_axis_rotation,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.copy()
                    }
                )
                await self.send_goal(goal)
    
    async def handle_grip_release(self, hand: str):
        """处理控制器的握把释放。"""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # 发送 idle 目标以停止机械臂控制
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🔓 {hand.upper()} 握把已释放 - 机械臂控制停止")
    
    async def handle_trigger_release(self, hand: str):
        """处理控制器的扳机释放。"""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # 发送夹爪关闭目标 - 反向行为：扳机释放时夹爪关闭
            goal = ControlGoal(
                arm=hand,
                gripper_closed=True,  # 扳机释放时关闭夹爪
                metadata={"source": "vr_trigger_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🤏 {hand.upper()} 夹爪已关闭 (扳机释放)")
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """将欧拉角（度）转换为四元数 [x, y, z, w]。"""
        import math
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()
    
    def update_quaternion_rotation(self, controller: VRControllerState, current_euler: dict):
        """更新基于四元数的旋转跟踪。"""
        if not current_euler:
            return
        
        # 将当前欧拉角转换为四元数
        current_quat = self.euler_to_quaternion(current_euler)

        # 存储当前四元数以计算累积旋转
        controller.accumulated_rotation_quat = current_quat
    
    def update_quaternion_rotation_direct(self, controller: VRControllerState, current_quat: np.ndarray):
        """直接使用四元数数据更新基于四元数的旋转跟踪。"""
        if current_quat is None:
            return
        
        # 存储当前四元数以计算累积旋转
        controller.accumulated_rotation_quat = current_quat
    
    def extract_roll_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """从相对四元数旋转中提取绕 Z 轴的翻滚（roll）旋转。"""
        if current_quat is None or origin_quat is None:
            return 0.0
        
        try:
            # 计算相对旋转四元数（从原点到当前）
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # 将相对旋转投影到 Z 轴（翻滚）
            # 获取旋转向量（轴角表示）
            rotvec = relative_rotation.as_rotvec()

            # 旋转向量的 Z 分量表示绕 Z 轴的旋转（翻滚）
            z_rotation_rad = rotvec[2]
            z_rotation_deg = -np.degrees(z_rotation_rad)
            
            return z_rotation_deg
        except Exception as e:
            logger.warning(f"从四元数提取翻滚角时出错: {e}")
            return 0.0
    
    def extract_pitch_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """从相对四元数旋转中提取绕 X 轴的俯仰（pitch）旋转。"""
        if current_quat is None or origin_quat is None:
            return 0.0
        
        try:
            # 计算相对旋转四元数（从原点到当前）
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # 将相对旋转投影到 X 轴（俯仰）
            # 获取旋转向量（轴角表示）
            rotvec = relative_rotation.as_rotvec()

            # 旋转向量的 X 分量表示绕 X 轴的旋转（俯仰）
            x_rotation_rad = rotvec[0]
            x_rotation_deg = np.degrees(x_rotation_rad)
            
            return x_rotation_deg
        except Exception as e:
            logger.warning(f"从四元数提取俯仰角时出错: {e}")
            return 0.0
