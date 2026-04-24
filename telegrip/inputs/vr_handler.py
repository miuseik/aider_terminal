"""
VR Handler for processing controller data from WebSocket client.
Handles VR controller state tracking and control goal generation.
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
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        self.origin_rotation = None
        
        # Quaternion-based rotation tracking (more stable than Euler)
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None  # Accumulated rotation as quaternion
        
        # Rotation tracking for wrist control
        self.z_axis_rotation = 0.0  # For wrist_roll
        self.x_axis_rotation = 0.0  # For wrist_flex (pitch)
        
        # Position tracking
        self.current_position = None
        
        # Rotation tracking
        self.origin_wrist_angle = 0.0
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0


class VRHandler(BaseInputProvider):
    """VR controller data handler - processes VR data and generates control goals."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None

    async def start(self):
        """Start the VR handler (no server needed)."""
        self.is_running = True
        logger.info("✅ VR Handler started")

    async def stop(self):
        """Stop the VR handler."""
        self.is_running = False
        logger.info("🛑 VR Handler stopped")
    
    async def process_message(self, message: str):
        """Process incoming VR controller data from WebSocket client."""
        try:
            data = json.loads(message)
            
            # Handle API-like commands
            if 'action' in data:
                await self.handle_api_command(data)
            else:
                # Handle VR controller data
                await self.process_controller_data(data)
        except json.JSONDecodeError:
            logger.warning(f"⚠️ Received non-JSON message: {message}")
        except Exception as e:
            logger.error(f"❌ Error processing data: {e}")
    
    async def handle_api_command(self, data: Dict):
        """Handle API-like commands."""
        action = data.get('action')
        logger.info(f"📡 VR API command: {action}")
        
        if action == 'get_status':
            # Return current status via callback if available
            status = {
                "type": "status_response",
                "robotEngaged": False,
                "keyboardEnabled": self.is_running,
                "vrConnected": True
            }
            if hasattr(self, 'on_status_callback'):
                await self.on_status_callback(status)
        
        elif action == 'enable_keyboard':
            logger.info("🎮 Keyboard control ENABLED")
        
        elif action == 'disable_keyboard':
            logger.info("🎮 Keyboard control DISABLED")
        
        elif action == 'robot_connect':
            logger.info("🔌 Robot connect command received")
        
        elif action == 'robot_disconnect':
            logger.info("🔌 Robot disconnect command received")
        
        elif action == 'keypress':
            key = data.get('key')
            event = data.get('event')
            logger.info(f"⌨️ Key {event}: {key}")
            if hasattr(self, 'web_keyboard_handler') and self.web_keyboard_handler:
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
        
        else:
            logger.warning(f"⚠️ Unknown VR command: {action}")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        
        # Handle new dual controller format
        if 'leftController' in data and 'rightController' in data:
            left_data = data['leftController']
            right_data = data['rightController']
            
            # Process left controller
            if left_data.get('position') and (left_data.get('gripActive', False) or left_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('left', left_data)
            elif not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            # Process right controller
            if right_data.get('position') and (right_data.get('gripActive', False) or right_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('right', right_data)
            elif not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return
        
        # Handle legacy single controller format
        hand = data.get('hand')
        
        # Handle explicit release messages
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
            
        # Process single controller data
        if hand and data.get('position') and (data.get('gripActive', False) or data.get('trigger', 0) > 0.5):
            await self.process_single_controller(hand, data)
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active
            
            # Send gripper control goal
            gripper_goal = ControlGoal(
                arm=hand,
                gripper_closed=not trigger_active,
                metadata={"source": "vr_trigger"}
            )
            await self.send_goal(gripper_goal)
            
            logger.info(f"🤏 {hand.upper()} gripper {'OPENED' if trigger_active else 'CLOSED'}")
        
        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_position = position.copy()
                
                # Use quaternion data directly if available
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                    controller.origin_rotation = controller.origin_quaternion
                else:
                    # Fallback to Euler angle conversion
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                    controller.origin_rotation = controller.origin_quaternion
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                controller.z_axis_rotation = 0.0
                controller.x_axis_rotation = 0.0
                
                # Send reset signal to control loop
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
                
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm")
            
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
            
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
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
            
            logger.info(f"🤏 {hand.upper()} gripper CLOSED (trigger released)")
    
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
