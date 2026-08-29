"""
VR 处理器，用于处理来自 WebSocket 客户端的控制器数据。
处理 VR 控制器状态跟踪和控制目标生成。
"""

import asyncio
import json
import numpy as np
import math
import logging
from typing import Dict
from scipy.spatial.transform import Rotation as R

from src.inputs.base import BaseInputProvider, ControlGoal, ControlMode
from src.inputs.base import mark_input_active, mark_input_inactive
from src.config.settings import TelegripConfig
from src.core.kinematic.pybullet.utils import compute_relative_position

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
        self.y_axis_rotation = 0.0  # 用于 wrist_yaw (偏航)
        
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
        self.y_axis_rotation = 0.0


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
        print("✅ VR 处理器已启动")

    async def stop(self):
        """停止 VR 处理器。"""
        self.is_running = False
        print("🛑 VR 处理器已停止")
    
    async def process_message(self, message: str):
        """处理来自 WebSocket 客户端的 VR 控制器数据。"""
        try:
            data = json.loads(message)
            # 只处理 VR 控制器数据（无 action 字段的消息）
            # 带 action 的 API 命令已在 client.py 中路由到 control_loop
            await self.process_controller_data(data)
        except json.JSONDecodeError:
            print(f"⚠️ 收到非 JSON 消息: {message}")
        except Exception as e:
            print(f"❌ 处理数据错误: {e}")
    
    async def process_controller_data(self, data: Dict):
        """处理传入的 VR 控制器数据。"""
        
        # keyboard 模式下忽略 VR 输入
        if self.control_loop and self.control_loop.control_mode == "keyboard":
            return
        
        # 0. 头显: 提取相对 yaw/pitch, 存到 vr_raw_data 供 adapter 映射
        if 'headset' in data and data['headset']:
            self._feed_headset_raw(data['headset'])
        
        # 处理新的双控制器格式
        if 'leftController' in data and 'rightController' in data:
            left_data = data.get('leftController') or {}
            right_data = data.get('rightController') or {}
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
        
        # 存储摇杆/扳机/手柄原始位置(供 control_loop 的 _update_mobile_base() 与动作录制使用)
        # 注意: 不通过 ControlGoal 传递,避免干扰机械臂的 POSITION_CONTROL 模式
        controller_key = f"{hand}Controller"
        if controller_key in self.control_loop.vr_raw_data:
            self.control_loop.vr_raw_data[controller_key]['joystick'] = joystick
            self.control_loop.vr_raw_data[controller_key]['trigger'] = trigger
            self.control_loop.vr_raw_data[controller_key]['position'] = position

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
                mark_input_active("vr")
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
                controller.y_axis_rotation = 0.0
                
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
                
                print(f"🔒 {hand.upper()} 握把已激活 - 控制 {hand} 机械臂")
            
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
                    controller.y_axis_rotation = self.extract_yaw_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)

                # 相对旋转四元数（VR 坐标系，[x,y,z,w]），供 control_loop 做全位姿 TCP IK
                relative_quat = None
                if controller.origin_quaternion is not None and controller.accumulated_rotation_quat is not None:
                    try:
                        _o = R.from_quat(controller.origin_quaternion)
                        _c = R.from_quat(controller.accumulated_rotation_quat)
                        relative_quat = (_c * _o.inv()).as_quat().tolist()  # [x,y,z,w]
                    except Exception:
                        relative_quat = None

                # 创建位置控制目标
                # 注意：这里发送相对位置，control_loop 会处理将其添加到机器人当前位置
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,
                    # 符号约定: roll 经 extract_roll(negate=True)+外部负号双重取反为净正;
                    # flex/yaw 原只有外部负号导致方向反(仿真实测), 故去掉负号使净正。
                    wrist_roll_deg=-controller.z_axis_rotation,
                    wrist_flex_deg=controller.x_axis_rotation,
                    wrist_yaw_deg=controller.y_axis_rotation,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.copy(),
                        "relative_quaternion": relative_quat
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
            
            # 检查两个控制器是否都已释放
            if not self.left_controller.grip_active and not self.right_controller.grip_active:
                if not self.left_controller.trigger_active and not self.right_controller.trigger_active:
                    mark_input_inactive("vr")

            # 发送 idle 目标以停止机械臂控制
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            print(f"🔓 {hand.upper()} 握把已释放 - 机械臂控制停止")
    
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
            
            print(f"🤏 {hand.upper()} 夹爪已关闭 (扳机释放)")
    
    # ======================== 头显原始数据透传 ========================
    
    def _feed_headset_raw(self, headset: dict):
        """透传头显原始数据到 vr_raw_data。不做任何计算。"""
        self.control_loop.vr_raw_data['headset'] = headset
    
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
    
    def _extract_axis_angle(self, current_quat: np.ndarray, origin_quat: np.ndarray,
                            axis_index: int, axis_name: str, negate: bool = False) -> float:
        """从相对四元数旋转中提取指定轴的旋转角（度）。
        
        Args:
            axis_index: 0=X(俯仰), 1=Y(偏航), 2=Z(翻滚)
            axis_name: 用于错误日志
            negate: 是否取反
        """
        if current_quat is None or origin_quat is None:
            return 0.0
        try:
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            rotvec = relative_rotation.as_rotvec()
            deg = np.degrees(rotvec[axis_index])
            return -deg if negate else deg
        except Exception as e:
            print(f"从四元数提取{axis_name}角时出错: {e}")
            return 0.0

    def extract_roll_from_quaternion(self, current_quat, origin_quat):
        return self._extract_axis_angle(current_quat, origin_quat, 2, "翻滚", negate=True)

    def extract_pitch_from_quaternion(self, current_quat, origin_quat):
        return self._extract_axis_angle(current_quat, origin_quat, 0, "俯仰")

    def extract_yaw_from_quaternion(self, current_quat, origin_quat):
        return self._extract_axis_angle(current_quat, origin_quat, 1, "偏航")

    