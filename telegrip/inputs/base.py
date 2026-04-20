"""
输入提供者的基类和数据结构定义。
"""

import asyncio
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from enum import Enum

class ControlMode(Enum):
    """遥操作系统的控制模式枚举。"""
    POSITION_CONTROL = "position"  # 位置控制模式：机械臂跟随目标点移动
    IDLE = "idle"                  # 空闲模式：停止控制，释放机械臂

@dataclass
class ControlGoal:
    """从输入提供者（如 VR 或键盘）发送给控制循环的高级控制目标消息。"""
    arm: Literal["left", "right"]  # 指定控制的机械臂：左臂或右臂
    mode: Optional[ControlMode] = None            # 控制模式切换（None 表示不改变当前模式）
    target_position: Optional[np.ndarray] = None  # 机器人坐标系下的 3D 目标位置 [x, y, z]
    wrist_roll_deg: Optional[float] = None        # 手腕旋转角度（Roll），单位：度
    wrist_flex_deg: Optional[float] = None        # 手腕弯曲角度（Flex/Pitch），单位：度
    gripper_closed: Optional[bool] = None         # 夹爪状态（True=闭合, False=张开, None=不改变）
    
    # --- 新增：移动底盘与升降轴控制字段 ---
    left_joystick: Optional[Dict[str, float]] = None   # 左摇杆数据 {'x': ..., 'y': ...}
    right_joystick: Optional[Dict[str, float]] = None  # 右摇杆数据 {'x': ..., 'y': ...}
    
    # 用于调试或监控的附加数据（例如：数据来源、相对位移标记等）
    metadata: Optional[Dict[str, Any]] = None

class BaseInputProvider(ABC):
    """输入提供者的抽象基类。所有具体的输入设备（VR、键盘等）都应继承此类。"""
    
    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue  # 用于向控制循环发送指令的异步队列
        self.is_running = False             # 运行状态标记
    
    @abstractmethod
    async def start(self):
        """启动输入提供者（例如：开启 WebSocket 监听或键盘钩子）。"""
        pass
    
    @abstractmethod
    async def stop(self):
        """停止输入提供者并清理资源。"""
        pass
    
    async def send_goal(self, goal: ControlGoal):
        """将一个控制目标（ControlGoal）发送到命令队列中。"""
        try:
            await self.command_queue.put(goal)
        except Exception as e:
            # 处理队列已满或其他潜在错误，防止程序崩溃
            pass 
