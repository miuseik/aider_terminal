"""
输入提供者的基类和数据结枃。
"""

import asyncio
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from enum import Enum

# ======================== 全局输入活跃状态 ========================
# 使用 set 追踪活跃输入源，避免 VR/键盘互相覆盖
_ACTIVE_INPUT_SOURCES: set = set()


def mark_input_active(source: str):
    """标记输入源为活跃状态（source: 'vr' / 'keyboard'）"""
    _ACTIVE_INPUT_SOURCES.add(source)


def mark_input_inactive(source: str):
    """标记输入源为非活跃状态"""
    _ACTIVE_INPUT_SOURCES.discard(source)


def is_any_input_active() -> bool:
    """检查是否有任何输入源（VR 或键盘）处于活跃状态"""
    return len(_ACTIVE_INPUT_SOURCES) > 0

class ControlMode(Enum):
    """遥操作系统的控制模式。"""
    POSITION_CONTROL = "position"  # 位置控制
    IDLE = "idle"                  # 空闲

@dataclass
class ControlGoal:
    """从输入提供者发送的高级控制目标消息。"""
    arm: Literal["left", "right"]          # 机械臂标识: 左/右
    mode: Optional[ControlMode] = None     # 控制模式 (None = 不改变模式)
    target_position: Optional[np.ndarray] = None  # 机器人坐标系中的3D位置
    wrist_roll_deg: Optional[float] = None        # 腕部翻滚角度(度) arm5 Z轴
    wrist_flex_deg: Optional[float] = None        # 腕部弯曲(俯仰)角度(度) arm6 X轴
    wrist_yaw_deg: Optional[float] = None         # 腕部偏航角度(度) arm7 Y轴
    gripper_closed: Optional[bool] = None         # 夹爪状态 (None = 不改变)
    
    # 用于调试/监控的附加数据
    metadata: Optional[Dict[str, Any]] = None

class BaseInputProvider(ABC):
    """输入提供者的抽象基类。"""
    
    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False
    
    @abstractmethod
    async def start(self):
        """启动输入提供者。"""
        pass
    
    @abstractmethod
    async def stop(self):
        """停止输入提供者。"""
        pass
    
    async def send_goal(self, goal: ControlGoal):
        """向命令队列发送控制目标。"""
        try:
            await self.command_queue.put(goal)
        except Exception as e:
            # 处理队列满或其他错误
            pass 