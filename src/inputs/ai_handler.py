"""AI / 脚本输入提供者: 直接以绝对 TCP 位姿控制末端。

与 VR / 键盘不同, AI 不需要"相对偏移 + 握把激活"模型,
而是直接给出目标 TCP 的位置 + 姿态四元数, 走 ControlGoal.absolute_tcp 通道,
由控制循环统一喂给 IK (Pink) 并驱动真机 / 仿真可视化。

用法(测试或 AI 主控进程):
    from src.inputs.ai_handler import AIInputProvider
    ai = AIInputProvider(command_queue)
    await ai.enable("left")                       # 激活左臂位置控制
    await ai.send_tcp("left", [0.2,-0.1,0.5], [0,0,0,1])   # 绝对位姿
    await ai.disable("left")
"""
import asyncio
import numpy as np
from typing import Optional

from src.inputs.base import (
    BaseInputProvider, ControlGoal, ControlMode,
    mark_input_active, mark_input_inactive,
)


class AIInputProvider(BaseInputProvider):
    """以绝对 TCP 位姿控制末端的输入提供者 (AI / 脚本用)。"""

    SOURCE = "ai"

    def __init__(self, command_queue: asyncio.Queue):
        super().__init__(command_queue)
        self._active_arms: set = set()

    async def start(self):
        """启动 AI 输入提供者 (无外部资源, 仅置运行标志)。"""
        self.is_running = True

    async def stop(self):
        """停止 AI 输入提供者, 停用所有已激活臂。"""
        for arm in list(self._active_arms):
            await self.disable(arm)
        self.is_running = False

    async def enable(self, arm: str = "left"):
        """激活某臂的位置控制 (类似 VR 握把按下)。"""
        if arm in self._active_arms:
            return
        self._active_arms.add(arm)
        mark_input_active(self.SOURCE)
        await self.send_goal(ControlGoal(arm=arm, mode=ControlMode.POSITION_CONTROL))

    async def disable(self, arm: str = "left"):
        """停用某臂位置控制。"""
        if arm not in self._active_arms:
            return
        self._active_arms.discard(arm)
        if not self._active_arms:
            mark_input_inactive(self.SOURCE)
        await self.send_goal(ControlGoal(arm=arm, mode=ControlMode.IDLE))

    async def send_tcp(self, arm: str, position: np.ndarray,
                       orientation: Optional[np.ndarray] = None):
        """发送绝对 TCP 目标 (基座系)。

        position:    [x, y, z] (米)
        orientation: [x, y, z, w] (四元数, 可选; 缺省保持当前姿态)
        """
        if arm not in self._active_arms:
            await self.enable(arm)
        tcp = {"position": np.asarray(position, dtype=float)}
        if orientation is not None:
            tcp["orientation"] = np.asarray(orientation, dtype=float)
        await self.send_goal(ControlGoal(
            arm=arm,
            mode=ControlMode.POSITION_CONTROL,
            absolute_tcp=tcp,
        ))
