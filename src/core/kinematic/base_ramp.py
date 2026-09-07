"""底盘速度线性斜坡 (rate-limit)。

把方波式的目标速度 (键盘瞬间 0↔max 跳变) 平滑成线性过渡,
消除起步/停止的顿挫。每帧调用 step(target, dt) 返回平滑后的实际速度。

加速度上限可调:
    lin_accel : 线加速度上限 (m/s^2)
    ang_accel : 角加速度上限 (theta 原始值 / s)
"""


import math
from typing import Dict


class BaseVelocityRamp:
    def __init__(self, lin_accel: float = 0.01, ang_accel: float = 0.01):
        """
        Args:
            lin_accel: 线加速度上限 (m/s^2)
            ang_accel: 角加速度上限 (theta 原始值 / s)
        """
        self.lin_accel = lin_accel
        self.ang_accel = ang_accel
        self.actual: Dict[str, float] = {"x": 0.0, "y": 0.0, "theta": 0.0}

    def step(self, target: Dict[str, float], dt: float) -> Dict[str, float]:
        """按最大加速度把 actual 线性逼近 target, 返回平滑后的速度字典。"""
        limits = {
            "x": self.lin_accel,
            "y": self.lin_accel,
            "theta": self.ang_accel,
        }
        for k, amax in limits.items():
            max_dv = amax * dt
            diff = target.get(k, 0.0) - self.actual[k]
            if abs(diff) <= max_dv:
                self.actual[k] = target.get(k, 0.0)
            else:
                self.actual[k] += math.copysign(max_dv, diff)
        return dict(self.actual)
