#!/usr/bin/env python3
"""
三轮全向轮运动学计算工具 (AlohaMini 专用)
提供车体速度到三轮电机原始速度值的转换。
"""

from __future__ import annotations
import numpy as np

# ------------------------ 物理常量配置 ------------------------ #
WHEEL_RADIUS: float = 0.05  # 轮子半径 (m)
BASE_RADIUS: float = 0.125  # 轮心到底盘中心距离 (m)
MAX_RAW_SPEED: int = 3000   # 电机原始速度上限 (对应 Feetech Goal_Velocity)


def degps_to_raw(degps: float) -> int:
    """角速度 (deg/s) 转换为 Feetech 原始寄存器值 (-32767 ~ +32767)"""
    steps_per_deg = 4096.0 / 360.0
    mag = int(round(abs(degps) * steps_per_deg))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


def body_to_wheel_raw(
    x_cmd: float,
    y_cmd: float,
    theta_cmd_degps: float,
    *,
    wheel_radius: float = WHEEL_RADIUS,
    base_radius: float = BASE_RADIUS,
    max_raw: int = MAX_RAW_SPEED,
) -> dict[str, int]:
    """
    将车体目标速度转换为三个轮子的原始速度指令。
    
    Args:
        x_cmd: 前后线速度 (m/s), 正值为前进
        y_cmd: 左右线速度 (m/s), 正值为左平移
        theta_cmd_degps: 旋转角速度 (deg/s), 正值为逆时针
        
    Returns:
        字典: {"left_wheel": raw_val, "back_wheel": raw_val, "right_wheel": raw_val}
    """
    # 1. 构建速度向量
    theta_rad = np.radians(theta_cmd_degps)
    vel = np.array([-x_cmd, -y_cmd, theta_rad])

    # 2. 定义轮子安装角度 (根据 AlohaMini 实际硬件布局调整)
    # 假设: 左轮 240°, 后轮 0°, 右轮 120° (相对于车身坐标系)
    angles = np.radians(np.array([240, 0, 120]) - 90)
    M = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

    # 3. 逆解计算各轮线速度
    v_lin = M.dot(vel)  # m/s
    w_rad = v_lin / wheel_radius  # rad/s
    w_degps = np.degrees(w_rad)   # deg/s

    # 4. 限幅保护 (防止超出电机最大响应)
    steps_per_deg = 4096.0 / 360.0
    raw_abs = np.abs(w_degps) * steps_per_deg
    peak = float(np.max(raw_abs)) if raw_abs.size else 0.0
    if peak > max_raw and peak > 1e-6:
        w_degps = w_degps * (max_raw / peak)

    # 5. 转换为原始寄存器值
    raw_vals = [degps_to_raw(v) for v in w_degps]
    return {
        "base_left_wheel": raw_vals[0], 
        "base_back_wheel": raw_vals[1], 
        "base_right_wheel": raw_vals[2]
    }
