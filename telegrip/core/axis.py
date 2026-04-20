#!/usr/bin/env python3
"""
升降轴控制工具 (AlohaMini 专用)
提供高度(mm)到电机脉冲(ticks)的转换及安全限幅逻辑。
"""

# ==================== 物理常量配置 ==================== #
LIFT_MOTOR_ID = 11            # 升降轴电机 ID
LEAD_MM_PER_REV = 84.0        # 丝杠导程 (mm/圈)
STEPS_PER_REV = 4096.0        # 电机每圈脉冲数
MIN_HEIGHT_MM = 0.0           # 最低高度
MAX_HEIGHT_MM = 785.0         # 最高高度
CURRENT_LIMIT_MA = 1000.0     # 过流保护阈值


def height_mm_to_ticks(height_mm: float) -> int:
    """将目标高度 (mm) 转换为电机目标脉冲数 (ticks)"""
    revolutions = height_mm / LEAD_MM_PER_REV
    return int(round(revolutions * STEPS_PER_REV))


def ticks_to_height_mm(ticks: int) -> float:
    """将电机脉冲数 (ticks) 转换为当前高度 (mm)"""
    revolutions = ticks / STEPS_PER_REV
    return revolutions * LEAD_MM_PER_REV


def clamp_height(height_mm: float) -> float:
    """确保高度在安全范围内"""
    return max(MIN_HEIGHT_MM, min(MAX_HEIGHT_MM, height_mm))
