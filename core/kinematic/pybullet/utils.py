"""PyBullet 运动学辅助工具函数。"""

import numpy as np


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    将 VR 控制器位置转换为机器人坐标系。
    
    VR 坐标系：X=右，Y=上，Z=后（朝向用户）
    机器人坐标系：X=前，Y=左，Z=上
    """
    return np.array([
        -vr_pos['x'] * scale,   # VR +Z（后）-> 机器人 +X（前）
        vr_pos['z'] * scale,    # VR +X（右）-> 机器人 -Y（右）
        vr_pos['y'] * scale     # VR +Y（上）-> 机器人 +Z（上）
    ])


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """计算从 VR 原点到当前位置的相对位置。"""
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'],
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    return vr_to_robot_coordinates(delta_vr, scale)
