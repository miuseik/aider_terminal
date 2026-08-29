"""PyBullet 运动学辅助工具函数。"""

import numpy as np


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    将 VR 控制器位置转换为机器人坐标系。

    VR 坐标系 (WebXR local-floor)：X=右，Y=上，Z=后（-Z=前）
    Aider base_link 坐标系 (URDF 实测)：+X=左，-Y=前，+Z=上

    操作假设：用户面朝机器人（face-to-face）。
    映射:
      VR 右(+X) → 机器人右(-X)   (真机实测: arm1-4 direction=-1 已镜像, 逻辑角需取反 X)
      VR 前(-Z) → 机器人前(-Y)
      VR 上(+Y) → 机器人上(+Z)
    """
    return np.array([
        -vr_pos['x'] * scale,   # VR 右(+X) → 机器人右(-X)
        vr_pos['z'] * scale,    # VR 前(-Z) → 机器人前(-Y)
        vr_pos['y'] * scale     # VR 上(+Y) → 机器人上(+Z)
    ])


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """计算从 VR 原点到当前位置的相对位置。"""
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'],
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    return vr_to_robot_coordinates(delta_vr, scale)
