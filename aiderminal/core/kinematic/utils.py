"""运动学/坐标变换辅助工具函数（与 IK 后端无关）。

VR → 机器人坐标映射按机器人类型分发:
  - aider: 8-DOF 双臂移动平台（修正映射）
  - aloha: SO100 6-DOF 双臂（原始映射）
"""

import numpy as np


def _vr_to_robot_aider(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """Aider 机器人坐标映射。

    VR 坐标系 (WebXR local-floor): X=右, Y=上, Z=后（-Z=前）
    Aider 基座坐标系 (由 URDF 轮位/臂位确定): +X=左, -Y=前, +Z=上

    映射:
      VR 前(-Z) → 机器人前(-Y)
      VR 右(+X) → 机器人右(-X，因为 +X 是左)
      VR 上(+Y) → 机器人上(+Z)
    """
    return np.array([
        -vr_pos['x'] * scale,   # VR 右(+X) → 机器人右(-X)
        vr_pos['z'] * scale,    # VR 前(-Z) → 机器人前(-Y)
        vr_pos['y'] * scale     # VR 上(+Y) → 机器人上(+Z)
    ])


def _vr_to_robot_aloha(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """Aloha/SO100 机器人坐标映射（原始版）。"""
    return np.array([
        -vr_pos['x'] * scale,
        vr_pos['z'] * scale,
        vr_pos['y'] * scale
    ])


_VR_COORD_MAP = {
    "aider": _vr_to_robot_aider,
    "aloha": _vr_to_robot_aloha,
}


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """按当前机器人类型分发 VR → 机器人坐标转换。"""
    from aiderminal.config.settings import get_robot_type
    fn = _VR_COORD_MAP.get(get_robot_type(), _vr_to_robot_aider)
    return fn(vr_pos, scale)


def vr_rotation_to_robot(rel_quat_xyzw) -> np.ndarray:
    """将 VR 坐标系下的相对旋转四元数 [x,y,z,w] 转换到机器人坐标系 [x,y,z,w]。

    与 _vr_to_robot_aider 的位置映射 [-vr_x, vr_z, vr_y] 是同一个旋转 R_POS，
    保证位置与姿态使用一致的坐标变换。R_POS @ m @ R_POS.T 为换基共轭。
    """
    from scipy.spatial.transform import Rotation as _R
    R_POS = np.array([[-1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0],
                      [0.0, 1.0, 0.0]])
    m = _R.from_quat(rel_quat_xyzw).as_matrix()
    m_robot = R_POS @ m @ R_POS.T
    return _R.from_matrix(m_robot).as_quat()


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0,
                              dead_zone: float = 0.005) -> np.ndarray:
    """计算从 VR 原点到当前位置的相对位置，带死区过滤微小抖动。

    Args:
        dead_zone: 位移阈值 (米)，小于此值的位移视为 0
    """
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'],
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    for axis in delta_vr:
        if abs(delta_vr[axis]) < dead_zone:
            delta_vr[axis] = 0.0
    return vr_to_robot_coordinates(delta_vr, scale)
