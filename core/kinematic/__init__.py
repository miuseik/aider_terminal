"""运动学模块 — 统一导出所有 IK/FK 实现。

子包:
    custom/   — 自定义纯 Python FK + DLS IK (URDF 解析, 双臂支持, 身体避碰)
    pybullet/ — PyBullet FK + IK (SO100 机械臂, 多参考位姿)
    pink/     — Pink (Pinocchio) IK (OpenArmX 双臂)

便捷导入示例:
    from core.kinematic.custom.fk_computer import FKComputer
    from core.kinematic.custom.ik_computer import DualArmIKComputer
    from core.kinematic.pybullet.fk_ik import ForwardKinematics, IKSolver
    from core.kinematic.pybullet.utils import compute_relative_position
"""

import os

# 项目根目录: core/kinematic/ → 上 3 层 = aider_terminal/
_PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
