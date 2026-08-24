"""运动学模块 — 统一导出所有 IK/FK 实现。

子包:
    pybullet/ — PyBullet FK + IK (SO100 机械臂, 多参考位姿)
    pink/     — Pink (Pinocchio) IK (OpenArmX 双臂)

便捷导入示例:
    from aiderminal.core.kinematic.pybullet.fk_ik import ForwardKinematics, IKSolver
    from aiderminal.core.kinematic.pybullet.utils import compute_relative_position
"""

import os

# 项目根目录: core/kinematic/ → 上 3 层 = aider_terminal/
# realpath 跟随 symlink（colcon --symlink-install 需要）
_PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
