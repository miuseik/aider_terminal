"""
Aider 机器人配置。

所有可调参数集中在此，启动时自动将关节限位同步到 URDF，Pinocchio 原生加载。
修改关节限位/姿态偏好只需改这个文件，重启即生效，无需手动改 URDF。
"""

# ---- 关节限位覆盖（度） ----
# 启动时 _patch_urdf_limits() 将这些值写入 URDF <limit> 标签，
# Pinocchio 加载 URDF 时原生读入正确限位，零运行时开销。
# 键 = 关节名（与 URDF 中 joint name 一致），值 = (下限°, 上限°)
JOINT_LIMIT_OVERRIDES = {
    # -- 左臂 --
    "left_arm1":  (-136, 136),     # 原始 -136~136, 下限提到 20° 防止肘顶身体
    "left_arm2":  (-151, 91),    # 原始值，未修改
    "left_arm3":  (-10, 90),     # 原始 -91~91, 缩到 ±10°
    "left_arm4":  (-1, 136),     # 原始值，未修改
    "left_arm5":  (-91, 91),     # 原始值，未修改
    "left_arm6":  (-30, 30),     # 原始值，未修改
    "left_arm7":  (-30, 30),     # 原始值，未修改
    "left_arm8":  (-91, 1),      # 原始值，未修改
    # -- 右臂 --
    "right_arm1": (-136, 136),   # 原始 -136~136, 上限提到 -20° 防止肘顶身体
    "right_arm2": (-151, 91),    # 原始值，未修改
    "right_arm3": (-10, 90),     # 原始 -91~91, 缩到 ±10°
    "right_arm4": (-136, 1),     # 原始值，未修改
    "right_arm5": (-91, 91),     # 原始值，未修改
    "right_arm6": (-30, 30),     # 原始值，未修改
    "right_arm7": (-30, 30),     # 原始值，未修改
    "right_arm8": (-91, 1),      # 原始值，未修改
    # -- 身体关节 --
    "waist_Link":  (-90, 0),    # 负值=鞠躬, 不能下腰
    "head_Link":   (-60, 60),   # 转头 ±60°
    "head_Link2":  (-30, 45),   # 负值=低头, 正值=抬头
}

# ---- 姿态偏好（度） ----
# IK 求解时的默认舒适姿态，PostureTask 以此为目标
POSTURE = {
    # "left_arm1":   45,
    # "left_arm2":   20,
    # "left_arm3":   0,    # arm3 now stays near 0°
    # "left_arm4":   50,
    # "left_arm5":   0,
    # "left_arm6":   0,
    # "left_arm7":   0,
    # "left_arm8":   0,
    # "right_arm1": -45,
    # "right_arm2": -20,
    # "right_arm3":  0,    # arm3 now stays near 0°
    # "right_arm4": -50,
    # "right_arm5":  0,
    # "right_arm6":  0,
    # "right_arm7":  0,
    # "right_arm8":  0,
    "left_arm1": 10,
    "left_arm2": 30,  # 肩稍向后/下
    "left_arm3": 50,
    "left_arm4": 50,  # 肘微弯（左臂 axis=-1，正=弯）
    "left_arm5": 0,
    "left_arm6": 0,
    "left_arm7": 0,
    "left_arm8": 0,
    "right_arm1": -10,
    "right_arm2": -30,  # 肩稍向后/下
    "right_arm3": -50,
    "right_arm4": -50,  # 肘微弯（右臂 axis=1，负=弯）
    "right_arm5": 0,
    "right_arm6": 0,
    "right_arm7": 0,
    "right_arm8": 0,
}
