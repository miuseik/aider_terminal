"""
Aider 机器人私有配置。

所有可调参数集中在此，启动时自动将关节限位同步到 URDF，Pinocchio 原生加载。
修改关节限位/姿态偏好只需改这个文件，重启即生效，无需手动改 URDF。
"""

# ======================== 机器人结构配置 ========================
NUM_JOINTS = 8
NUM_IK_JOINTS = 8
WRIST_FLEX_INDEX = 5
WRIST_ROLL_INDEX = 4
WRIST_YAW_INDEX = 6
GRIPPER_INDEX = 7
JOINT_NAMES = ["arm1", "arm2", "arm3", "arm4", "arm5", "arm6", "arm7", "arm8"]
ARM_JOINT_NAMES_LEFT = ["left_arm1", "left_arm2", "left_arm3", "left_arm4",
                         "left_arm5", "left_arm6", "left_arm7", "left_arm8"]
ARM_JOINT_NAMES_RIGHT = ["right_arm1", "right_arm2", "right_arm3", "right_arm4",
                          "right_arm5", "right_arm6", "right_arm7", "right_arm8"]
URDF_PATH = "URDF/aider/urdf/aider_pro.SLDASM.urdf"
END_EFFECTOR_LINK_NAME = "left_arm8"
COMMON_MOTORS = {
    "arm1": [1, "sts3215"], "arm2": [2, "sts3215"], "arm3": [3, "sts3215"],
    "arm4": [4, "sts3215"], "arm5": [5, "sts3215"], "arm6": [6, "sts3215"],
    "arm7": [7, "sts3215"], "arm8": [8, "sts3215"],
}
URDF_TO_INTERNAL_NAME_MAP = {
    "left_arm1": "arm1", "left_arm2": "arm2", "left_arm3": "arm3",
    "left_arm4": "arm4", "left_arm5": "arm5", "left_arm6": "arm6",
    "left_arm7": "arm7", "left_arm8": "arm8",
    "right_arm1": "arm1", "right_arm2": "arm2", "right_arm3": "arm3",
    "right_arm4": "arm4", "right_arm5": "arm5", "right_arm6": "arm6",
    "right_arm7": "arm7", "right_arm8": "arm8",
}

# ======================== 私有配置 ========================

# ---- 初始位置（度） ----
# 断开/复位时的目标角度（arm1~arm8）
INITIAL_LEFT_ARM = [0, 0, 0, 0, 0, 0, 0, 0]
INITIAL_RIGHT_ARM = [0, 0, 0, 0, 0, 0, 0, 0]

# ---- 关节限位（度，通用 internal 名） ----
# 用于 custom IK (_clamp)，key 为 arm1~arm8
JOINT_LIMITS_DEG = {
    "arm1": {"lower": -90,  "upper": 90},
    "arm2": {"lower": -150, "upper": 30},
    "arm3": {"lower": -150, "upper": 30},
    "arm4": {"lower": -90,  "upper": 90},
    "arm5": {"lower": -45,  "upper": 45},
    "arm6": {"lower": -70,  "upper": 70},
    "arm7": {"lower": -45,  "upper": 45},
    "arm8": {"lower": -90,  "upper": 0},
}

# ---- 身体关节限位 ----
BODY_JOINT_LIMITS = {
    "waist_Link":  {"lower_deg": -90, "upper_deg": 90},
    "head_Link":   {"lower_deg": -60, "upper_deg": 60},
    "head_Link2":  {"lower_deg": -30, "upper_deg": 45},
    "lift_Link":   {"lower_m":    0.0, "upper_m":  0.5},
}

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
    "left_arm5":  (-180, 180),   # 腕部滚转：放宽到 ±180°，配合键盘/VR 控制
    "left_arm6":  (-90, 90),     # 腕部俯仰：放宽到 ±90°
    "left_arm7":  (-90, 90),     # 腕部偏航：放宽到 ±90°
    "left_arm8":  (-91, 1),      # 原始值，未修改
    # -- 右臂 --
    "right_arm1": (-136, 136),   # 原始 -136~136, 上限提到 -20° 防止肘顶身体
    "right_arm2": (-151, 91),    # 原始值，未修改
    "right_arm3": (-90, 10),     # 原始 -91~91, 缩到 ±10°（镜像左臂：右臂往外=负）
    "right_arm4": (-136, 1),     # 原始值，未修改
    "right_arm5": (-180, 180),   # 腕部滚转：放宽到 ±180°，配合键盘/VR 控制
    "right_arm6": (-90, 90),     # 腕部俯仰：放宽到 ±90°
    "right_arm7": (-90, 90),     # 腕部偏航：放宽到 ±90°
    "right_arm8": (-91, 1),      # 原始值，未修改
    # -- 身体关节 --
    "waist_Link":  (-90, 0),    # 负值=鞠躬, 不能下腰
    "head_Link":   (-60, 60),   # 转头 ±60°
    "head_Link2":  (-30, 45),   # 负值=低头, 正值=抬头
}

# ---- 姿态偏好（度） ----
# IK 求解时的默认舒适姿态，PostureTask 以此为目标
# TODO: 将 POSTURE 硬编码值迁移到 servo_ids.yaml 的 default_angle 字段，
#       统一从 YAML 读取默认姿态，避免 settings.py 和 YAML 各存一份。
#       参考: aider_server/sql/servo_ids.yaml (已加好 default_angle 字段)
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
