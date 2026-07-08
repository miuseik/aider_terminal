"""
Aloha 机器人私有配置。
"""

# ======================== 机器人结构配置 ========================
NUM_JOINTS = 6
NUM_IK_JOINTS = 6
WRIST_FLEX_INDEX = 3
WRIST_ROLL_INDEX = 4
WRIST_YAW_INDEX = 0
GRIPPER_INDEX = 5
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]
ARM_JOINT_NAMES_LEFT = ["1", "2", "3", "4", "5", "6"]
ARM_JOINT_NAMES_RIGHT = ["1", "2", "3", "4", "5", "6"]
URDF_PATH = "URDF/SO100/so100.urdf"
ALOHA_URDF_PATH = "URDF/aloha/Aloha.urdf"
END_EFFECTOR_LINK_NAME = "Fixed_Jaw_tip_joint"
COMMON_MOTORS = {
    "shoulder_pan": [1, "sts3215"], "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"], "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"], "gripper": [6, "sts3215"],
}
URDF_TO_INTERNAL_NAME_MAP = {
    "1": "shoulder_pan", "2": "shoulder_lift",
    "3": "elbow_flex", "4": "wrist_flex",
    "5": "wrist_roll", "6": "gripper",
}

# ======================== 私有配置 ========================

# ---- 初始位置（度） ----
# 断开/复位时的目标角度（6 关节：shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper）
INITIAL_LEFT_ARM = [0, -100, 100, 60, 0, 0]
INITIAL_RIGHT_ARM = [0, -100, 100, 60, 0, 0]

# ---- 关节限位（度，通用 internal 名） ----
# 用于 custom IK (_clamp)
JOINT_LIMITS_DEG = {
    "shoulder_pan":  {"lower": -90, "upper": 90},
    "shoulder_lift": {"lower": -150, "upper": 30},
    "elbow_flex":    {"lower": -150, "upper": 30},
    "wrist_flex":    {"lower": -90, "upper": 90},
    "wrist_roll":    {"lower": -90, "upper": 90},
    "gripper":       {"lower": -90, "upper": 0},
}

# ---- 身体关节限位（Aloha 无身体关节） ----
BODY_JOINT_LIMITS = {}
