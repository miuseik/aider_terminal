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
# 断开/复位时的目标角度（arm1~arm8），全零即归零姿态
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
#
# 2026-07-25 收紧：原值多为机械硬限位，并非碰撞驱动。经 PyBullet 单关节自碰撞扫描
# （从中性位向 ±200° 扫，检测该关节驱动连杆的新增穿透接触）实测，多个关节在到达
# 配置限位前就已自撞。下列危险关节已收紧到「单关节自碰撞边界」，再由
# SOFT_LIMIT_MARGIN_DEG(5°) 软限位留 5° 缓冲，确保指令永远到不了自撞点。
# 注：单关节-from-neutral 是一种保守场景，组合姿势边界更复杂，但足以证明原限位不安全。
JOINT_LIMIT_OVERRIDES = {
    # -- 左臂 --
    "left_arm1":  (-136, 136),     # 碰撞边界 ±200° 无撞，保持
    "left_arm2":  (-3, 91),        # 碰撞下限 -3°（中性位负向即自撞），原 -151 收紧
    "left_arm3":  (-54, 91),       # 碰撞下限 -54°，原 -91 收紧
    "left_arm4":  (-1, 136),       # 碰撞边界 [-161,157] 内，保持
    "left_arm5":  (-42, 180),      # 碰撞下限 -42°，原 -180 收紧
    "left_arm6":  (-90, 90),       # 碰撞边界 [-130,159] 内，保持
    "left_arm7":  (-22, 90),       # 碰撞下限 -22°，原 -90 收紧
    "left_arm8":  (-59, 1),        # 碰撞下限 -59°，原 -91 收紧
    # -- 右臂 --
    "right_arm1": (-136, 136),     # 碰撞边界 ±200° 无撞，保持
    "right_arm2": (-151, 3),       # 碰撞上限 +3°，原 91 收紧
    "right_arm3": (-59, 71),       # 碰撞 [-59,71]，原 ±91 收紧
    "right_arm4": (-136, 1),       # 碰撞边界 [-156,158] 内，保持
    "right_arm5": (-180, 47),      # 碰撞上限 +47°，原 180 收紧
    "right_arm6": (-90, 90),       # 碰撞边界 [-154,130] 内，保持
    "right_arm7": (-90, 22),       # 碰撞上限 +22°，原 90 收紧
    "right_arm8": (-59, 1),        # 碰撞下限 -59°，原 -91 收紧
    # -- 身体关节 --
    "waist_Link":  (-90, 30),   # 负值=鞠躬(前倾) 正值=下腰(后仰)；放开上限到 +30° 使中立直立(0°)可达，不再永久前倾（碰撞边界 [-132,185] 内）
    "head_Link":   (-60, 60),   # 碰撞边界 ±200° 无撞，保持
    "head_Link2":  (-30, 45),   # 碰撞边界 [-61,70] 内，保持
}

# ---- 软限位安全余量（度） ----
# 指令角度在物理限位（URDF/Pinocchio 读到的关节限位）基础上各留 SOFT_LIMIT_MARGIN_DEG 的余量，
# 确保电机永远顶不到物理死区（硬限位），避免堵转失能/掉电。
# 调大更保守（更不会撞限位），调小动作范围更大。
SOFT_LIMIT_MARGIN_DEG = 5

# ---- 姿态预设（角度为度，lift 为 mm） ----
# 用户可在前端动作列表中选择姿态，机器人按预设角度移动到目标位置。
# 键 = 姿态名称（前端会显示），值 = {left: [...], right: [...], body: {waist, head_yaw, head_pitch, lift}}
# body 关节: waist (腰旋转, °), head_yaw (头偏航, °), head_pitch (头俯仰, °), lift (升降高度, mm)
_BODY_ZERO = {"waist": 0, "head_yaw": 0, "head_pitch": 0, "lift": 0}
POSES = {
    "safe": {
        "left":  INITIAL_LEFT_ARM,
        "right": INITIAL_RIGHT_ARM,
        "body":  _BODY_ZERO,
    },
    "default": {
        "left":  [10, 30, 50, 50, 0, 0, 0, 0],
        "right": [-10, -30, -50, -50, 0, 0, 0, 0],
        "body":  _BODY_ZERO,
    },
    "zero": {
        "left":  [0, 0, 0, 0, 0, 0, 0, 0],
        "right": [0, 0, 0, 0, 0, 0, 0, 0],
        "body":  _BODY_ZERO,
    },
    "test": {
        "left":  [30, 30, 30, 30, 30, 30, 30, 30],
        "right": [-30, -30, -30, -30, -30, -30, -30, -30],
        "body":  _BODY_ZERO,
    },
}

# ---- 默认姿态（度） ----
# 启动姿态 / IK 舒适姿态 = 下拉框中的某个预设姿态，单一数据源，
# 不再在 settings.py 里维护一份与 POSES 重复的硬编码常量。
# 想改机器人默认姿态，改 DEFAULT_POSE_NAME 指向 POSES 里任意一个键即可。
# 默认用 "safe"（全 0，干净初始位）；"default" 是 [10,30,50,50] 舒适位，保留在下拉框供手动选用。
DEFAULT_POSE_NAME = "safe"


def get_default_posture() -> dict:
    """从 POSES[DEFAULT_POSE_NAME] 推导 IK 默认舒适姿态（joint_name→角度, 度）。

    与前端姿态下拉框共享同一份预设，避免再硬编码一份重复值。
    """
    pose = POSES.get(DEFAULT_POSE_NAME, {})
    result = {}
    for arm in ("left", "right"):
        angles = pose.get(arm, [])
        joint_names = ARM_JOINT_NAMES_LEFT if arm == "left" else ARM_JOINT_NAMES_RIGHT
        for i, deg in enumerate(angles):
            if i < len(joint_names):
                result[joint_names[i]] = deg
    return result
