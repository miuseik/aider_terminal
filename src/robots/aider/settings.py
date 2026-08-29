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
INITIAL_LEFT_ARM = [10, 0, 0, 20, 0, 0, 0, 0]
INITIAL_RIGHT_ARM = [-10, 0, 0, -20, 0, 0, 0, 0]

# ---- 关节限位覆盖（度；lift_Link 为米） ----
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
    # 全部归 0：无外部真源时保持最保守死值，不接 servo_ids.yaml，不接任何动态源。
    # 若需恢复动作范围，改回对应关节的合理 [min,max] 即可。
    # -- 左臂 --
    "left_arm1": (0, 0),
    "left_arm2": (0, 0),
    "left_arm3": (0, 0),
    "left_arm4": (0, 0),
    "left_arm5": (0, 0),
    "left_arm6": (0, 0),
    "left_arm7": (0, 0),
    "left_arm8": (0, 0),
    "left_arm12": (0, 0),  # 夹爪右指: 随 arm8 反向联动 (arm12=-arm8)，无独立舵机
    # -- 右臂 --
    "right_arm1": (0, 0),
    "right_arm2": (0, 0),
    "right_arm3": (0, 0),
    "right_arm4": (0, 0),
    "right_arm5": (0, 0),
    "right_arm6": (0, 0),
    "right_arm7": (0, 0),
    "right_arm8": (0, 0),
    "right_arm12": (0, 0),  # 夹爪右指: 随 arm8 反向联动 (arm12=-arm8)，无独立舵机
    # -- 身体关节 --
    "waist_Link": (0, 0),
    "head_Link": (0, 0),
    "head_Link2": (0, 0),
    "lift_Link": (0.0, 0.0),  # prismatic 升降关节，单位米（非角度）
}


def apply_servo_limits_from_yaml(cfg: dict):
    """用 server 下发的 servo_ids.yaml 限位覆盖 JOINT_LIMIT_OVERRIDES（原地修改）。

    - aider_ik / adapter 均持有同一 dict 对象的引用，原地修改后 IK 构造与钳制层同步生效。
    - terminal 启动时（setup_kinematics 之前）由 set_servo_ids_config 调用，重启生效。
    - JOINT_LIMIT_OVERRIDES 初始全为 (0,0) 兜底占位，yaml 拉到后覆盖为真实限位。
    - 只覆盖 arm1~arm8（yaml 的 left_arm/right_arm），身体关节/升降保持 (0,0) 兜底。
    - yaml 缺值或非法值的关节保持原 (0,0) 不动。
    """
    if not isinstance(cfg, dict):
        return
    for side in ("left", "right"):
        arm = cfg.get(f"{side}_arm")
        if not isinstance(arm, dict):
            continue
        for i in range(1, 9):
            jname = f"{side}_arm{i}"
            node = arm.get(jname)   # yaml 键为 left_arm1 / right_arm1（带 side 前缀）
            if not isinstance(node, dict):
                continue
            mn = node.get("min_angle")
            mx = node.get("max_angle")
            if mn is None or mx is None:
                continue
            try:
                mn, mx = float(mn), float(mx)
            except (TypeError, ValueError):
                continue
            if mn >= mx:
                print(f"⚠️ 跳过非法限位 {jname}: [{mn}, {mx}] (min>=max)")
                continue
            if jname in JOINT_LIMIT_OVERRIDES:
                JOINT_LIMIT_OVERRIDES[jname] = (mn, mx)
                print(f"✅ 限位来自 yaml: {jname} -> [{mn}, {mx}]")
            else:
                print(f"⚠️ yaml 关节 {jname} 不在 JOINT_LIMIT_OVERRIDES，跳过")

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
        "left": INITIAL_LEFT_ARM,
        "right": INITIAL_RIGHT_ARM,
        "body": _BODY_ZERO,
    },
    "default": {
        "left": [10, 0, 0, 20, 0, 0, 0, 0],
        "right": [-10, -0, 0, -20, 0, 0, 0, 0],
        "body": _BODY_ZERO,
    },
    "zero": {
        "left": [0, 0, 0, 0, 0, 0, 0, 0],
        "right": [0, 0, 0, 0, 0, 0, 0, 0],
        "body": _BODY_ZERO,
    },
    "test": {
        "left": [30, 30, 30, 30, 30, 30, 30, 30],
        "right": [-30, -30, -30, -30, -30, -30, -30, -30],
        "body": _BODY_ZERO,
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
