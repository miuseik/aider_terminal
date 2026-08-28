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
# ★ 数据源（2026-08-27 起）：Server 端 servo_ids.yaml 的 min_angle/max_angle 是唯一真源，
#   Terminal 启动时经 apply_joint_limits_from_servo() 原地覆盖本表（yaml 缺失的关节
#   保留下列兜底值）。前端改限位 → 写 yaml → 热更新钳制层；IK 模型内限位重启生效。
#
# 2026-07-25 收紧：原值多为机械硬限位，并非碰撞驱动。经 PyBullet 单关节自碰撞扫描
# （从中性位向 ±200° 扫，检测该关节驱动连杆的新增穿透接触）实测，多个关节在到达
# 配置限位前就已自撞。下列危险关节已收紧到「单关节自碰撞边界」，再由
# SOFT_LIMIT_MARGIN_DEG(5°) 软限位留 5° 缓冲，确保指令永远到不了自撞点。
# 注：单关节-from-neutral 是一种保守场景，组合姿势边界更复杂，但足以证明原限位不安全。
# ★ 占位值（已被主动清空，2026-08-28）：
# 全部设为 (0, 0) 安全占位：若 Server 配置未成功替换，限位即「锁死在中位」，
# 机器人绝不会乱转（比 ±1000 宽占位安全得多）。
# 唯一真源是 Server 端 servo_ids.yaml 的 min_angle/max_angle，
# 必须在 IK 构建前经 apply_joint_limits_from_servo() 原地覆盖本表。
# 运行时若关节被锁死无法动，即证明 Server 配置未成功替换（链路断了），
# docker compose logs 里会打印 preload/apply 的详细日志，自查即可。
JOINT_LIMIT_OVERRIDES = {
    # -- 左臂 --
    "left_arm1": (0, 0),
    "left_arm2": (0, 0),
    "left_arm3": (0, 0),
    "left_arm4": (0, 0),
    "left_arm5": (0, 0),
    "left_arm6": (0, 0),
    "left_arm7": (0, 0),
    "left_arm8": (0, 0),
    "left_arm12": (0, 0),
    # -- 右臂 --
    "right_arm1": (0, 0),
    "right_arm2": (0, 0),
    "right_arm3": (0, 0),
    "right_arm4": (0, 0),
    "right_arm5": (0, 0),
    "right_arm6": (0, 0),
    "right_arm7": (0, 0),
    "right_arm8": (0, 0),
    "right_arm12": (0, 0),
    # -- 身体关节 --
    "waist_Link": (0, 0),
    "head_Link": (0, 0),
    "head_Link2": (0, 0),
    "lift_Link": (0.0, 0.0),  # prismatic 升降关节，单位米，占位
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


# ---- 限位真源接入：servo_ids.yaml ----
# Server 下发的扁平舵机配置中，这些 part 的 min_angle/max_angle 是关节逻辑限位（度）。
# base(连续轮)/lift_axis(电机角度≠URDF米制) 不参与，保持本文件兜底值。
_LIMIT_SERVO_PARTS = ("left_arm", "right_arm", "neck", "waist")
# neck = Server rename 后的 yaml body_joints(含 head)
# waist = yaml 顶层独立段 waist(含 waist_Link)


def apply_joint_limits_from_servo(servo_config: dict) -> int:
    """从 servo_ids.yaml 配置提取 min/max_angle，原地覆盖 JOINT_LIMIT_OVERRIDES。

    必须在 IK 模型构建（_patch_urdf_limits）之前调用，IK/软限位/钳制才全链路一致。
    yaml 中缺失的关节保留本文件占位值（±1000，便于暴露未替换）。返回覆盖条数。
    """
    if not servo_config:
        print("⚠️ apply_joint_limits_from_servo: servo_config 为空，未覆盖任何关节！"
              " 限位仍使用 settings.py 占位 ±1000（无限位）")
        return 0
    n = 0
    applied = []
    for part in _LIMIT_SERVO_PARTS:
        part_cfg = servo_config.get(part) or {}
        if not isinstance(part_cfg, dict):
            continue
        for jname, info in part_cfg.items():
            if not isinstance(info, dict):
                continue
            lo, hi = info.get("min_angle"), info.get("max_angle")
            if lo is None or hi is None:
                print(f"⚠️ servo 配置关节 {jname} 缺 min_angle/max_angle，跳过")
                continue
            JOINT_LIMIT_OVERRIDES[jname] = (float(lo), float(hi))
            applied.append(f"{jname}=({lo},{hi})")
            n += 1
    print(f"✅ apply_joint_limits_from_servo: 成功覆盖 {n} 条关节限位")
    for line in applied:
        print(f"   -> {line}")
    still_placeholder = [k for k, v in JOINT_LIMIT_OVERRIDES.items() if v in ((0, 0), (0.0, 0.0))]
    if still_placeholder:
        print(f"⚠️ 以下关节仍是 settings.py 占位 (0,0)（Server 未提供，已锁死中位）: {still_placeholder}")
    return n


def preload_servo_limits() -> int:
    """启动时从 Server 拉取 servo_ids.yaml 并灌入全局 JOINT_LIMIT_OVERRIDES。

    在 IK / visualizer 加载 URDF（_patch_urdf_limits）之前调用一次，
    使 PyBullet 模型与 Pinocchio IK 模型都用 servo_ids.yaml 真源限位。
    不依赖 control_loop / robot_interface —— 由 aider_ik 模块加载时直接触发。
    失败则静默保留本文件兜底值。
    """
    try:
        from aiderminal.comm.api.client import ServerAPIClient
        print("🔄 preload_servo_limits: 正在从 Server 拉取 servo_ids.yaml ...")
        cfg = ServerAPIClient().get_servo_ids_config()
        if cfg:
            n = apply_joint_limits_from_servo(cfg)
            print(f"✅ preload_servo_limits: 拉取成功，覆盖 {n} 条")
            return n
        else:
            print("⚠️ preload_servo_limits: Server 返回空配置！限位将保持 settings.py 占位 ±1000（无限位）")
    except Exception as e:
        print(f"⚠️ preload_servo_limits: 拉取失败，使用 settings.py 占位 ±1000（无限位）: {e!r}")
    return 0
