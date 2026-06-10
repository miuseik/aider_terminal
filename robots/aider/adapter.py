"""
Aider 机器人完整适配器。

统一封装该机器人所需的全部计算逻辑:
  - 8-DOF 双臂 IK/FK 解算（DLS 数值法）
  - 4 轮麦克纳姆轮底盘运动学
  - 升降轴高度积分
  - 腰/头关节直接控制
  - PyBullet 仿真可视化更新
  - 动作字典构建（供真机发送和仿真使用）

关节层次结构 (URDF):
  base_link → lift_Link (prismatic) → waist_Link (revolute)
    → head_Link (revolute, yaw) → head_Link2 (revolute, pitch)
    → left_arm1~8 (8 revolute joints)
    → right_arm1~8 (8 revolute joints)
    → whel_Link1~4 (continuous, 4 轮)

双臂关节角色:
  arm1: X轴旋转 (shoulder pan, ±180°)
  arm2: Y轴旋转 (shoulder lift)
  arm3: Z轴旋转 (elbow flex)
  arm4: X轴旋转 (wrist)
  arm5: Z轴旋转 (wrist)
  arm6: X轴旋转 (wrist)
  arm7: Y轴旋转 (wrist)
  arm8: X轴旋转 (gripper/tool)
"""

import numpy as np
import math
from typing import Optional, Dict, List, Tuple
from scipy.spatial.transform import Rotation as R

from core.kinematic.pink.aider_ik import AiderPinkSolver
from config.settings import (
    TelegripConfig, NUM_JOINTS, NUM_IK_JOINTS,
    GRIPPER_INDEX, ARM_JOINT_NAMES_LEFT, ARM_JOINT_NAMES_RIGHT,
)
from robots.aider.settings import JOINT_LIMIT_OVERRIDES

# ======================== 麦克纳姆轮常量 ========================
WHEEL_RADIUS: float = 0.05
# 轮子布局参数 (从 URDF 提取)
HALF_TRACK: float = 0.201       # 半轮距 (x方向)
HALF_WHEELBASE: float = 0.24   # 半轴距 (y方向)
MECANUM_K: float = HALF_TRACK + HALF_WHEELBASE  # 旋转耦合系数

MAX_RAW_SPEED: int = 3000
ROTATION_GAIN: float = 100.0
MAX_LIFT_SPEED_MPS: float = 0.1
MAX_LIFT_SPEED_RAW: int = 1500

# 四轮命名: FL=前左, FR=前右, RL=后左, RR=后右
WHEEL_NAMES = ["whel_Link1", "whel_Link2", "whel_Link3", "whel_Link4"]


def _degps_to_raw(degps: float) -> int:
    """角速度 (deg/s) → Feetech 原始寄存器值 (-32767~+32767)。"""
    steps_per_deg = 4096.0 / 360.0
    mag = int(round(abs(degps) * steps_per_deg))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


class AiderAdapter:
    """Aider 机器人完整控制适配器。

    职责:
      - 8-DOF 双臂 IK/FK 解算
      - 双臂关节角度管理（含限位、夹爪映射）
      - 4 轮麦克纳姆轮底盘运动学
      - 升降轴速度 → 高度积分
      - 腰/头关节控制
      - 仿真可视化统一更新
      - 构建完整机器人动作字典
    """

    def __init__(self):
        # ---- 运动学 ----
        self.ik_solver: Optional[AiderPinkSolver] = None

        # ---- 8-DOF 关节状态 ----
        self.left_angles = np.zeros(NUM_JOINTS)
        self.right_angles = np.zeros(NUM_JOINTS)

        # ---- 身体关节 ----
        self.waist_angle: float = 0.0     # 腰部旋转 (rad)
        self.head_yaw: float = 0.0        # 头 yaw (rad)
        self.head_pitch: float = 0.0      # 头 pitch (rad)

        # ---- 头显姿态基准 ----
        self._headset_origin_quat: Optional[np.ndarray] = None
        self._headset_origin_z: Optional[float] = None  # 坐标 Z 基准 (前/后)

        # ---- 底盘速度 ----
        self.base_vx: float = 0.0
        self.base_vy: float = 0.0
        self.base_vtheta: float = 0.0

        # ---- 升降轴 ----
        self.lift_velocity: float = 0.0
        self.lift_height_mm: float = 0.0

        # ---- 状态标记 ----
        self.is_setup: bool = False
        self.config: Optional[TelegripConfig] = None
        self.visualizer = None

        # ---- 外部引用 ----
        self._physics_client = None
        self._aider_robot_id: Optional[int] = None

    # ======================== 初始化 ========================

    def setup(self, visualizer, config: TelegripConfig) -> None:
        """初始化适配器。

        visualizer 必须已经加载 Aider URDF 并完成 joint mapping。
        """
        self.config = config
        self.visualizer = visualizer

        # 初始化 Pink IK 求解器（自含 Pinocchio URDF 模型，限位来自 URDF）
        self.ik_solver = AiderPinkSolver()

        # 从 IK 模型提取每个臂的关节限位（度），用于 update_arm_angles 钳制
        # 限位与 settings.py 同步，是唯一数据源
        m = self.ik_solver.robot.model
        self._arm_limits_deg: Dict[str, np.ndarray] = {}
        for arm in ("left", "right"):
            limb = []
            for jname in self.ik_solver.arm_joints[arm]:
                qi = self.ik_solver._q_idx(jname)
                limb.append([np.degrees(m.lowerPositionLimit[qi]),
                            np.degrees(m.upperPositionLimit[qi])])
            self._arm_limits_deg[arm] = np.array(limb)

        # 从 solver 的姿态偏好同步初始角度，启动就在舒适位
        self.left_angles = self.ik_solver.get_posture("left")
        self.right_angles = self.ik_solver.get_posture("right")

        if visualizer and visualizer.is_connected:
            self._physics_client = visualizer.physics_client
            self._aider_robot_id = visualizer.aider_id

        self.is_setup = True
        print("[AiderAdapter] 适配器初始化完成 (8-DOF IK/FK + 4轮 + 腰/头)")

    # ======================== IK / FK ========================

    def compute_fk(self, arm: str, angles_deg: np.ndarray) -> np.ndarray:
        """正运动学: 8 关节角度 → 末端位置 (base_link 坐标系, 米)。"""
        if self.ik_solver is None:
            return np.array([0.3, 0.0, 0.5])
        body = {
            "lift_Link": self.lift_height_mm / 1000.0,
            "waist_Link": self.waist_angle,
            "head_Link": self.head_yaw,
            "head_Link2": self.head_pitch,
        }
        pos = self.ik_solver.forward_kinematics(arm, angles_deg, body)
        if pos is None:
            return np.array([0.3, 0.0, 0.5])
        return pos

    def solve_ik(self, arm: str, target_position: np.ndarray,
                 current_angles: Optional[np.ndarray] = None) -> np.ndarray:
        """逆运动学: 末端位置 → 8 关节角度 (度)。"""
        if current_angles is None:
            current_angles = self._get_angles(arm)

        if self.ik_solver is None:
            return current_angles

        body = {
            "lift_Link": self.lift_height_mm / 1000.0,
            "waist_Link": self.waist_angle,
            "head_Link": self.head_yaw,
            "head_Link2": self.head_pitch,
        }

        sol = self.ik_solver.solve(
            arm=arm,
            target_position=np.array(target_position),
            current_angles=current_angles,
            body_state=body,
        )

        if sol is None:
            return current_angles
        return sol

    # ======================== 关节管理 ========================

    def _get_angles(self, arm: str) -> np.ndarray:
        if arm == "left":
            return self.left_angles.copy()
        return self.right_angles.copy()

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray,
                          wrist_flex: float, wrist_roll: float,
                          gripper: float, wrist_yaw: float = 0.0) -> np.ndarray:
        """更新指定臂的关节角度（含限位钳制）。

        Aider 8-DOF: IK 解全部 8 关节，wrist_roll/flex/yaw 覆盖 arm5/6/7。
        """
        angles = ik_angles.copy()

        # arm5 = wrist roll (Z轴), arm6 = wrist flex (X轴), arm7 = wrist yaw (Y轴), arm8 = gripper
        if len(angles) >= 5:
            angles[4] = wrist_roll    # arm5 (Z轴, 前臂旋前/旋后)
        if len(angles) >= 6:
            angles[5] = wrist_flex    # arm6 (X轴)
        if len(angles) >= 7:
            angles[6] = wrist_yaw     # arm7 (Y轴, 偏航)
        if len(angles) >= 8:
            angles[7] = gripper       # arm8 (夹爪)

        if arm == "left":
            self.left_angles = self._clamp_arm_angles(arm, angles)
        else:
            self.right_angles = self._clamp_arm_angles(arm, angles)
        return angles

    def _clamp_arm_angles(self, arm: str, angles: np.ndarray) -> np.ndarray:
        """钳制臂关节角度到 URDF 限位（与 settings.py 同步）。"""
        limits = self._arm_limits_deg.get(arm)
        if limits is None:
            return angles
        for i in range(min(len(angles), len(limits))):
            angles[i] = np.clip(angles[i], limits[i, 0], limits[i, 1])
        return angles

    def apply_gripper_from_trigger(self, arm: str, trigger_value: float) -> None:
        """根据 VR 扳机值 (0~1) 设置夹爪角度，钳制到 URDF 限位。"""
        gripper_angle = -trigger_value * 90.0
        limits = self._arm_limits_deg.get(arm)
        if limits is not None and len(limits) > GRIPPER_INDEX:
            gripper_angle = np.clip(gripper_angle, limits[GRIPPER_INDEX, 0], limits[GRIPPER_INDEX, 1])
        if arm == "left" and len(self.left_angles) > GRIPPER_INDEX:
            self.left_angles[GRIPPER_INDEX] = gripper_angle
        elif arm == "right" and len(self.right_angles) > GRIPPER_INDEX:
            self.right_angles[GRIPPER_INDEX] = gripper_angle

    # ======================== 身体关节 ========================

    def set_body_joint_delta(self, joint_name: str, delta_rad: float) -> None:
        """增量更新身体关节（腰/头），钳制到 URDF 限位内。"""
        lo_deg, hi_deg = JOINT_LIMIT_OVERRIDES.get(joint_name, (-180, 180))
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        if joint_name == "waist_Link":
            self.waist_angle = np.clip(self.waist_angle + delta_rad, lo, hi)
        elif joint_name == "head_Link":
            self.head_yaw = np.clip(self.head_yaw + delta_rad, lo, hi)
        elif joint_name == "head_Link2":
            self.head_pitch = np.clip(self.head_pitch + delta_rad, lo, hi)
    
    def set_body_joint_absolute(self, joint_name: str, angle_rad: float) -> None:
        """绝对设置身体关节角度（头显 VR），钳制到 URDF 限位内。"""
        lo_deg, hi_deg = JOINT_LIMIT_OVERRIDES.get(joint_name, (-180, 180))
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        val = float(np.clip(angle_rad, lo, hi))
        if joint_name == "waist_Link":
            self.waist_angle = val
        elif joint_name == "head_Link":
            self.head_yaw = val
        elif joint_name == "head_Link2":
            self.head_pitch = val

    # ======================== 头显 → 身体关节映射 ========================
    # 腰 (waist_Link):  绕X轴, 负值=鞠躬, 正值=下腰  1 DOF  -90°~0° (坐标→弯腰)
    # 脖 (head_Link):   绕Z轴, 转头/偏航  1 DOF  ±60°
    # 脖 (head_Link2):  绕X轴, 负值=低头, 正值=抬头  1 DOF  -30°~45° (俯仰→低头)
    HEAD_YAW_TO_NECK   = 1.0   # 头显 yaw  → 脖子转头 (100%)
    HEAD_PITCH_TO_NECK = 1.0   # 头显 pitch → 脖子低头 (100%)
    HEAD_POS_TO_WAIST  = 3.0   # 头显坐标 → 弯腰 (增益)

    def feed_headset_raw(self, headset: dict):
        """头显原始数据 → 身体关节映射。
        
        坐标 → 腰 (waist_Link), 俯仰 → 脖子低头 (head_Link2), 偏航 → 脖子转头 (head_Link)
        WebXR: +X=右, +Y=上, +Z=后
        """
        pos = headset.get('position') or {}
        quat = headset.get('quaternion') or {}

        # --- 坐标 → 腰 (Z 前/后偏移) ---
        if pos and 'z' in pos:
            z = float(pos['z'])
            if self._headset_origin_z is None:
                self._headset_origin_z = z
            else:
                dz = (z - self._headset_origin_z)  # 前倾 Z↓ → dz<0 → 弯腰(负)
                self.set_body_joint_absolute("waist_Link", dz * self.HEAD_POS_TO_WAIST)

        # --- 俯仰/偏航 → 脖子 ---
        if quat and all(k in quat for k in ['x', 'y', 'z', 'w']):
            current = np.array([quat['x'], quat['y'], quat['z'], quat['w']])
            if self._headset_origin_quat is None:
                self._headset_origin_quat = current.copy()
            else:
                origin_rot = R.from_quat(self._headset_origin_quat)
                current_rot = R.from_quat(current)
                relative_rot = current_rot * origin_rot.inv()
                rotvec = relative_rot.as_rotvec()
                yaw_rad = float(rotvec[1])
                pitch_rad = float(rotvec[0])
                self.set_body_joint_absolute("head_Link",  -yaw_rad   * self.HEAD_YAW_TO_NECK)
                self.set_body_joint_absolute("head_Link2", -pitch_rad * self.HEAD_PITCH_TO_NECK)

    # ======================== 4 轮底盘运动学 ========================

    def set_base_velocity(self, vx: float, vy: float, vtheta: float) -> None:
        """设置底盘目标速度（车身坐标系）。"""
        self.base_vx = vx
        self.base_vy = vy
        self.base_vtheta = vtheta

    def _compute_wheel_omegas(self) -> np.ndarray:
        """计算四轮当前角速度（rad/s），用于仿真轮子旋转。"""
        theta_scaled = self.base_vtheta * ROTATION_GAIN
        k = MECANUM_K
        v_linear = np.array([
            self.base_vx - self.base_vy - k * theta_scaled,
            self.base_vx + self.base_vy + k * theta_scaled,
            self.base_vx + self.base_vy - k * theta_scaled,
            self.base_vx - self.base_vy + k * theta_scaled,
        ])
        return v_linear / WHEEL_RADIUS

    def compute_wheel_speeds(self, vx: float = None, vy: float = None,
                             vtheta: float = None) -> Dict[str, int]:
        """底盘速度 → 四轮原始速度指令 (Feetech 寄存器值)。

        四轮麦克纳姆轮逆运动学:
          v1 = (1/R) * (vx - vy - k*wz)   # FL
          v2 = (1/R) * (vx + vy + k*wz)   # FR
          v3 = (1/R) * (vx + vy - k*wz)   # RL
          v4 = (1/R) * (vx - vy + k*wz)   # RR

        其中 k = HALF_TRACK + HALF_WHEELBASE

        Returns:
            {"whel_Link1": raw, "whel_Link2": raw, ...}
        """
        x = vx if vx is not None else self.base_vx
        y = vy if vy is not None else self.base_vy
        t = vtheta if vtheta is not None else self.base_vtheta

        # 车身坐标系: x=前, y=左, ω=左转正
        # 注意: URDF Y=前 (SolidWorks convention)
        # 这里保持与键盘映射一致: arrowup=前进
        theta_scaled = t * ROTATION_GAIN
        k = MECANUM_K

        # 四轮速度 (线速度 m/s)
        v_linear = np.array([
            x - y - k * theta_scaled,   # whel_Link1 (FR in URDF coords)
            x + y + k * theta_scaled,   # whel_Link2 (FL)
            x + y - k * theta_scaled,   # whel_Link3 (RL)
            x - y + k * theta_scaled,   # whel_Link4 (RR)
        ])

        w_rad = v_linear / WHEEL_RADIUS
        w_degps = np.degrees(w_rad)

        # 限幅
        steps_per_deg = 4096.0 / 360.0
        raw_abs = np.abs(w_degps) * steps_per_deg
        peak = float(np.max(raw_abs)) if raw_abs.size else 0.0
        if peak > MAX_RAW_SPEED and peak > 1e-6:
            w_degps = w_degps * (MAX_RAW_SPEED / peak)

        raw_vals = [_degps_to_raw(v) for v in w_degps]
        return {
            "whel_Link1": raw_vals[0],
            "whel_Link2": raw_vals[1],
            "whel_Link3": raw_vals[2],
            "whel_Link4": raw_vals[3],
        }

    # ======================== 升降轴 ========================

    def set_lift_velocity(self, velocity: float) -> None:
        self.lift_velocity = velocity

    def step_lift_height(self, dt: float) -> float:
        speed_mps = (self.lift_velocity / 1000.0) * MAX_LIFT_SPEED_MPS
        delta_m = speed_mps * dt
        old_m = self.lift_height_mm / 1000.0
        new_m = old_m + delta_m
        self.lift_height_mm = new_m * 1000.0
        return new_m

    # ======================== 硬件命令构建 ========================

    def build_hardware_actions(self, servo_ids: dict, servo_ports: dict = None) -> dict:
        """根据当前状态和舵机配置，构建结构化的硬件命令。

        本方法完全封装 Aider 特有的底盘/轮子/升降轴映射逻辑，
        调用方（robot_interface）只需无脑派发返回的命令。

        Args:
            servo_ids: 扁平舵机配置 {left_arm: {joint: {id,...}}, right_arm: ..., base: ..., ...}
            servo_ports: 运行时端口发现结果 {part_name: port}

        Returns:
            {
                "position_commands": [{"port": str, "targets": {servo_id: angle}}, ...],
                "speed_commands":    [{"port": str, "targets": {servo_id: speed}}, ...],
            }
        """
        if servo_ports is None:
            servo_ports = {}
        actions = {"position_commands": [], "speed_commands": []}

        # --- 左臂（位置控制） ---
        left_port = servo_ports.get("left_arm")
        if left_port:
            targets = {}
            for i, (_jname, jinfo) in enumerate(servo_ids.get("left_arm", {}).items()):
                if isinstance(jinfo, dict) and i < len(self.left_angles):
                    targets[jinfo["id"]] = float(self.left_angles[i])
            if targets:
                actions["position_commands"].append({"port": left_port, "targets": targets})

        # --- 右臂（位置控制） ---
        right_port = servo_ports.get("right_arm")
        if right_port:
            targets = {}
            for i, (_jname, jinfo) in enumerate(servo_ids.get("right_arm", {}).items()):
                if isinstance(jinfo, dict) and i < len(self.right_angles):
                    targets[jinfo["id"]] = float(self.right_angles[i])
            if targets:
                actions["position_commands"].append({"port": right_port, "targets": targets})

        # --- 底盘 + 升降轴 + 身体关节（速度 / 位置控制） ---
        # 这三个部位通常共用同一端口
        base_port = (servo_ports.get("base")
                     or servo_ports.get("lift_axis")
                     or servo_ports.get("neck"))  # Server 将 body_joints 重命名为 neck
        if base_port:
            speed_targets = {}
            position_targets = {}

            # Aider 四轮: whel_Link1~4 → 对应 servo_ids.base 中的键
            wheel_speeds = self.compute_wheel_speeds()
            base_config = servo_ids.get("base", {})
            for wheel_name, wheel_info in base_config.items():
                if isinstance(wheel_info, dict) and wheel_name in wheel_speeds:
                    speed_targets[wheel_info["id"]] = int(wheel_speeds[wheel_name])

            # 升降轴
            lift_config = servo_ids.get("lift_axis", {})
            for _axis_name, axis_info in lift_config.items():
                if isinstance(axis_info, dict):
                    speed_targets[axis_info["id"]] = int(self.lift_velocity)

            # 身体关节 (腰/头/头俯仰) — 作为位置命令发送
            body_joint_config = servo_ids.get("neck", {})  # Server 将 body_joints 重命名为 neck
            body_angles_deg = {
                "head_Link":  float(np.degrees(self.head_yaw)),
                "head_Link2": float(np.degrees(self.head_pitch)),
            }
            for jname, jinfo in body_joint_config.items():
                if isinstance(jinfo, dict) and jname in body_angles_deg:
                    position_targets[jinfo["id"]] = body_angles_deg[jname]

            if speed_targets:
                actions["speed_commands"].append({"port": base_port, "targets": speed_targets})
            if position_targets:
                actions["position_commands"].append({"port": base_port, "targets": position_targets})

        return actions

    def build_action(self, vr_raw_data: dict,
                     base_vel: dict = None,
                     lift_vel: float = None) -> dict:
        """[兼容] 构建完整的机器人动作字典（已被 build_hardware_actions 替代）。"""
        left_angles = self.left_angles.copy()
        right_angles = self.right_angles.copy()

        left_trigger = vr_raw_data.get("leftController", {}).get("trigger")
        right_trigger = vr_raw_data.get("rightController", {}).get("trigger")
        if left_trigger is not None and len(left_angles) > GRIPPER_INDEX:
            left_angles[GRIPPER_INDEX] = -left_trigger * 90.0
        if right_trigger is not None and len(right_angles) > GRIPPER_INDEX:
            right_angles[GRIPPER_INDEX] = -right_trigger * 90.0

        bv = base_vel or {}
        wheel_speeds = self.compute_wheel_speeds(
            bv.get("x", self.base_vx),
            bv.get("y", self.base_vy),
            bv.get("theta", self.base_vtheta),
        )
        lv = lift_vel if lift_vel is not None else self.lift_velocity

        return {
            "left_arm_angles": left_angles,
            "right_arm_angles": right_angles,
            "base.whel_Link1.vel": wheel_speeds["whel_Link1"],
            "base.whel_Link2.vel": wheel_speeds["whel_Link2"],
            "base.whel_Link3.vel": wheel_speeds["whel_Link3"],
            "base.whel_Link4.vel": wheel_speeds["whel_Link4"],
            "lift.axis1.vel": int(lv),
            "waist_Link": float(np.degrees(self.waist_angle)),
            "head_Link": float(np.degrees(self.head_yaw)),
            "head_Link2": float(np.degrees(self.head_pitch)),
        }

    # ======================== VR 摇杆 → 底盘/升降轴 ========================

    def update_from_vr_joystick(self, vr_data: dict) -> None:
        left_joy = vr_data.get("leftController", {}).get("joystick", {"x": 0, "y": 0})
        right_joy = vr_data.get("rightController", {}).get("joystick", {"x": 0, "y": 0})

        lx, ly = left_joy.get("x", 0), left_joy.get("y", 0)
        rx, ry = right_joy.get("x", 0), right_joy.get("y", 0)

        def deadzone(val, threshold=0.1):
            return val if abs(val) > threshold else 0.0

        lx, ly = deadzone(lx), deadzone(ly)
        rx, ry = deadzone(rx), deadzone(ry)

        MAX_LIN_SPEED = 0.1
        MAX_ANG_SPEED = 1.0

        self.base_vx = -ly * MAX_LIN_SPEED
        self.base_vy = -lx * MAX_LIN_SPEED
        self.base_vtheta = -rx * MAX_ANG_SPEED

        if abs(ry) > 0.1:
            self.lift_velocity = int(ry * MAX_LIFT_SPEED_RAW)
        else:
            self.lift_velocity = 0

    # ======================== 仿真可视化 ========================

    def update_aider_arm_pose_sim(self, visualizer, arm: str,
                                   joint_angles_deg: np.ndarray) -> None:
        """将 8-DOF 关节角度写入 PyBullet 的 Aider URDF 仿真。"""
        if not visualizer or visualizer.aider_id is None:
            return

        import pybullet as p
        cid = visualizer.physics_client
        joint_angles_rad = np.deg2rad(joint_angles_deg)

        num_joints = p.getNumJoints(visualizer.aider_id, physicsClientId=cid)
        prefix = f"{arm}_arm"

        for i in range(num_joints):
            info = p.getJointInfo(visualizer.aider_id, i, physicsClientId=cid)
            joint_name = info[1].decode("UTF-8")
            if joint_name.startswith(prefix):
                try:
                    joint_num = int(joint_name[len(prefix):]) - 1  # 0-indexed
                    if 0 <= joint_num < len(joint_angles_rad):
                        p.resetJointState(visualizer.aider_id, i,
                                          joint_angles_rad[joint_num],
                                          physicsClientId=cid)
                except ValueError:
                    pass

    def update_body_joints_sim(self, visualizer) -> None:
        """更新身体关节（腰/头/升降）在仿真中的角度。"""
        if not visualizer or visualizer.aider_id is None:
            return

        import pybullet as p
        cid = visualizer.physics_client

        body_targets = {
            "waist_Link": self.waist_angle,
            "head_Link": self.head_yaw,
            "head_Link2": self.head_pitch,
        }

        num_joints = p.getNumJoints(visualizer.aider_id, physicsClientId=cid)
        for i in range(num_joints):
            info = p.getJointInfo(visualizer.aider_id, i, physicsClientId=cid)
            joint_name = info[1].decode("UTF-8")
            if joint_name in body_targets:
                p.resetJointState(visualizer.aider_id, i,
                                  body_targets[joint_name],
                                  physicsClientId=cid)

    def update_lift_sim(self, visualizer, height_m: float) -> None:
        """更新升降轴在仿真中的高度。"""
        if not visualizer or visualizer.aider_id is None:
            return

        import pybullet as p
        cid = visualizer.physics_client

        num_joints = p.getNumJoints(visualizer.aider_id, physicsClientId=cid)
        for i in range(num_joints):
            info = p.getJointInfo(visualizer.aider_id, i, physicsClientId=cid)
            joint_name = info[1].decode("UTF-8")
            if joint_name == "lift_Link":
                p.resetJointState(visualizer.aider_id, i, height_m,
                                  physicsClientId=cid)
                break

    def update_base_sim(self, visualizer, dt: float) -> None:
        """更新底盘在仿真中的位置。"""
        if not visualizer or visualizer.aider_id is None:
            return

        import pybullet as p
        cid = visualizer.physics_client

        pos, orn = p.getBasePositionAndOrientation(visualizer.aider_id,
                                                    physicsClientId=cid)
        euler = p.getEulerFromQuaternion(orn)
        new_yaw = euler[2] + np.radians(self.base_vtheta * ROTATION_GAIN) * dt
        new_orn = p.getQuaternionFromEuler([euler[0], euler[1], new_yaw])

        cos_yaw = math.cos(new_yaw)
        sin_yaw = math.sin(new_yaw)
        delta_x = (self.base_vy * cos_yaw - self.base_vx * sin_yaw) * dt
        delta_y = (self.base_vx * cos_yaw + self.base_vy * sin_yaw) * dt

        p.resetBasePositionAndOrientation(
            visualizer.aider_id,
            [pos[0] + delta_x, pos[1] + delta_y, pos[2]],
            new_orn,
            physicsClientId=cid,
        )

    def update_visualization(self, visualizer, state: dict) -> None:
        """统一更新所有仿真可视化。"""
        if not visualizer or not visualizer.is_connected:
            return

        dt = state.get("dt", 0.02)
        vr_raw_data = state.get("vr_raw_data", {})

        # ---- 诊断 ----
        if not hasattr(self, '_vis_diag_printed'):
            self._vis_diag_printed = True
            print(f"[DIAG] Aider update_visualization 首次进入 | "
                  f"aider_id={visualizer.aider_id} | "
                  f"左臂={self.left_angles.round(1)} | "
                  f"右臂={self.right_angles.round(1)} | "
                  f"腰={math.degrees(self.waist_angle):.1f}° | "
                  f"base=(x={self.base_vx:.3f} y={self.base_vy:.3f} t={self.base_vtheta:.3f})")

        # 1. 升降轴
        new_height = self.step_lift_height(dt)
        self.update_lift_sim(visualizer, new_height)

        # 2. 底盘 + 轮子旋转
        self.update_base_sim(visualizer, dt)
        wheel_omegas = self._compute_wheel_omegas()
        for wname, w_radps in zip(WHEEL_NAMES, wheel_omegas):
            visualizer.update_wheel_rotation(wname, w_radps, dt)

        # 3. 双臂姿态
        for arm in ["left", "right"]:
            angles = self._get_angles(arm)
            self.update_aider_arm_pose_sim(visualizer, arm, angles)

        # 4. 身体关节
        self.update_body_joints_sim(visualizer)

    # ======================== 属性 ========================

    @property
    def arm_names(self) -> list:
        return ["left", "right"]
