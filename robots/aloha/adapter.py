"""
Aloha 机器人完整适配器。

统一封装该机器人所需的全部计算逻辑:
  - SO100 IK/FK 解算（双机械臂）
  - SO100 → Aloha 关节角度映射
  - 麦克纳姆轮运动学（底盘速度 → 三轮原始速度）
  - 升降轴高度积分
  - PyBullet 仿真可视化更新
  - 动作字典构建（供真机发送和仿真使用）
"""

import numpy as np
import math
from typing import Optional, Dict

from core.kinematic.pybullet.fk_ik import ForwardKinematics, IKSolver
from config.settings import (
    TelegripConfig, NUM_JOINTS, NUM_IK_JOINTS,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE, GRIPPER_INDEX,
    WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, JOINT_NAMES,
    URDF_TO_INTERNAL_NAME_MAP,
)

# ======================== 麦克纳姆轮常量 ========================
WHEEL_RADIUS: float = 0.05
BASE_RADIUS: float = 0.125
MAX_RAW_SPEED: int = 3000
ROTATION_GAIN: float = 100.0     # 底盘旋转速度增益
MAX_LIFT_SPEED_MPS: float = 0.1  # 升降轴最大速度 (m/s)
MAX_LIFT_SPEED_RAW: int = 1500   # VR 升降轴最大原始速度值


def _degps_to_raw(degps: float) -> int:
    """角速度 (deg/s) → Feetech 原始寄存器值 (-32767~+32767)。"""
    steps_per_deg = 4096.0 / 360.0
    mag = int(round(abs(degps) * steps_per_deg))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


class AlohaAdapter:
    """Aloha 机器人完整控制适配器。

    职责:
      - SO100 双臂 IK/FK 解算
      - 双臂关节角度管理（含限位、夹爪映射）
      - 底盘麦克纳姆轮运动学
      - 升降轴速度 → 高度积分
      - SO100 → Aloha 关节映射（仿真用）
      - 仿真可视化统一更新
      - 构建完整机器人动作字典

    使用方式:
        adapter = AlohaAdapter()
        adapter.setup(visualizer, config, robot_ids, joint_indices, end_effector_indices, joint_limits)
        ...
        ik = adapter.solve_ik("left", target_pos, current_angles)
        adapter.update_arm_angles("left", ik, wrist_flex, wrist_roll, gripper)
        action = adapter.build_action(vr_raw_data, base_vel, lift_vel)
        adapter.update_visualization(visualizer, dt)
    """

    def __init__(self):
        # ---- SO100 运动学 ----
        self.fk_solvers: Dict[str, ForwardKinematics] = {}
        self.ik_solvers: Dict[str, IKSolver] = {}

        # ---- 关节状态 ----
        self.left_angles = np.zeros(NUM_JOINTS)
        self.right_angles = np.zeros(NUM_JOINTS)
        self.joint_limits_lower = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_upper = np.full(NUM_JOINTS, 180.0)

        # ---- 底盘速度 ----
        self.base_vx: float = 0.0
        self.base_vy: float = 0.0
        self.base_vtheta: float = 0.0

        # ---- 升降轴 ----
        self.lift_velocity: float = 0.0     # 原始速度 (-1000 ~ 1000)
        self.lift_height_mm: float = 0.0    # 当前高度 (毫米)

        # ---- 状态标记 ----
        self.is_setup: bool = False
        self.config: Optional[TelegripConfig] = None
        self.visualizer = None

        # ---- 外部引用（由 setup 注入） ----
        self._physics_client = None
        self._robot_ids: Dict[str, int] = {}
        self._joint_indices: Dict[str, list] = {}
        self._end_effector_indices: Dict[str, int] = {}

    # ======================== 初始化 ========================

    async def setup(self, visualizer, config: TelegripConfig) -> None:
        """初始化适配器。

        在 PyBullet visualizer.setup() 完成后调用，
        此时 robot_ids / joint_indices / joint_limits 已可用。
        """
        self.config = config
        self.visualizer = visualizer

        if visualizer and visualizer.is_connected:
            self._physics_client = visualizer.physics_client
            self._robot_ids = visualizer.robot_ids
            self._joint_indices = visualizer.joint_indices
            self._end_effector_indices = visualizer.end_effector_link_indices

            jmin, jmax = visualizer.get_joint_limits
            self.joint_limits_lower = jmin.copy()
            self.joint_limits_upper = jmax.copy()

            # 创建 FK/IK 解算器
            for arm in ["left", "right"]:
                self.fk_solvers[arm] = ForwardKinematics(
                    self._physics_client,
                    self._robot_ids[arm],
                    self._joint_indices[arm],
                    self._end_effector_indices[arm],
                )
                self.ik_solvers[arm] = IKSolver(
                    self._physics_client,
                    self._robot_ids[arm],
                    self._joint_indices[arm],
                    self._end_effector_indices[arm],
                    jmin, jmax,
                    arm_name=arm,
                )

        self.is_setup = True
        print("[AlohaAdapter] 适配器初始化完成 (IK/FK + 轮子 + Aloha 映射)")

    # ======================== SO100 IK / FK ========================

    def compute_fk(self, arm: str, angles_deg: np.ndarray) -> np.ndarray:
        """正运动学: 关节角度 → 末端位置。"""
        solver = self.fk_solvers.get(arm)
        if solver:
            pos, _ = solver.compute(angles_deg)
            return pos
        return np.array([0.2, 0.0, 0.15])

    def solve_ik(self, arm: str, target_position: np.ndarray,
                 current_angles: Optional[np.ndarray] = None) -> np.ndarray:
        """逆运动学: 末端位置 → 前 NUM_IK_JOINTS 个关节角度。"""
        if current_angles is None:
            current_angles = self._get_angles(arm)

        solver = self.ik_solvers.get(arm)
        if solver:
            return solver.solve(target_position, None, current_angles)
        return current_angles[:NUM_IK_JOINTS]

    # ======================== 关节管理 ========================

    def _get_angles(self, arm: str) -> np.ndarray:
        """获取指定臂的当前关节角度。"""
        if arm == "left":
            return self.left_angles.copy()
        return self.right_angles.copy()

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray,
                          wrist_flex: float, wrist_roll: float,
                          gripper: float = 0.0, wrist_yaw: float = 0.0) -> np.ndarray:
        """更新指定臂的关节角度（含限位钳制）。

        Returns:
            钳制后的关节角度
        """
        angles = self._get_angles(arm)

        # 前 3 关节来自 IK
        angles[:NUM_IK_JOINTS] = ik_angles
        # 腕部直接设置
        angles[WRIST_FLEX_INDEX] = wrist_flex
        angles[WRIST_ROLL_INDEX] = wrist_roll
        # 夹爪限位
        angles[GRIPPER_INDEX] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)

        # 全局限位钳制（保留已处理的夹爪值）
        clamped = np.clip(angles, self.joint_limits_lower, self.joint_limits_upper)
        clamped[GRIPPER_INDEX] = angles[GRIPPER_INDEX]

        if arm == "left":
            self.left_angles = clamped
        else:
            self.right_angles = clamped

        return clamped

    def apply_gripper_from_trigger(self, arm: str, trigger_value: float) -> None:
        """根据 VR 扳机值 (0~1) 设置夹爪角度 (0°~-90°)。"""
        gripper_angle = -trigger_value * 90.0
        if arm == "left":
            self.left_angles[GRIPPER_INDEX] = gripper_angle
        else:
            self.right_angles[GRIPPER_INDEX] = gripper_angle

    # ======================== 底盘轮子运动学 ========================

    def set_base_velocity(self, vx: float, vy: float, vtheta: float) -> None:
        """设置底盘目标速度（车身坐标系）。

        Args:
            vx: 前后速度 (m/s)，正=前进
            vy: 左右速度 (m/s)，正=左移
            vtheta: 旋转速度 (底盘原始值，内部会应用增益)
        """
        self.base_vx = vx
        self.base_vy = vy
        self.base_vtheta = vtheta

    def compute_wheel_speeds(self, vx: float = None, vy: float = None,
                             vtheta: float = None) -> Dict[str, int]:
        """底盘速度 → 三轮原始速度指令。

        Returns:
            {"base_left_wheel": raw, "base_back_wheel": raw, "base_right_wheel": raw}
        """
        x = vx if vx is not None else self.base_vx
        y = vy if vy is not None else self.base_vy
        t = vtheta if vtheta is not None else self.base_vtheta

        theta_scaled = t * ROTATION_GAIN
        theta_rad = np.radians(theta_scaled)
        vel = np.array([-x, -y, theta_rad])

        # 三轮安装角度 (240°, 0°, 120°) 偏移 -90°
        angles_rad = np.radians(np.array([240, 0, 120]) - 90)
        M = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in angles_rad])

        v_lin = M.dot(vel)
        w_rad = v_lin / WHEEL_RADIUS
        w_degps = np.degrees(w_rad)

        # 限幅
        steps_per_deg = 4096.0 / 360.0
        raw_abs = np.abs(w_degps) * steps_per_deg
        peak = float(np.max(raw_abs)) if raw_abs.size else 0.0
        if peak > MAX_RAW_SPEED and peak > 1e-6:
            w_degps = w_degps * (MAX_RAW_SPEED / peak)

        raw_vals = [_degps_to_raw(v) for v in w_degps]
        return {
            "base_left_wheel": raw_vals[0],
            "base_back_wheel": raw_vals[1],
            "base_right_wheel": raw_vals[2],
        }

    # ======================== 升降轴 ========================

    def set_lift_velocity(self, velocity: float) -> None:
        """设置升降轴原始速度 (-1000~+1000)。"""
        self.lift_velocity = velocity

    def step_lift_height(self, dt: float) -> float:
        """根据当前速度积分一步高度。

        Returns:
            新的高度 (米)
        """
        speed_mps = (self.lift_velocity / 1000.0) * MAX_LIFT_SPEED_MPS
        delta_m = speed_mps * dt
        old_m = self.lift_height_mm / 1000.0
        new_m = old_m + delta_m
        self.lift_height_mm = new_m * 1000.0
        return new_m

    # ======================== 硬件命令构建 ========================

    def build_hardware_actions(self, servo_ids: dict) -> dict:
        """根据当前状态和舵机配置，构建结构化的硬件命令。

        本方法完全封装 Aloha 特有的底盘/轮子/升降轴映射逻辑，
        调用方（robot_interface）只需无脑派发返回的命令。

        Returns:
            {
                "position_commands": [{"port": str, "targets": {servo_id: angle}}, ...],
                "speed_commands":    [{"port": str, "targets": {servo_id: speed}}, ...],
            }
        """
        actions = {"position_commands": [], "speed_commands": []}

        # --- 左臂（位置控制） ---
        left_bus = servo_ids.get("left_bus", {})
        left_port = left_bus.get("port")
        if left_port:
            targets = {}
            for i, (_jname, jinfo) in enumerate(left_bus.get("left_arm", {}).items()):
                if i < len(self.left_angles):
                    targets[jinfo["id"]] = float(self.left_angles[i])
            if targets:
                actions["position_commands"].append({"port": left_port, "targets": targets})

        # --- 右臂（位置控制） ---
        right_bus = servo_ids.get("right_bus", {})
        right_port = right_bus.get("port")
        if right_port:
            targets = {}
            for i, (_jname, jinfo) in enumerate(right_bus.get("right_arm", {}).items()):
                if i < len(self.right_angles):
                    targets[jinfo["id"]] = float(self.right_angles[i])
            if targets:
                actions["position_commands"].append({"port": right_port, "targets": targets})

        # --- 底盘 + 升降轴（速度控制） ---
        base_bus = servo_ids.get("base_lift_bus", {})
        base_port = base_bus.get("port")
        if base_port:
            speed_targets = {}
            # Aloha 三轮: compute_wheel_speeds 返回 base_left_wheel / base_back_wheel / base_right_wheel
            wheel_speeds = self.compute_wheel_speeds()
            base_config = base_bus.get("base", {})
            # 适配器轮名 → servo_ids 键名的映射
            wheel_key_map = {
                "base_left_wheel": "left_wheel",
                "base_back_wheel": "front_wheel",
                "base_right_wheel": "right_wheel",
            }
            for adapter_name, speed_val in wheel_speeds.items():
                config_key = wheel_key_map.get(adapter_name, adapter_name)
                wheel_info = base_config.get(config_key)
                if wheel_info:
                    speed_targets[wheel_info["id"]] = int(speed_val)

            # 升降轴
            lift_config = base_bus.get("lift_axis", {})
            for _axis_name, axis_info in lift_config.items():
                speed_targets[axis_info["id"]] = int(self.lift_velocity)

            if speed_targets:
                actions["speed_commands"].append({"port": base_port, "targets": speed_targets})

        return actions

    def build_action(self, vr_raw_data: dict,
                     base_vel: dict = None,
                     lift_vel: float = None) -> dict:
        """[兼容] 构建完整的机器人动作字典（已被 build_hardware_actions 替代）。"""
        # 1. 双臂角度（应用 VR 扳机 → 夹爪映射）
        left_angles = self.left_angles.copy()
        right_angles = self.right_angles.copy()

        left_trigger = vr_raw_data.get("leftController", {}).get("trigger")
        right_trigger = vr_raw_data.get("rightController", {}).get("trigger")

        if left_trigger is not None and len(left_angles) > GRIPPER_INDEX:
            left_angles[GRIPPER_INDEX] = -left_trigger * 90.0
        if right_trigger is not None and len(right_angles) > GRIPPER_INDEX:
            right_angles[GRIPPER_INDEX] = -right_trigger * 90.0

        # 2. 底盘轮子速度
        bv = base_vel or {}
        wheel_speeds = self.compute_wheel_speeds(
            bv.get("x", self.base_vx),
            bv.get("y", self.base_vy),
            bv.get("theta", self.base_vtheta),
        )

        # 3. 升降轴速度
        lv = lift_vel if lift_vel is not None else self.lift_velocity

        return {
            "left_arm_angles": left_angles,
            "right_arm_angles": right_angles,
            "base.front_wheel.vel": wheel_speeds["base_back_wheel"],
            "base.left_wheel.vel": wheel_speeds["base_left_wheel"],
            "base.right_wheel.vel": wheel_speeds["base_right_wheel"],
            "lift.axis1.vel": int(lv),
        }

    # ======================== VR 摇杆 → 底盘/升降轴 ========================

    def update_from_vr_joystick(self, vr_data: dict) -> None:
        """根据 VR 摇杆数据更新底盘速度和升降轴速度。

        左摇杆: 前进/后退/左右平移
        右摇杆 X: 旋转
        右摇杆 Y: 升降轴
        """
        left_joy = vr_data.get("leftController", {}).get("joystick", {"x": 0, "y": 0})
        right_joy = vr_data.get("rightController", {}).get("joystick", {"x": 0, "y": 0})

        lx, ly = left_joy.get("x", 0), left_joy.get("y", 0)
        rx, ry = right_joy.get("x", 0), right_joy.get("y", 0)

        # 死区
        def deadzone(val, threshold=0.1):
            return val if abs(val) > threshold else 0.0

        lx, ly = deadzone(lx), deadzone(ly)
        rx, ry = deadzone(rx), deadzone(ry)

        MAX_LIN_SPEED = 0.1
        MAX_ANG_SPEED = 1.0

        self.base_vx = -ly * MAX_LIN_SPEED   # 前推=前进
        self.base_vy = -lx * MAX_LIN_SPEED   # 左推=左移
        self.base_vtheta = -rx * MAX_ANG_SPEED  # 左推=左转

        # 升降轴: 右摇杆 Y 控制
        if abs(ry) > 0.1:
            self.lift_velocity = int(ry * MAX_LIFT_SPEED_RAW)
        else:
            self.lift_velocity = 0

    # ======================== 仿真可视化 ========================

    def _map_so100_to_aloha(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """SO100 关节角度 → Aloha 关节角度（加偏移对齐初始姿态）。"""
        adjusted = joint_angles_deg.copy()
        if len(adjusted) >= 3:
            adjusted[1] += 90.0   # shoulder_lift +90°
            adjusted[2] -= 90.0   # elbow_flex -90°
        return adjusted

    def update_aloha_arm_pose_sim(self, visualizer, arm: str,
                                   joint_angles_deg: np.ndarray) -> None:
        """将 SO100 IK 结果映射到 Aloha 双臂的仿真姿态。"""
        if not visualizer or visualizer.aloha_id is None:
            return

        import pybullet as p

        adjusted = self._map_so100_to_aloha(joint_angles_deg)
        joint_angles_rad = np.deg2rad(adjusted)
        cid = visualizer.physics_client

        num_joints = p.getNumJoints(visualizer.aloha_id, physicsClientId=cid)
        for i in range(num_joints):
            info = p.getJointInfo(visualizer.aloha_id, i, physicsClientId=cid)
            joint_name = info[1].decode("UTF-8")
            prefix = f"{arm}_joint"
            if joint_name.startswith(prefix):
                joint_num = int(joint_name[len(prefix):]) - 1
                if 0 <= joint_num < 6:
                    p.resetJointState(visualizer.aloha_id, i, joint_angles_rad[joint_num], physicsClientId=cid)

    def update_aloha_base_sim(self, visualizer, dt: float) -> None:
        """更新 Aloha 底盘在仿真中的位置。"""
        if not visualizer or visualizer.aloha_id is None:
            return

        import pybullet as p

        cid = visualizer.physics_client
        pos, orn = p.getBasePositionAndOrientation(visualizer.aloha_id, physicsClientId=cid)
        euler = p.getEulerFromQuaternion(orn)
        new_yaw = euler[2] + np.radians(self.base_vtheta * ROTATION_GAIN) * dt
        new_orn = p.getQuaternionFromEuler([euler[0], euler[1], new_yaw])

        cos_yaw = math.cos(new_yaw)
        sin_yaw = math.sin(new_yaw)
        delta_x = (self.base_vy * cos_yaw - self.base_vx * sin_yaw) * dt
        delta_y = (self.base_vx * cos_yaw + self.base_vy * sin_yaw) * dt

        p.resetBasePositionAndOrientation(
            visualizer.aloha_id,
            [pos[0] + delta_x, pos[1] + delta_y, pos[2]],
            new_orn,
            physicsClientId=cid,
        )

    def update_aloha_lift_sim(self, visualizer, height_m: float) -> None:
        """更新 Aloha 升降轴在仿真中的高度。"""
        if not visualizer or visualizer.aloha_id is None:
            return

        import pybullet as p

        cid = visualizer.physics_client
        URDF_HEIGHT = 0.45
        joint_value = height_m - URDF_HEIGHT

        num_joints = p.getNumJoints(visualizer.aloha_id, physicsClientId=cid)
        for i in range(num_joints):
            info = p.getJointInfo(visualizer.aloha_id, i, physicsClientId=cid)
            joint_name = info[1].decode("UTF-8")
            if joint_name == "vertical_move":
                p.resetJointState(visualizer.aloha_id, i, joint_value, physicsClientId=cid)
                break

    def update_visualization(self, visualizer, state: dict) -> None:
        """统一更新所有仿真可视化。

        state 应包含:
          - dt: 时间步长
          - vr_raw_data: VR 原始数据
        """
        if not visualizer or not visualizer.is_connected:
            return

        dt = state.get("dt", 0.02)
        vr_raw_data = state.get("vr_raw_data", {})

        # ---- 诊断: 首次进入打印关键状态 ----
        if not hasattr(self, '_vis_diag_printed'):
            self._vis_diag_printed = True
            print(f"[DIAG] update_visualization 首次进入 | "
                  f"aloha_enabled={self.config.aloha_enabled if self.config else 'N/A'} | "
                  f"aloha_id={visualizer.aloha_id} | "
                  f"左臂={self.left_angles.round(1)} | "
                  f"右臂={self.right_angles.round(1)} | "
                  f"base=(x={self.base_vx:.3f} y={self.base_vy:.3f} t={self.base_vtheta:.3f})")

        # 1. 升降轴积分 + 仿真更新
        if self.config and self.config.aloha_enabled:
            new_height = self.step_lift_height(dt)
            self.update_aloha_lift_sim(visualizer, new_height)

        # 2. 底盘仿真位置更新
        if self.config and self.config.aloha_enabled:
            self.update_aloha_base_sim(visualizer, dt)

        # 3. SO100 双臂 + Aloha 双臂映射
        for arm in ["left", "right"]:
            angles = self._get_angles(arm)
            # SO100 姿态
            visualizer.update_robot_pose(angles, arm)
            # SO100 → Aloha 映射
            if self.config and self.config.aloha_enabled and visualizer.aloha_id is not None:
                self.update_aloha_arm_pose_sim(visualizer, arm, angles)

    # ======================== 属性 ========================

    @property
    def arm_names(self) -> list:
        return ["left", "right"]
