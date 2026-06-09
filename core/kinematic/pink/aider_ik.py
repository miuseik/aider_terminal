"""
Aider 机器人 Pink IK 求解器。
使用 Pink (Pinocchio) 库进行基于 QP 的数值 IK 求解，
替代原有的纯 Python DLS 实现。
"""

import os
import re
import time
import xml.etree.ElementTree as ET
# import logging
import numpy as np
from typing import Dict, List, Optional, Tuple

import pinocchio as pin

# 抑制 Pinocchio 内部限位警告（已通过钳制机制大幅减少，剩余不干扰求解）
# logging.getLogger("root").setLevel(logging.ERROR)

import pink
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
import qpsolvers

# 项目根目录定位
_PROJ_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# 加载机器人 settings（用文件路径直读，绕过 robots/aider/__init__.py 避免循环导入）
import importlib.util as _iu
import importlib.machinery as _im
_settings_path = os.path.join(_PROJ_ROOT, "robots", "aider", "settings.py")
_settings_loader = _im.SourceFileLoader("_robot_settings", _settings_path)
_settings_spec = _iu.spec_from_loader("_robot_settings", _settings_loader)
_robot_settings = _iu.module_from_spec(_settings_spec)
_settings_loader.exec_module(_robot_settings)
JOINT_LIMIT_OVERRIDES = _robot_settings.JOINT_LIMIT_OVERRIDES
POSTURE = _robot_settings.POSTURE


class AiderPinkSolver:
    """使用 Pink 库的 Aider 机器人 IK 求解器。

    加载 Aider 完整 URDF，支持单臂 IK 求解（另一臂和身体关节保持当前值）。
    """

    # 末端执行器 frame 名称（在 URDF 中定义）
    ARM_LINKS = {
        "left": "left_arm8",
        "right": "right_arm8",
    }

    def __init__(self, urdf_path: Optional[str] = None):
        if urdf_path is None:
            urdf_path = os.path.join(
                _PROJ_ROOT, "URDF", "aider", "urdf", "aider_pro.SLDASM.urdf")
        print(f"[AiderPinkSolver] 加载 URDF: {urdf_path}")

        import tempfile, atexit
        urdf_dir = os.path.dirname(os.path.dirname(urdf_path))  # URDF/aider/
        mesh_dir = os.path.join(urdf_dir, "meshes")             # URDF/aider/meshes/

        # 重写 URDF 中的 package:// 路径为实际文件路径
        with open(urdf_path, 'r', encoding='utf-8') as f:
            urdf_content = f.read()
        urdf_content = re.sub(
            r'filename="package://[^"]*/([^"/]+)"',
            lambda m: f'filename="{os.path.join(mesh_dir, m.group(1))}"',
            urdf_content,
        )

        # 将 settings 的关节限位同步写入 URDF XML，Pinocchio 加载时原生生效
        urdf_content = self._patch_urdf_limits(urdf_content)

        # 写入临时文件
        tmp_urdf = tempfile.NamedTemporaryFile(
            mode='w', suffix='.urdf', delete=False, encoding='utf-8')
        tmp_urdf.write(urdf_content)
        tmp_urdf.close()
        self._tmp_urdf_path = tmp_urdf.name
        atexit.register(lambda: os.unlink(self._tmp_urdf_path))

        self.robot = pin.RobotWrapper.BuildFromURDF(
            self._tmp_urdf_path,
            package_dirs=[],
            root_joint=pin.JointModelFreeFlyer(),
            verbose=False,
        )

        # 重命名重复的 frame，使 getFrameId 不冲突
        # URDF 中每个 link body frame 和 joint frame 同名，
        # 第二个 frame 改成带有 "_body" 后缀
        frame_map: Dict[str, int] = {}
        for i in range(self.robot.model.nframes):
            name = self.robot.model.frames[i].name
            if name in frame_map:
                from pinocchio import Frame
                old = self.robot.model.frames[i]
                self.robot.model.frames[i] = Frame(
                    name + "_body", old.parent, old.previousFrame,
                    old.placement, old.type)
            else:
                frame_map[name] = i

        # ---- 发现关节 ----
        self.joint_names: List[str] = []
        self.arm_joints: Dict[str, List[str]] = {"left": [], "right": []}
        self.body_joints: List[str] = ["lift_Link", "waist_Link", "head_Link", "head_Link2"]

        for i in range(1, self.robot.model.njoints):
            name = self.robot.model.names[i]
            self.joint_names.append(name)
            if name.startswith("left_arm"):
                self.arm_joints["left"].append(name)
            elif name.startswith("right_arm"):
                self.arm_joints["right"].append(name)

        print(f"  [AiderPinkSolver] 发现 {len(self.joint_names)} 个关节: "
              f"左臂 {len(self.arm_joints['left'])} + "
              f"右臂 {len(self.arm_joints['right'])} + 身体 {len(self.body_joints)}")

        # ---- 末端 frame ID（取第一个匹配的） ----
        self._end_frame_ids: Dict[str, int] = {}
        for arm, link in self.ARM_LINKS.items():
            for i in range(self.robot.model.nframes):
                if self.robot.model.frames[i].name == link:
                    self._end_frame_ids[arm] = i
                    print(f"  [AiderPinkSolver] {arm} 末端 frame='{link}' ID={i}")
                    break
            else:
                print(f"  ⚠️  找不到 frame '{link}'")

        # ---- 选择 QP 求解器 ----
        self.solver = "quadprog" if "quadprog" in qpsolvers.available_solvers \
            else qpsolvers.available_solvers[0]

        # ---- 初始配置（零位） ----
        self._q0 = self._make_zero_config()

        # ---- 姿态偏好（自然下垂、肘微弯） ----
        self._posture_q = self._make_posture_config()

        # ---- 初始配置（直接设为姿态偏好，启动就在舒适位） ----
        self._current_q = self._make_posture_config()

    def _make_zero_config(self) -> np.ndarray:
        """构建初始配置（free flyer 基座在原点，所有关节在零位）。"""
        q = np.zeros(self.robot.model.nq)
        q[6] = 1.0  # FreeFlyer [x,y,z, qx,qy,qz,qw], qw=1
        return q

    def _make_posture_config(self) -> np.ndarray:
        """构建姿态偏好配置，从 robots/aider/settings.py 的 POSTURE 字典读取。

        关节值单位：度。
        """
        q = self._make_zero_config()
        for name, deg in POSTURE.items():
            qi = self._q_idx(name)
            if qi >= 0:
                q[qi] = np.radians(deg)
        return q

    @staticmethod
    def _patch_urdf_limits(urdf_xml: str) -> str:
        """将 settings 中的关节限位同步写入 URDF XML，Pinocchio 加载时原生生效。

        使用 XML parser 而非 regex，确保每次替换精确针对目标 joint 的 limit 标签。
        """
        root = ET.fromstring(urdf_xml)
        for joint_elem in root.iter("joint"):
            jname = joint_elem.get("name")
            if jname and jname in JOINT_LIMIT_OVERRIDES:
                lo_deg, hi_deg = JOINT_LIMIT_OVERRIDES[jname]
                limit = joint_elem.find("limit")
                if limit is not None:
                    limit.set("lower", f"{np.radians(lo_deg):.8f}")
                    limit.set("upper", f"{np.radians(hi_deg):.8f}")
        return ET.tostring(root, encoding="unicode")

    def get_posture(self, arm: str) -> np.ndarray:
        """返回指定臂的初始姿态角度（度）。"""
        return self._extract_arm_angles(self._posture_q, arm)

    def _q_idx(self, joint_name: str) -> int:
        """返回关节在配置向量 q 中的起始索引。"""
        jid = self.robot.model.getJointId(joint_name)
        return self.robot.model.idx_qs[jid]

    def _update_current_q(self, arm: str, current_angles_deg: np.ndarray,
                          body_state: Optional[dict] = None) -> np.ndarray:
        """更新当前配置 q。

        Args:
            arm: 当前求解的臂 ('left' / 'right')
            current_angles_deg: 该臂的当前角度（8 个，度）
            body_state: 身体关节状态 {lift_m, waist_rad, head_yaw_rad, head_pitch_rad}
        """
        q = self._current_q.copy()

        if body_state:
            m = self.robot.model
            for joint_name, val in body_state.items():
                qi = self._q_idx(joint_name)
                if qi >= 0:
                    lo, hi = m.lowerPositionLimit[qi], m.upperPositionLimit[qi]
                    q[qi] = np.clip(val, lo, hi)

        # 更新当前臂的角度
        for i, jname in enumerate(self.arm_joints[arm]):
            qi = self._q_idx(jname)
            if qi >= 0 and i < len(current_angles_deg):
                q[qi] = np.radians(current_angles_deg[i])

        return q

    def forward_kinematics(self, arm: str, angles_deg: np.ndarray,
                           body_state: Optional[dict] = None) -> Optional[np.ndarray]:
        """正运动学: 关节角度 → 末端位置 (base_link 坐标系, 米)。

        Args:
            arm: 'left' 或 'right'
            angles_deg: 8 个关节角度（度）
            body_state: 身体关节状态 {lift_m, waist_rad, head_yaw_rad, head_pitch_rad}

        Returns:
            [x, y, z] 位置，失败返回 None
        """
        if arm not in self._end_frame_ids:
            return None
        q = self._update_current_q(arm, angles_deg, body_state)
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        frame_id = self._end_frame_ids[arm]
        pos = self.robot.data.oMf[frame_id].translation
        return np.array(pos[:3])

    def _extract_arm_angles(self, q: np.ndarray, arm: str) -> np.ndarray:
        """从完整配置 q 中提取指定臂的 8 个关节角度（度）。"""
        angles = np.zeros(len(self.arm_joints[arm]))
        for i, jname in enumerate(self.arm_joints[arm]):
            qi = self._q_idx(jname)
            if qi >= 0:
                angles[i] = np.degrees(q[qi])
        return angles

    def solve(self, arm: str, target_position: np.ndarray,
              current_angles: np.ndarray,
              body_state: Optional[dict] = None,
              target_orientation: Optional[np.ndarray] = None,
              dt: float = 0.05) -> Optional[np.ndarray]:
        """单臂 IK 求解。

        Args:
            arm: 'left' 或 'right'
            target_position: 目标位置 [x, y, z] (米)
            current_angles: 该臂当前角度（8 个，度）
            body_state: 身体关节状态 {lift_m, waist_rad, head_yaw_rad, head_pitch_rad}
            target_orientation: 目标姿态四元数 [x,y,z,w]（可选）
            dt: 积分时间步长

        Returns:
            8 个关节角度（度），失败返回 None
        """
        if arm not in self._end_frame_ids:
            return None

        # 更新当前配置
        q = self._update_current_q(arm, current_angles, body_state)
        configuration = pink.Configuration(self.robot.model, self.robot.data, q)

        # ---- 创建任务 ----
        tasks = []

        # 1. 末端执行器位置任务
        ee_task = FrameTask(
            self.ARM_LINKS[arm],
            position_cost=1.0,
            orientation_cost=0.5 if target_orientation is not None else 0.0,
            lm_damping=1.0,
        )
        target_se3 = pin.SE3.Identity()
        target_se3.translation = np.asarray(target_position, dtype=float)
        if target_orientation is not None:
            target_se3.rotation = pin.Quaternion(*target_orientation).matrix()
        ee_task.transform_target_to_world = target_se3
        tasks.append(ee_task)

        # 2. 基座固定任务（保持 free flyer 在原点，与 PyBullet 一致）
        base_task = FrameTask(
            "root_joint",
            position_cost=100.0,
            orientation_cost=100.0,
            lm_damping=1.0,
        )
        base_task.transform_target_to_world = pin.SE3.Identity()
        tasks.append(base_task)

        # 3. 姿态偏好任务（未求解关节趋向舒适姿态，参考 OpenArmX）
        # 当前求解臂用其当前角度，另一臂用姿态偏好
        posture_target = q.copy()
        other = "right" if arm == "left" else "left"
        for jname in self.arm_joints[other]:
            qi = self._q_idx(jname)
            if qi >= 0:
                posture_target[qi] = self._posture_q[qi]
        posture_task = PostureTask(cost=0.05)
        posture_task.set_target(posture_target)
        tasks.append(posture_task)

        # ---- 求解 ----
        try:
            velocity = solve_ik(
                configuration, tasks, dt,
                solver=self.solver, safety_break=False)
            configuration.integrate_inplace(velocity, dt)
        except Exception as e:
            print(f"  [PinkIK] {arm} IK 求解失败: {e}")
            return None

        # 钳制关节值到限位内，避免 Pinocchio 内部报 warning
        new_q = configuration.q.copy()
        m = self.robot.model
        for jid in range(2, m.njoints):
            qi = m.idx_qs[jid]
            nq = m.nqs[jid]
            if nq == 1:
                new_q[qi] = np.clip(new_q[qi],
                                     m.lowerPositionLimit[qi],
                                     m.upperPositionLimit[qi])

        # 保存并检查位置误差
        self._current_q = new_q
        # 更新 configuration 的 q，使位置读取准确
        configuration = pink.Configuration(m, self.robot.data, new_q)
        current_pos = configuration.get_transform_frame_to_world(
            self.ARM_LINKS[arm]).translation
        error = np.linalg.norm(current_pos - target_position)
        if error > 1.0:
            return None

        # 提取结果
        return self._extract_arm_angles(new_q, arm)
