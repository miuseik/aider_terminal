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

_PINK_AVAILABLE = False
_pink_import_error = None

try:
    import pinocchio as pin
    import pink
    from pink import solve_ik
    from pink.tasks import FrameTask, PostureTask
    import qpsolvers
    _PINK_AVAILABLE = True
except ImportError as e:
    _pink_import_error = str(e)
    pin = None
    pink = None
    solve_ik = None
    FrameTask = None
    PostureTask = None
    qpsolvers = None

# 包目录定位（realpath 跟随 symlink，兼容符号链接部署）
# src/core/kinematic/pink/ → 上 3 层到 src/ 包目录
_PROJ_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", ".."))

# 正常导入 settings（与 adapter / robot_interface 共享同一模块实例，
# 保证 JOINT_LIMIT_OVERRIDES 原地覆盖后 IK 与钳制层同步生效，同源单一真值）。
# 注：Early SourceFileLoader 加载会创建第二个模块副本，导致 IK 看到的 dict
# 与 apply_servo_limits_from_yaml 修改的不是同一对象（限位始终 (0,0)）。
from src.robots.aider.settings import (
    JOINT_LIMIT_OVERRIDES,
    ARM_JOINT_NAMES_LEFT,
    ARM_JOINT_NAMES_RIGHT,
    get_default_posture,
)


class AiderPinkSolver:
    """使用 Pink 库的 Aider 机器人 IK 求解器。

    加载 Aider 完整 URDF，支持单臂 IK 求解（另一臂和身体关节保持当前值）。
    """

    # 末端执行器 frame 名称（在 URDF 中定义）
    # 使用 TCP (tool center point) 而非 arm8：
    # TCP 是工具尖点，fixed 挂在 arm7 下（不随夹爪旋转）。
    # IK 控制 TCP 的位置+姿态，VR 手柄直接映射到 TCP，旋转以 TCP 为圆心。
    ARM_LINKS = {
        "left": "left_TCP",
        "right": "right_TCP",
    }

    def __init__(self, urdf_path: Optional[str] = None):
        if not _PINK_AVAILABLE:
            raise ImportError(
                f"Pink/Pinocchio 未安装 ({_pink_import_error})。"
                f"请先执行: conda install -c conda-forge pinocchio -y && pip install -e \".[pink]\"")
        if urdf_path is None:
            # _PROJ_ROOT = src/ 包目录，URDF 在项目根（上一级）
            urdf_path = os.path.join(
                os.path.dirname(_PROJ_ROOT), "URDF", "aider", "urdf", "aider_pro.SLDASM.urdf")
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

        print(f"  [AiderPinkSolver] 构建 Pinocchio 模型...")
        t0 = time.time()
        self.robot = pin.RobotWrapper.BuildFromURDF(
            self._tmp_urdf_path,
            package_dirs=[],
            root_joint=pin.JointModelFreeFlyer(),
            verbose=False,
        )
        print(f"  [AiderPinkSolver] 模型构建完成 ({time.time() - t0:.1f}s)")

        # 重命名重复的 frame，使 getFrameId 不冲突
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

        # IK 运动关节白名单 = settings 的 8 个臂关节。
        # 新 URDF 的 arm12 是夹爪右指（无舵机），随 arm8 反向联动，不参与 IK。
        arm_joint_whitelist = set(ARM_JOINT_NAMES_LEFT) | set(ARM_JOINT_NAMES_RIGHT)

        for i in range(1, self.robot.model.njoints):
            name = self.robot.model.names[i]
            self.joint_names.append(name)
            if name in arm_joint_whitelist:
                self.arm_joints["left" if name in ARM_JOINT_NAMES_LEFT else "right"].append(name)

        print(f"  [AiderPinkSolver] 发现 {len(self.joint_names)} 个关节: "
              f"左臂 {len(self.arm_joints['left'])} + "
              f"右臂 {len(self.arm_joints['right'])} + 身体 {len(self.body_joints)}")

        # ---- 末端 frame ID（取第一个匹配的） ----
        self._end_frame_ids: Dict[str, int] = {}
        for arm, link in self.ARM_LINKS.items():
            for i in range(self.robot.model.nframes):
                if self.robot.model.frames[i].name == link:
                    self._end_frame_ids[arm] = i
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

        # ---- 肩部系裁剪模型（每臂独立 8-DOF，基座 = 肩安装座 waist_Link）----
        # 锁定除本臂 8 关节外的全部关节（freeflyer 基座/升降/腰/头/另一臂/轮/夹爪右指），
        # IK 在 8 自由度小模型上求解：升降/腰/头不在模型内 → 与臂零耦合，
        # QP 维度从 27 降到 8，求解更快；也无需每帧腰部补偿 FK。
        # 注意: arm1 关节坐标系随 arm1 旋转，不能当固定基座；
        # 真正的肩基座是肩安装座 waist_Link（只随升降/腰动，与臂关节无关）。
        self._reduced: Dict[str, Tuple[object, object]] = {}
        self._S0: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self._reduced_arm_qidx: Dict[str, List[int]] = {}
        for arm in ("left", "right"):
            arm_set = set(self.arm_joints[arm])
            lock_ids = [j for j in range(1, self.robot.model.njoints)
                        if self.robot.model.names[j] not in arm_set]
            rmodel = pin.buildReducedModel(self.robot.model, lock_ids, self._posture_q)
            rdata = rmodel.createData()
            q0 = pin.neutral(rmodel)
            pin.forwardKinematics(rmodel, rdata, q0)
            pin.updateFramePlacements(rmodel, rdata)
            tf = rdata.oMf[rmodel.getFrameId("waist_Link")]
            # S0: 肩安装座在裁剪模型世界系的固定位姿（常量，预计算一次）
            self._S0[arm] = (tf.translation.copy(), tf.rotation.copy())
            self._reduced_arm_qidx[arm] = [
                rmodel.idx_qs[rmodel.getJointId(n)] for n in self.arm_joints[arm]]
            self._reduced[arm] = (rmodel, rdata)
        print(f"  [AiderPinkSolver] 肩部系裁剪模型就绪: "
              f"左/右各 8-DOF (基座=waist_Link)")

    def _make_zero_config(self) -> np.ndarray:
        """构建初始配置（free flyer 基座在原点，所有关节在零位）。"""
        q = np.zeros(self.robot.model.nq)
        q[6] = 1.0  # FreeFlyer [x,y,z, qx,qy,qz,qw], qw=1
        return q

    def _make_posture_config(self) -> np.ndarray:
        """构建姿态偏好配置，从下拉框预设 POSES[DEFAULT_POSE_NAME] 推导。

        关节值单位：度。
        """
        q = self._make_zero_config()
        for name, deg in get_default_posture().items():
            qi = self._q_idx(name)
            if qi >= 0:
                q[qi] = np.radians(deg)
        return q

    @staticmethod
    def _patch_urdf_limits(urdf_xml: str) -> str:
        """将 settings 中的关节限位同步写入 URDF XML，Pinocchio 加载时原生生效。

        使用 XML parser 而非 regex，确保每次替换精确针对目标 joint 的 limit 标签。
        角度关节（arm/waist/head）单位度→弧度；lift_Link 为 prismatic 米制，值直接用。
        """
        root = ET.fromstring(urdf_xml)
        for joint_elem in root.iter("joint"):
            jname = joint_elem.get("name")
            limit = joint_elem.find("limit")
            if limit is None:
                continue
            if jname and jname in JOINT_LIMIT_OVERRIDES:
                lo, hi = JOINT_LIMIT_OVERRIDES[jname]
                if jname == "lift_Link":
                    # prismatic 关节，限位单位为米，不转弧度
                    limit.set("lower", f"{float(lo):.8f}")
                    limit.set("upper", f"{float(hi):.8f}")
                else:
                    limit.set("lower", f"{np.radians(lo):.8f}")
                    limit.set("upper", f"{np.radians(hi):.8f}")
        return ET.tostring(root, encoding="unicode")

    def get_posture(self, arm: str) -> np.ndarray:
        """返回指定臂的初始姿态角度（度）。"""
        return self._extract_arm_angles(self._posture_q, arm)

    def _q_idx(self, joint_name: str) -> int:
        """返回关节在配置向量 q 中的起始索引。"""
        jid = self.robot.model.getJointId(joint_name)
        return self.robot.model.idx_qs[jid]

    def _apply_gripper_link_coupling(self, q: np.ndarray) -> np.ndarray:
        """夹爪两指联动: arm12（右指）随 arm8（左指）反向转动。

        arm12 无独立舵机，URDF 限位 (-1°, 59°) 与 arm8 (-59°, 1°) 镜像。
        arm12 是 arm7 下的独立分支，不影响 TCP 位姿，仅在 FK/仿真中跟随。
        """
        for arm in ("left", "right"):
            try:
                q[self._q_idx(f"{arm}_arm12")] = -q[self._q_idx(f"{arm}_arm8")]
            except Exception:
                pass
        return q

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

        return self._apply_gripper_link_coupling(q)

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

    def forward_kinematics_se3(self, arm: str, angles_deg: np.ndarray,
                               body_state: Optional[dict] = None):
        """正运动学: 关节角度 → TCP 位姿 (位置 + 旋转矩阵, base_link 坐标系)。

        Returns:
            (pos[3], rot[3x3])，失败返回 (None, None)
        """
        if arm not in self._end_frame_ids:
            return None, None
        q = self._update_current_q(arm, angles_deg, body_state)
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        tf = self.robot.data.oMf[self._end_frame_ids[arm]]
        return tf.translation.copy(), tf.rotation.copy()

    def frame_pose(self, frame_name: str, body_state: Optional[dict] = None):
        """任意 frame 在 base_link 系的位姿 (pos, rot)。

        手臂关节置零（waist_Link 在臂上游，不受臂关节影响），
        仅 body_state (lift/waist/head) 决定其位姿。
        """
        q = self._make_zero_config()
        if body_state:
            m = self.robot.model
            for jname, val in body_state.items():
                qi = self._q_idx(jname)
                if qi >= 0:
                    # 与 _update_current_q 统一钳制，保证腰部补偿与 IK 内部
                    # 看到的肩基座一致（不一致会导致臂追不可达目标）
                    q[qi] = np.clip(val, m.lowerPositionLimit[qi],
                                    m.upperPositionLimit[qi])
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        tf = self.robot.data.oMf[self.robot.model.getFrameId(frame_name)]
        return tf.translation.copy(), tf.rotation.copy()

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
              dt: float = 0.05,
              position_cost: float = 1.0,
              orientation_cost: float = 0.5,
              posture_cost: float = 0.05,
              enable_elbow_avoidance: bool = True,
              lock_wrist: bool = True) -> Optional[np.ndarray]:
        """单臂 IK 求解。

        Args:
            arm: 'left' 或 'right'
            target_position: 目标位置 [x, y, z] (米)
            current_angles: 该臂当前角度（8 个，度）
            body_state: 身体关节状态 {lift_m, waist_rad, head_yaw_rad, head_pitch_rad}
            target_orientation: 目标姿态四元数 [x,y,z,w]（可选）
            dt: 积分时间步长
            position_cost: 位置任务权重（提高可减小位置稳态误差，旋转更锁 TCP 圆心）
            orientation_cost: 姿态任务权重（仅 target_orientation 提供时生效）
            lock_wrist: True 时手腕(arm5/6/7)不进入 IK 求解范围，由直控值决定
                （与 aloha 一致：位置 IK + 手腕直控）。此时 orientation_cost 强制为 0，
                姿态完全由直控手腕角决定，IK 只约束 TCP 位置。

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

        # 锁手腕模式（对齐 aloha）：只约束 TCP 位置，不约束姿态。
        # IK 不再解手腕姿态，arm5/6/7 由直控值决定（见下方手腕锁任务）。
        if lock_wrist:
            target_orientation = None
            orientation_cost = 0.0

        # 1. 末端执行器位置任务
        ee_task = FrameTask(
            self.ARM_LINKS[arm],
            position_cost=position_cost,
            orientation_cost=orientation_cost if target_orientation is not None else 0.0,
            lm_damping=1.0,
        )
        target_se3 = pin.SE3.Identity()
        target_se3.translation = np.asarray(target_position, dtype=float)
        if target_orientation is not None:
            # target_orientation 约定为 [x,y,z,w]（scipy 顺序）。
            # pin.Quaternion(w,x,y,z) 按 Eigen 构造，必须显式重排，否则姿态完全错误。
            qx, qy, qz, qw = target_orientation
            target_se3.rotation = pin.Quaternion(qw, qx, qy, qz).matrix()
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

        # 3. 姿态偏好任务
        posture_target = q.copy()
        other = "right" if arm == "left" else "left"
        for jname in self.arm_joints[other]:
            qi = self._q_idx(jname)
            if qi >= 0:
                posture_target[qi] = self._posture_q[qi]

        posture_task = PostureTask(cost=posture_cost)
        posture_task.set_target(posture_target)
        tasks.append(posture_task)

        # 4. 肘部避碰：直接在肘部 frame 加位置约束，推离身体
        #    Y 方向：远离 Y=0（以 elbow_x 符号为参考区分左右臂）
        #    X 方向：远离 X=0（同样用 elbow_x 符号）
        elbow_frame = f"{arm}_arm4"
        elbow_fid = self.robot.model.getFrameId(elbow_frame)
        if enable_elbow_avoidance and elbow_fid < self.robot.model.nframes:
            elbow_pos = configuration.get_transform_frame_to_world(
                elbow_frame).translation
            elbow_y = elbow_pos[1]
            elbow_x = elbow_pos[0]

            min_clearance = 0.07
            safe_clearance = 0.16

            if abs(elbow_y) < safe_clearance or abs(elbow_x) < safe_clearance:
                danger = np.clip(
                    1.0 - (min(abs(elbow_y), abs(elbow_x)) - min_clearance) /
                    (safe_clearance - min_clearance), 0.0, 1.0)

                elbow_target = elbow_pos.copy()

                # Y 方向：保持当前符号，放大到 safe_clearance 以外
                if abs(elbow_y) < safe_clearance:
                    push_dir_y = 1.0 if elbow_y >= 0 else -1.0
                    elbow_target[1] = push_dir_y * safe_clearance

                # X 方向：elbow_x 的符号天然区分左右臂，往同符号方向推
                if abs(elbow_x) < safe_clearance:
                    push_dir_x = 1.0 if elbow_x >= 0 else -1.0
                    elbow_target[0] = push_dir_x * safe_clearance

                elbow_task = FrameTask(
                    elbow_frame,
                    position_cost=0.05 + danger * 0.80,
                    orientation_cost=0.0,
                    lm_damping=1.0,
                )
                target_se3 = pin.SE3.Identity()
                target_se3.translation = elbow_target
                elbow_task.transform_target_to_world = target_se3
                tasks.append(elbow_task)

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

        # 夹爪两指联动（arm12 = -arm8），保持 _current_q 一致
        new_q = self._apply_gripper_link_coupling(new_q)

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
        angles = self._extract_arm_angles(new_q, arm)

        # 对齐 aloha：手腕(arm5/6/7)不在 IK 求解范围内，用直控值覆盖
        # （等价 aloha 的 update_arm_angles: IK 解出手腕被输入值丢弃）。
        # IK 只决定 arm1-4 位置链，arm5/6/7 严格等于传入的 current_angles。
        if lock_wrist and current_angles is not None and len(current_angles) >= 7:
            angles[4] = current_angles[4]
            angles[5] = current_angles[5]
            angles[6] = current_angles[6]

        return angles

    def solve_local(self, arm: str, target_shoulder: np.ndarray,
                    current_angles: np.ndarray,
                    dt: float = 0.05,
                    position_cost: float = 1.0,
                    posture_cost: float = 0.05,
                    enable_elbow_avoidance: bool = True,
                    lock_wrist: bool = True) -> Optional[np.ndarray]:
        """肩部系 IK：在裁剪的 8-DOF 手臂模型上求解（基座 = 肩安装座 waist_Link）。

        Args:
            arm: 'left' 或 'right'
            target_shoulder: TCP 目标在肩安装座局部系的坐标 [x, y, z] (米)。
                该坐标系与升降/腰/头完全无关——同一组坐标永远解出同一组臂角，
                身体任何部位运动都不会影响手臂。
            current_angles: 该臂当前角度（8 个，度）
            dt: 积分时间步长
            position_cost: 位置任务权重
            posture_cost: 姿态偏好任务权重
            enable_elbow_avoidance: 肘部避碰（肩系坐标下推离躯干中心）
            lock_wrist: True 时手腕(arm5/6/7)不进 IK，由直控值决定

        Returns:
            8 个关节角度（度），失败返回 None
        """
        if arm not in self._reduced:
            return None
        rmodel, rdata = self._reduced[arm]
        qidx = self._reduced_arm_qidx[arm]

        q = pin.neutral(rmodel)
        for i, qi in enumerate(qidx):
            if i < len(current_angles):
                q[qi] = np.radians(current_angles[i])
        configuration = pink.Configuration(rmodel, rdata, q)

        # 目标: 肩部系坐标 → 裁剪模型世界系（常量变换 S0，预计算一次）
        s0_pos, s0_rot = self._S0[arm]
        target_se3 = pin.SE3.Identity()
        target_se3.translation = \
            s0_rot @ np.asarray(target_shoulder, dtype=float) + s0_pos

        tasks = []

        # 1. 末端位置任务（锁手腕模式只约束位置，与 solve() 一致）
        ee_task = FrameTask(
            self.ARM_LINKS[arm],
            position_cost=position_cost,
            orientation_cost=0.0,
            lm_damping=1.0,
        )
        ee_task.transform_target_to_world = target_se3
        tasks.append(ee_task)

        # 2. 姿态偏好任务（弱拉回当前位形，8 维小向量）
        posture_task = PostureTask(cost=posture_cost)
        posture_task.set_target(q)
        tasks.append(posture_task)

        # 3. 肘部避碰（裁剪模型世界系 = 肩安装座系，推离躯干中心语义不变）
        elbow_frame = f"{arm}_arm4"
        if enable_elbow_avoidance:
            elbow_pos = configuration.get_transform_frame_to_world(
                elbow_frame).translation
            elbow_y = elbow_pos[1]
            elbow_x = elbow_pos[0]

            min_clearance = 0.07
            safe_clearance = 0.16

            if abs(elbow_y) < safe_clearance or abs(elbow_x) < safe_clearance:
                danger = np.clip(
                    1.0 - (min(abs(elbow_y), abs(elbow_x)) - min_clearance) /
                    (safe_clearance - min_clearance), 0.0, 1.0)

                elbow_target = elbow_pos.copy()
                if abs(elbow_y) < safe_clearance:
                    push_dir_y = 1.0 if elbow_y >= 0 else -1.0
                    elbow_target[1] = push_dir_y * safe_clearance
                if abs(elbow_x) < safe_clearance:
                    push_dir_x = 1.0 if elbow_x >= 0 else -1.0
                    elbow_target[0] = push_dir_x * safe_clearance

                elbow_task = FrameTask(
                    elbow_frame,
                    position_cost=0.05 + danger * 0.80,
                    orientation_cost=0.0,
                    lm_damping=1.0,
                )
                elbow_se3 = pin.SE3.Identity()
                elbow_se3.translation = elbow_target
                elbow_task.transform_target_to_world = elbow_se3
                tasks.append(elbow_task)

        # ---- 求解（8-DOF QP，比全身 27-DOF 快） ----
        try:
            velocity = solve_ik(
                configuration, tasks, dt,
                solver=self.solver, safety_break=False)
            configuration.integrate_inplace(velocity, dt)
        except Exception as e:
            print(f"  [PinkIK] {arm} 肩系 IK 求解失败: {e}")
            return None

        # 钳制到限位
        new_q = configuration.q.copy()
        for jid in range(1, rmodel.njoints):
            qi = rmodel.idx_qs[jid]
            if rmodel.nqs[jid] == 1:
                new_q[qi] = np.clip(new_q[qi],
                                    rmodel.lowerPositionLimit[qi],
                                    rmodel.upperPositionLimit[qi])

        # 位置误差检查
        configuration = pink.Configuration(rmodel, rdata, new_q)
        current_pos = configuration.get_transform_frame_to_world(
            self.ARM_LINKS[arm]).translation
        error = np.linalg.norm(current_pos - target_se3.translation)
        if error > 1.0:
            return None

        angles = np.array([np.degrees(new_q[qi]) for qi in qidx])

        # 同步全模型 _current_q 的本臂关节，保证全身 FK（含身体状态）一致
        for i, jname in enumerate(self.arm_joints[arm]):
            qi_full = self._q_idx(jname)
            if qi_full >= 0:
                self._current_q[qi_full] = np.radians(angles[i])

        # 手腕直控覆盖（与 solve() 一致）
        if lock_wrist and current_angles is not None and len(current_angles) >= 7:
            angles[4] = current_angles[4]
            angles[5] = current_angles[5]
            angles[6] = current_angles[6]

        return angles
