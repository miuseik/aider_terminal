"""
Aider 机器人 PyBullet 可视化器。
加载 Aider 完整 URDF（双臂 + 身体 + 底盘 + 4 轮），管理关节映射和可视化。
"""

import os
import math
import subprocess
import numpy as np
import pybullet as p
import pybullet_data
from typing import Dict, Optional, Tuple
import logging
from scipy.spatial.transform import Rotation as R

from aiderminal.config.settings import (
    NUM_JOINTS, JOINT_NAMES, ARM_JOINT_NAMES_LEFT, ARM_JOINT_NAMES_RIGHT,
)

class AiderVisualizer:
    """Aider 机器人 PyBullet 仿真可视化器。

    加载完整 Aider URDF（含双臂、身体关节、4 轮底盘），
    管理关节索引映射和可视化标记点。
    """

    def __init__(self, urdf_path: str, use_gui: bool = True, log_level: str = "warning"):
        self.urdf_path = urdf_path
        self.use_gui = use_gui
        self.log_level = log_level

        self.physics_client = None
        self.aider_id = None                # Aider 完整机器人 URDF 实例
        self.aloha_id = None                # Aider 不需要 Aloha ID
        self.robot_ids = {'left': None, 'right': None}  # 指向 aider_id
        self.joint_indices = {'left': [None] * NUM_JOINTS, 'right': [None] * NUM_JOINTS}
        self.end_effector_link_indices = {'left': -1, 'right': -1}

        self.body_joint_indices: Dict[str, int] = {}
        self.wheel_joint_indices: Dict[str, int] = {}

        self.viz_markers = {}
        self.debug_line_ids = {}
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        self.is_connected = False

    # ==================== 初始化 ====================

    @staticmethod
    def _has_x11_display() -> bool:
        """可靠检测 X11 是否可用（xdpyinfo 连通性测试）。

        pybullet GUI 在无 X11 环境会 C 层崩溃，try/except 兜不住，
        因此必须在 p.connect(GUI) 之前做可靠判断。

        仅检查 DISPLAY + socket 文件不够 —— Xauth cookie/权限
        可能不匹配。用 xdpyinfo 做真正的连接测试。
        """
        display = os.environ.get('DISPLAY', '')
        if not display:
            return False
        try:
            subprocess.run(
                ['xdpyinfo'], check=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=2,
            )
            return True
        except Exception:
            return False

    def setup(self) -> bool:
        """初始化 PyBullet 并加载 Aider URDF。"""
        has_display = self._has_x11_display()
        use_gui = self.use_gui and has_display

        # 写文件诊断（C 层崩溃时 print 可能来不及 flush）
        try:
            with open('/tmp/pybullet_diag.log', 'a') as _df:
                _df.write(f"[AiderVisualizer.setup] use_gui={self.use_gui} "
                          f"DISPLAY={os.environ.get('DISPLAY', 'NOT_SET')} "
                          f"has_x11={has_display} use_gui_final={use_gui}\n")
        except Exception:
            pass

        if not has_display and self.use_gui:
            print("⚠️ 无 X11 显示，PyBullet 将以 headless 模式运行")

        try:
            if use_gui:
                with open('/tmp/pybullet_diag.log', 'a') as _df:
                    _df.write("[AiderVisualizer.setup] 即将调用 p.connect(p.GUI)...\n")
                self.physics_client = p.connect(p.GUI)
                print("PyBullet GUI 已连接")
            else:
                with open('/tmp/pybullet_diag.log', 'a') as _df:
                    _df.write("[AiderVisualizer.setup] 即将调用 p.connect(p.DIRECT)...\n")
                self.physics_client = p.connect(p.DIRECT)
        except p.error as e:
            print(f"PyBullet 连接失败: {e}")
            try:
                self.physics_client = p.connect(p.DIRECT)
            except p.error:
                print("PyBullet 连接完全失败")
                return False

        if self.physics_client < 0:
            return False

        if use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.02)
        p.loadURDF("plane.urdf")

        # 加载 Aider URDF（自动压缩超大网格）
        if not os.path.exists(self.urdf_path):
            print(f"Aider URDF not found: {self.urdf_path}")
            return False

        import re, tempfile

        urdf_dir = os.path.dirname(self.urdf_path)          # .../URDF/aider/urdf/
        aider_pkg_dir = os.path.dirname(urdf_dir)           # .../URDF/aider/
        mesh_dir = os.path.join(aider_pkg_dir, "meshes")    # .../URDF/aider/meshes/

        # ---- 自动压缩大 STL（缺失依赖会自动安装） ----
        from aiderminal.utils.mesh_compressor import compress_directory
        saved, msg = compress_directory(mesh_dir, max_single_mb=2.0, total_budget_mb=150.0)
        if saved > 0:
            print(f"  [网格压缩] {msg}")

        # ---- 构建压缩后的 mesh 路径映射 ----
        compressed_dir = os.path.join(mesh_dir, "compressed")
        def _resolve_mesh(filename: str) -> str:
            """优先使用压缩版，不存在则回退到原始文件。"""
            compressed = os.path.join(compressed_dir, filename)
            if os.path.exists(compressed):
                return compressed
            return os.path.join(mesh_dir, filename)

        # ---- 重写 URDF mesh 路径 ----
        with open(self.urdf_path, 'r', encoding='utf-8') as f:
            urdf_content = f.read()

        def _replace_mesh_uri(match):
            filename = match.group(1)
            return f'filename="{_resolve_mesh(filename)}"'

        # 把 package://xxx/meshes/yyy.STL → 实际文件路径
        urdf_content = re.sub(
            r'filename="package://[^"]*/([^/"]+)"',
            _replace_mesh_uri,
            urdf_content
        )

        # 写入临时文件加载
        tmp_urdf = tempfile.NamedTemporaryFile(
            mode='w', suffix='.urdf', delete=False, encoding='utf-8')
        tmp_urdf.write(urdf_content)
        tmp_urdf.close()

        try:
            p.setAdditionalSearchPath(mesh_dir)
            if os.path.isdir(compressed_dir):
                p.setAdditionalSearchPath(compressed_dir)
            self.aider_id = p.loadURDF(tmp_urdf.name, [0, 0, 0], [0, 0, 0, 1], useFixedBase=False)
            print(f"Aider robot loaded successfully (ID={self.aider_id})")
        except p.error as e:
            print(f"Failed to load Aider URDF: {e}")
            return False
        finally:
            os.unlink(tmp_urdf.name)

        self.robot_ids = {'left': self.aider_id, 'right': self.aider_id}

        if not self._map_joints():
            return False

        self._read_joint_limits()
        self._create_markers()
        self._setup_camera()
        self.is_connected = True
        print("PyBullet Aider visualization setup complete")
        return True

    def _map_joints(self) -> bool:
        """映射 Aider URDF 中所有关节到 PyBullet 索引。"""
        if self.aider_id is None:
            return False

        num_joints = p.getNumJoints(self.aider_id)
        pybullet_name_map: Dict[str, int] = {}

        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            print(f"Mapping joints for Aider (ID={self.aider_id}, {num_joints} joints):")

        for i in range(num_joints):
            info = p.getJointInfo(self.aider_id, i)
            joint_name = info[1].decode('UTF-8')
            joint_type = info[2]
            pybullet_name_map[joint_name] = i

            if getattr(logging, self.log_level.upper()) <= logging.INFO:
                print(f"  [{i}] '{joint_name}' type={joint_type}")

            # 身体关节
            if joint_name in ("lift_Link", "waist_Link", "head_Link", "head_Link2"):
                self.body_joint_indices[joint_name] = i

            # 轮子关节
            if joint_name.startswith("whel_Link"):
                self.wheel_joint_indices[joint_name] = i

        # 映射左臂 8 关节
        for i, urdf_name in enumerate(ARM_JOINT_NAMES_LEFT):
            if urdf_name in pybullet_name_map:
                self.joint_indices['left'][i] = pybullet_name_map[urdf_name]

        # 映射右臂 8 关节
        for i, urdf_name in enumerate(ARM_JOINT_NAMES_RIGHT):
            if urdf_name in pybullet_name_map:
                self.joint_indices['right'][i] = pybullet_name_map[urdf_name]

        # 末端执行器
        if "left_arm8" in pybullet_name_map:
            self.end_effector_link_indices['left'] = pybullet_name_map["left_arm8"]
        if "right_arm8" in pybullet_name_map:
            self.end_effector_link_indices['right'] = pybullet_name_map["right_arm8"]

        mapped_left = sum(1 for x in self.joint_indices['left'] if x is not None)
        mapped_right = sum(1 for x in self.joint_indices['right'] if x is not None)
        print(f"Aider joint mapping: left={mapped_left}/{NUM_JOINTS}, right={mapped_right}/{NUM_JOINTS}, "
              f"body={len(self.body_joint_indices)}, wheels={len(self.wheel_joint_indices)}")
        return mapped_left >= NUM_JOINTS and mapped_right >= NUM_JOINTS

    def _read_joint_limits(self):
        if self.aider_id is None:
            return
        for i in range(NUM_JOINTS):
            pb_index = self.joint_indices['left'][i]
            if pb_index is not None:
                joint_info = p.getJointInfo(self.aider_id, pb_index)
                lower, upper = joint_info[8], joint_info[9]
                if lower < upper:
                    self.joint_limits_min_deg[i] = math.degrees(lower)
                    self.joint_limits_max_deg[i] = math.degrees(upper)
                else:
                    self.joint_limits_min_deg[i] = -180.0
                    self.joint_limits_max_deg[i] = 180.0

    # ==================== 可视化标记 ====================

    def _create_markers(self):
        red_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
        self.viz_markers['left_target'] = p.createMultiBody(baseVisualShapeIndex=red_shape, basePosition=[0, 0, -1])
        blue_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8])
        self.viz_markers['right_target'] = p.createMultiBody(baseVisualShapeIndex=blue_shape, basePosition=[0, 0, -1])
        green_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[0, 1, 0, 0.9])
        self.viz_markers['left_goal'] = p.createMultiBody(baseVisualShapeIndex=green_shape, basePosition=[0, 0, -1])
        yellow_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[1, 1, 0, 0.9])
        self.viz_markers['right_goal'] = p.createMultiBody(baseVisualShapeIndex=yellow_shape, basePosition=[0, 0, -1])

        self.viz_markers['left_target_frame'] = []
        self.viz_markers['right_target_frame'] = []
        self.viz_markers['left_goal_frame'] = []
        self.viz_markers['right_goal_frame'] = []

        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for marker_name in ['left_target_frame', 'right_target_frame', 'left_goal_frame', 'right_goal_frame']:
            frame_lines = []
            for i in range(3):
                line_id = p.addUserDebugLine([0, 0, -1], [0, 0, -1], lineColorRGB=axis_colors[i], lineWidth=3)
                frame_lines.append(line_id)
            self.viz_markers[marker_name] = frame_lines

    def _setup_camera(self):
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5, cameraYaw=160, cameraPitch=-30,
            cameraTargetPosition=[0.0, 0.0, 0.2]
        )

    # ==================== 关节更新 ====================

    def update_robot_pose(self, joint_angles_deg: np.ndarray, arm: str = 'left'):
        """更新指定机械臂在 Aider URDF 中的关节位置。"""
        if not self.is_connected or self.aider_id is None:
            return
        joint_angles_rad = np.deg2rad(joint_angles_deg)
        for i in range(min(NUM_JOINTS, len(joint_angles_rad))):
            if self.joint_indices[arm][i] is not None:
                p.resetJointState(self.aider_id, self.joint_indices[arm][i],
                                  joint_angles_rad[i], physicsClientId=self.physics_client)

    def update_body_joint(self, joint_name: str, angle_rad: float) -> None:
        """更新 Aider 身体关节（腰/头）。"""
        if not self.is_connected or self.aider_id is None:
            return
        if joint_name in self.body_joint_indices:
            p.resetJointState(self.aider_id, self.body_joint_indices[joint_name],
                              angle_rad, physicsClientId=self.physics_client)

    def update_wheel_rotation(self, wheel_name: str, speed_radps: float, dt: float) -> None:
        """累积旋转 Aider 轮子关节。"""
        if not self.is_connected or self.aider_id is None:
            return
        if wheel_name in self.wheel_joint_indices:
            idx = self.wheel_joint_indices[wheel_name]
            state = p.getJointState(self.aider_id, idx, physicsClientId=self.physics_client)
            pos = state[0]
            p.resetJointState(self.aider_id, idx, pos + speed_radps * dt,
                              physicsClientId=self.physics_client)

    # ==================== 标记点更新 ====================

    def update_marker_position(self, marker_name: str, position: np.ndarray,
                              orientation: Optional[np.ndarray] = None):
        if not self.is_connected or marker_name not in self.viz_markers:
            return
        if orientation is None:
            orientation = [0, 0, 0, 1]
        p.resetBasePositionAndOrientation(self.viz_markers[marker_name],
                                          position.tolist(), orientation)

    def update_coordinate_frame(self, frame_name: str, position: np.ndarray,
                               orientation_quat: Optional[np.ndarray] = None):
        if not self.is_connected or frame_name not in self.viz_markers:
            return
        frame_lines = self.viz_markers[frame_name]
        if not frame_lines:
            return
        axis_length = 0.05
        if orientation_quat is None:
            orientation_quat = [0, 0, 0, 1]
        r = R.from_quat(orientation_quat)
        rotation_matrix = r.as_matrix()
        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for i in range(3):
            if i < len(frame_lines):
                axis_vector = rotation_matrix[:, i] * axis_length
                end_point = position + axis_vector
                p.addUserDebugLine(position.tolist(), end_point.tolist(),
                                  lineColorRGB=axis_colors[i], lineWidth=3,
                                  replaceItemUniqueId=frame_lines[i])

    def hide_marker(self, marker_name: str):
        if marker_name in self.viz_markers:
            self.update_marker_position(marker_name, np.array([0, 0, -1]))

    def hide_frame(self, frame_name: str):
        if frame_name in self.viz_markers:
            for line_id in self.viz_markers[frame_name]:
                p.addUserDebugLine([0, 0, -1], [0, 0, -1],
                                  lineColorRGB=[0, 0, 0], lineWidth=1,
                                  replaceItemUniqueId=line_id)

    # ==================== 诊断 ====================

    def get_diagnostic_info(self) -> str:
        return f"aider_id={self.aider_id} | body_joints={len(self.body_joint_indices)} | wheels={len(self.wheel_joint_indices)}"

    # ==================== 生命周期 ====================

    def step_simulation(self):
        if self.is_connected:
            p.stepSimulation()

    def disconnect(self):
        if self.is_connected and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
            self.is_connected = False

    @property
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.joint_limits_min_deg.copy(), self.joint_limits_max_deg.copy()
