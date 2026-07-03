"""
Aloha 机器人 PyBullet 可视化器。
加载两个 SO100 臂 URDF + 一个 Aloha 基底 URDF，管理关节映射和可视化。
"""

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from typing import Dict, Optional, Tuple
import logging
from scipy.spatial.transform import Rotation as R

from aiderminal.config.settings import (
    NUM_JOINTS, JOINT_NAMES, ARM_JOINT_NAMES_LEFT, ARM_JOINT_NAMES_RIGHT,
    END_EFFECTOR_LINK_NAME, ALOHA_URDF_PATH,
)


class AlohaVisualizer:
    """Aloha 机器人 PyBullet 仿真可视化器。

    加载两个 SO100 臂 URDF + Aloha 基底 URDF（可选），
    管理关节索引映射和可视化标记点。
    """

    def __init__(self, urdf_path: str, use_gui: bool = True, log_level: str = "warning",
                 aloha_urdf_path: str = None):
        self.urdf_path = urdf_path                   # SO100 arm URDF
        self.use_gui = use_gui
        self.log_level = log_level
        self.aloha_urdf_path = aloha_urdf_path or ALOHA_URDF_PATH  # Aloha base URDF

        self.physics_client = None
        self.aider_id = None                 # Aloha 不需要 Aider ID
        self.aloha_id = None                # Aloha 基底 URDF 实例
        self.robot_ids = {'left': None, 'right': None}  # 左/右 SO100 臂
        self.joint_indices = {'left': [None] * NUM_JOINTS, 'right': [None] * NUM_JOINTS}
        self.end_effector_link_indices = {'left': -1, 'right': -1}

        self.viz_markers = {}
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        self.is_connected = False

    # ==================== 初始化 ====================

    def setup(self) -> bool:
        """初始化 PyBullet 并加载 SO100 双臂 + Aloha 基底。"""
        try:
            if self.use_gui:
                self.physics_client = p.connect(p.GUI)
                print("PyBullet GUI 已连接")
            else:
                self.physics_client = p.connect(p.DIRECT)
        except p.error as e:
            print(f"PyBullet GUI 连接失败: {e}, 回退 headless")
            try:
                self.physics_client = p.connect(p.DIRECT)
            except p.error:
                print("PyBullet 连接完全失败")
                return False

        if self.physics_client < 0:
            return False

        if self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.02)
        p.loadURDF("plane.urdf")

        # 加载 SO100 双臂
        if not os.path.exists(self.urdf_path):
            print(f"SO100 URDF not found: {self.urdf_path}")
            return False

        so100_dir = os.path.dirname(self.urdf_path)
        so100_mesh = os.path.join(os.path.dirname(so100_dir), "meshes")
        if os.path.exists(so100_mesh):
            p.setAdditionalSearchPath(so100_mesh)

        self.robot_ids['left'] = p.loadURDF(
            self.urdf_path, [0.2, 0, 0], [0, 0, 0, 1], useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
        print(f"SO100 left arm loaded (ID={self.robot_ids['left']})")

        self.robot_ids['right'] = p.loadURDF(
            self.urdf_path, [-0.2, 0, 0], [0, 0, 0, 1], useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
        print(f"SO100 right arm loaded (ID={self.robot_ids['right']})")

        # 加载 Aloha 基底 URDF
        if not os.path.exists(self.aloha_urdf_path):
            print(f"Aloha URDF not found: {self.aloha_urdf_path}，跳过 Aloha 基底")
            self.aloha_id = None
        else:
            aloha_dir = os.path.dirname(self.aloha_urdf_path)
            aloha_mesh = os.path.join(aloha_dir, "meshes")
            if os.path.exists(aloha_mesh):
                p.setAdditionalSearchPath(aloha_mesh)
            try:
                self.aloha_id = p.loadURDF(self.aloha_urdf_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
                print(f"Aloha base loaded (ID={self.aloha_id})")
            except p.error as e:
                print(f"Failed to load Aloha URDF: {e}")
                self.aloha_id = None

        if not self._map_joints():
            return False

        self._read_joint_limits()
        self._create_markers()
        self._setup_camera()
        self.is_connected = True
        print("PyBullet Aloha visualization setup complete")
        return True

    def _map_joints(self) -> bool:
        """映射 SO100 URDF 关节到 PyBullet 索引。"""
        for arm_side, robot_id in [("left", self.robot_ids['left']),
                                    ("right", self.robot_ids['right'])]:
            if robot_id is None:
                return False

            num_joints = p.getNumJoints(robot_id)
            pybullet_name_map: Dict[str, int] = {}

            if getattr(logging, self.log_level.upper()) <= logging.INFO:
                print(f"Mapping joints for SO100 {arm_side} (ID={robot_id}, {num_joints} joints):")

            for i in range(num_joints):
                info = p.getJointInfo(robot_id, i)
                joint_name = info[1].decode('UTF-8')
                pybullet_name_map[joint_name] = i
                if getattr(logging, self.log_level.upper()) <= logging.INFO:
                    print(f"  [{i}] '{joint_name}' type={info[2]}")

            arm_joint_names = ARM_JOINT_NAMES_LEFT if arm_side == "left" else ARM_JOINT_NAMES_RIGHT
            for i, urdf_name in enumerate(arm_joint_names):
                if urdf_name in pybullet_name_map:
                    self.joint_indices[arm_side][i] = pybullet_name_map[urdf_name]

            # 末端执行器（SO100 双臂 URDF 相同，joint 名字固定）
            ee_name = END_EFFECTOR_LINK_NAME
            if ee_name in pybullet_name_map:
                self.end_effector_link_indices[arm_side] = pybullet_name_map[ee_name]

        mapped_l = sum(1 for x in self.joint_indices['left'] if x is not None)
        mapped_r = sum(1 for x in self.joint_indices['right'] if x is not None)
        print(f"SO100 joint mapping: left={mapped_l}/{NUM_JOINTS}, right={mapped_r}/{NUM_JOINTS}")
        return mapped_l >= NUM_JOINTS and mapped_r >= NUM_JOINTS

    def _read_joint_limits(self):
        robot_id = self.robot_ids.get('left')
        if robot_id is None:
            return
        for i in range(NUM_JOINTS):
            pb_index = self.joint_indices['left'][i]
            if pb_index is not None:
                joint_info = p.getJointInfo(robot_id, pb_index)
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

        self.viz_markers.update({'left_target_frame': [], 'right_target_frame': [],
                                  'left_goal_frame': [], 'right_goal_frame': []})
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
        """更新指定机械臂在 SO100 URDF 中的关节位置。"""
        if not self.is_connected:
            return
        robot_id = self.robot_ids.get(arm)
        if robot_id is None:
            return
        joint_angles_rad = np.deg2rad(joint_angles_deg)
        for i in range(min(NUM_JOINTS, len(joint_angles_rad))):
            if self.joint_indices[arm][i] is not None:
                p.resetJointState(robot_id, self.joint_indices[arm][i],
                                  joint_angles_rad[i], physicsClientId=self.physics_client)

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
                p.addUserDebugLine(position.tolist(), (position + axis_vector).tolist(),
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
        return f"aloha_id={self.aloha_id} | robot_ids={list(self.robot_ids.keys())}"

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
