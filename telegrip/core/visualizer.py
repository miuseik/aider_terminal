"""
PyBullet SO100 机器人可视化模块。
处理 3D 可视化、标记点和坐标系。"""

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from typing import Dict, List, Optional, Tuple
import logging
import sys
import contextlib
from scipy.spatial.transform import Rotation as R

from ..config import (
    JOINT_NAMES, NUM_JOINTS, URDF_TO_INTERNAL_NAME_MAP, 
    END_EFFECTOR_LINK_NAME
)

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """上下文管理器，在文件描述符级别抑制标准输出和标准错误输出。"""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class PyBulletVisualizer:
    """机器人遥操作的 PyBullet 可视化工具。"""
    
    def __init__(self, urdf_path: str, use_gui: bool = True, log_level: str = "warning"):
        self.urdf_path = urdf_path
        self.use_gui = use_gui
        self.log_level = log_level
        
        # PyBullet 状态
        self.physics_client = None
        self.robot_ids = {'left': None, 'right': None}  # 两个机器人实例
        self.aloha_id = None  # Aloha 底盘实例
        self.joint_indices = {'left': [None] * NUM_JOINTS, 'right': [None] * NUM_JOINTS}  # 两个机械臂的关节索引
        self.end_effector_link_indices = {'left': -1, 'right': -1}  # 两个机械臂的末端执行器链接
        
        # 可视化标记点
        self.viz_markers = {}
        self.debug_line_ids = {}
        
        # 关节限位
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        self.is_connected = False
    
    def _can_use_display(self) -> bool:
        """检查显示是否可用于带 OpenGL 支持的 GUI 模式。"""
        # Windows 不需要 X11 检查，直接返回 True
        if os.name == 'nt':  # Windows
            return True
            
        display = os.environ.get('DISPLAY')
        if not display:
            return False
        # Try to verify X11 connection is possible
        try:
            import subprocess
            result = subprocess.run(
                ['xdpyinfo'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2
            )
            if result.returncode != 0:
                return False

            # Also check if GLX (OpenGL) is available - this fails over SSH X11 forwarding
            result = subprocess.run(
                ['glxinfo'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            if result.returncode != 0:
                print("glxinfo failed - OpenGL not available")
                return False

            # Check for common failure indicators in glxinfo output
            output = result.stdout.decode('utf-8', errors='ignore') + result.stderr.decode('utf-8', errors='ignore')
            if 'Error' in output or 'failed' in output.lower():
                print("glxinfo reported errors - OpenGL context may not work")
                return False

            return True
        except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
            print(f"Display check failed: {e}")
            return False

    def setup(self) -> bool:
        """初始化 PyBullet 并加载机器人。"""
        # 确定是否应该抑制输出（但不抑制 GUI 显示）
        should_suppress_output = getattr(logging, self.log_level.upper()) > logging.INFO

        # 在尝试 GUI 模式之前检查显示是否可用
        use_gui = self.use_gui
        if use_gui and not self._can_use_display():
            print("No display available (X11 not connected), falling back to headless mode")
            use_gui = False

        try:
            # GUI 可见性由 use_gui 标志控制，而非日志级别
            if use_gui:
                if should_suppress_output:
                    # 抑制控制台输出但仍显示 GUI
                    with suppress_stdout_stderr():
                        self.physics_client = p.connect(p.GUI)
                else:
                    self.physics_client = p.connect(p.GUI)
            else:
                if should_suppress_output:
                    with suppress_stdout_stderr():
                        self.physics_client = p.connect(p.DIRECT)
                else:
                    self.physics_client = p.connect(p.DIRECT)
        except p.error as e:
            print(f"Could not connect to PyBullet: {e}")
            try:
                if should_suppress_output:
                    with suppress_stdout_stderr():
                        self.physics_client = p.connect(p.DIRECT)
                else:
                    self.physics_client = p.connect(p.DIRECT)
                    print("Fallback to DIRECT mode")
            except p.error:
                print("Failed to connect to PyBullet")
                return False
        
        if self.physics_client < 0:
            return False
        
        # 配置 PyBullet 以减少输出（仅在不使用 GUI 时）
        if should_suppress_output and not self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        
        # 即使使用 GUI，也关闭一些不必要的元素以提高性能
        if self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 关闭侧边栏
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # 保持渲染
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)  # 关闭线框模式
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        if should_suppress_output:
            with suppress_stdout_stderr():
                p.loadURDF("plane.urdf")
        else:
            p.loadURDF("plane.urdf")
        
        # 加载机器人 URDF
        if not os.path.exists(self.urdf_path):
            print(f"URDF file not found: {self.urdf_path}")
            return False
        
        try:
            if should_suppress_output:
                with suppress_stdout_stderr():
                    self.robot_ids['left'] = p.loadURDF(self.urdf_path, [0.2, 0, 0], [0, 0, 0, 1], useFixedBase=1)
            else:
                self.robot_ids['left'] = p.loadURDF(self.urdf_path, [0.2, 0, 0], [0, 0, 0, 1], useFixedBase=1)
        except p.error as e:
            print(f"Failed to load URDF: {e}")
            return False
        
        # 在 X 方向 40cm 处加载右侧机器人
        try:
            if should_suppress_output:
                with suppress_stdout_stderr():
                    self.robot_ids['right'] = p.loadURDF(self.urdf_path, [-0.2, 0, 0], [0, 0, 0, 1], useFixedBase=1)
            else:
                self.robot_ids['right'] = p.loadURDF(self.urdf_path, [-0.2, 0, 0], [0, 0, 0, 1], useFixedBase=1)
        except p.error as e:
            print(f"Failed to load right robot URDF: {e}")
            return False
        
        # === 加载 Aloha 移动底盘 URDF 模型 ===
        # 1. 构建 URDF 文件路径
        #    __file__ = visualizer.py 的绝对路径
        #    os.path.dirname() × 3 = 向上追溯三层目录,到达 telegrip/ 根目录
        #    最终路径: /home/zwz/www/lerobot/aider/aider_terminal/URDF/aloha/Aloha.urdf
        aloha_urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 
                                       "URDF", "aloha", "Aloha.urdf")
        
        # 2. 检查文件是否存在
        if os.path.exists(aloha_urdf_path):
            try:
                # 3. 根据日志级别决定是否抑制 PyBullet 的输出信息
                if should_suppress_output:
                    # 使用上下文管理器屏蔽 stdout/stderr(避免刷屏)
                    with suppress_stdout_stderr():
                        # 4. 加载 URDF 到 PyBullet 物理引擎
                        #    - [0, 0, 0]: 初始位置(世界坐标系原点)
                        #    - [0, 0, 0, 1]: 初始朝向(四元数,无旋转)
                        #    - useFixedBase=1: 固定基座(底盘不会因重力掉落)
                        self.aloha_id = p.loadURDF(aloha_urdf_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=1)
                else:
                    # 不抑制输出,直接加载
                    self.aloha_id = p.loadURDF(aloha_urdf_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=1)
                
                # 5. 如果日志级别允许(INFO 及以上),打印成功消息
                if getattr(logging, self.log_level.upper()) <= logging.INFO:
                    print("Aloha chassis loaded successfully")
                    
            except p.error as e:
                # 6. 捕获 PyBullet 加载错误(如 URDF 格式问题)
                print(f"Failed to load Aloha URDF: {e}")
        else:
            # 7. URDF 文件不存在,记录警告(不影响程序运行,只是看不到底盘模型)
            print(f"Aloha URDF not found at: {aloha_urdf_path}")
        
        # === 将关节名称映射到 PyBullet 索引 ===
        if not self._map_joints():
            return False
        
        # 查找末端执行器链接
        if not self._find_end_effector():
            return False
        
        # 读取关节限位
        self._read_joint_limits()
        
        # 创建可视化标记点
        self._create_markers()
        
        # 设置相机位置（机器人后方，负 Y 方向）
        self._setup_camera()
        
        self.is_connected = True
        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            print("PyBullet visualization setup complete")
        return True
    
    def _map_joints(self) -> bool:
        """将关节名称映射到两个机器人的 PyBullet 索引。"""
        success = True
        
        for arm_name, robot_id in self.robot_ids.items():
            if getattr(logging, self.log_level.upper()) <= logging.INFO:
                print(f"Mapping joints for {arm_name} robot:")
            num_joints = p.getNumJoints(robot_id)
            p_name_to_index = {}
            
            for i in range(num_joints):
                info = p.getJointInfo(robot_id, i)
                joint_name = info[1].decode('UTF-8')
                joint_type = info[2]
                if getattr(logging, self.log_level.upper()) <= logging.INFO:
                    print(f"  Index: {i}, Name: '{joint_name}', Type: {joint_type}")
                p_name_to_index[joint_name] = i
                if joint_type != p.JOINT_FIXED:
                    p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)
            
            # 映射到我们的关节索引
            mapped_count = 0
            for urdf_name, internal_name in URDF_TO_INTERNAL_NAME_MAP.items():
                if internal_name in JOINT_NAMES and urdf_name in p_name_to_index:
                    target_idx = JOINT_NAMES.index(internal_name)
                    self.joint_indices[arm_name][target_idx] = p_name_to_index[urdf_name]
                    mapped_count += 1
                    if getattr(logging, self.log_level.upper()) <= logging.INFO:
                        print(f"  Mapped: '{internal_name}' -> '{urdf_name}' (Index {p_name_to_index[urdf_name]})")
            
            if mapped_count < NUM_JOINTS:
                missing = [name for i, name in enumerate(JOINT_NAMES) if self.joint_indices[arm_name][i] is None]
                print(f"Could not map all joints for {arm_name} robot. Missing: {missing}")
                success = False
        
        return success
    
    def _find_end_effector(self) -> bool:
        """查找两个机器人的末端执行器链接索引。"""
        success = True
        
        for arm_name, robot_id in self.robot_ids.items():
            num_joints = p.getNumJoints(robot_id)
            found = False
            for i in range(num_joints):
                info = p.getJointInfo(robot_id, i)
                link_name = info[12].decode('UTF-8')
                if link_name == END_EFFECTOR_LINK_NAME:
                    self.end_effector_link_indices[arm_name] = i
                    if getattr(logging, self.log_level.upper()) <= logging.INFO:
                        print(f"Found end effector link '{END_EFFECTOR_LINK_NAME}' for {arm_name} robot at index {i}")
                    found = True
                    break
            
            if not found:
                print(f"Could not find end effector link '{END_EFFECTOR_LINK_NAME}' for {arm_name} robot")
                success = False
        
        return success
    
    def _read_joint_limits(self):
        """从 URDF 读取关节限位（使用左侧机器人作为参考）。"""
        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            print("Reading URDF joint limits:")
        for i in range(NUM_JOINTS):
            pb_index = self.joint_indices['left'][i]
            joint_name = JOINT_NAMES[i]
            if pb_index is not None:
                joint_info = p.getJointInfo(self.robot_ids['left'], pb_index)
                lower, upper = joint_info[8], joint_info[9]
                if lower < upper:
                    self.joint_limits_min_deg[i] = math.degrees(lower)
                    self.joint_limits_max_deg[i] = math.degrees(upper)
                    if getattr(logging, self.log_level.upper()) <= logging.INFO:
                        print(f"  {joint_name}: {self.joint_limits_min_deg[i]:.1f}° to {self.joint_limits_max_deg[i]:.1f}°")
                else:
                    if getattr(logging, self.log_level.upper()) <= logging.INFO:
                        print(f"  {joint_name}: No limits found, using defaults")
    
    def _create_markers(self):
        """创建可视化标记点。"""
        # 两个机械臂的目标标记点
        red_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
        self.viz_markers['left_target'] = p.createMultiBody(baseVisualShapeIndex=red_shape, basePosition=[0, 0, -1])
        
        blue_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8])
        self.viz_markers['right_target'] = p.createMultiBody(baseVisualShapeIndex=blue_shape, basePosition=[0, 0, -1])
        
        # 目标点标记
        green_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[0, 1, 0, 0.9])
        self.viz_markers['left_goal'] = p.createMultiBody(baseVisualShapeIndex=green_shape, basePosition=[0, 0, -1])
        
        yellow_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[1, 1, 0, 0.9])
        self.viz_markers['right_goal'] = p.createMultiBody(baseVisualShapeIndex=yellow_shape, basePosition=[0, 0, -1])
        
        # 初始化坐标系
        self.viz_markers['left_target_frame'] = []
        self.viz_markers['right_target_frame'] = []
        self.viz_markers['left_goal_frame'] = []
        self.viz_markers['right_goal_frame'] = []
        
        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # RGB for XYZ
        for marker_name in ['left_target_frame', 'right_target_frame', 'left_goal_frame', 'right_goal_frame']:
            frame_lines = []
            for i in range(3):
                line_id = p.addUserDebugLine([0, 0, -1], [0, 0, -1], lineColorRGB=axis_colors[i], lineWidth=3)
                frame_lines.append(line_id)
            self.viz_markers[marker_name] = frame_lines
    
    def _setup_camera(self):
        """设置相机位置（机器人后方，负 Y 方向）。"""
        # 在负 Y 方向将相机定位在机器人后方
        camera_distance = 0.5  # 距目标的距离
        camera_yaw = 160       # 从负 Y 看向正 Y（朝向机器人）
        camera_pitch = -30    # 轻微向下角度
        camera_target = [0.0, 0.0, 0.2]  # 看向机器人工作区中心
        
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw, 
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target
        )
        
        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            print(f"Camera positioned behind robot at distance={camera_distance}, yaw={camera_yaw}°, pitch={camera_pitch}°")
    
    def update_robot_pose(self, joint_angles_deg: np.ndarray, arm: str = 'left'):
        """更新指定机械臂在可视化中的关节位置。"""
        if not self.is_connected or arm not in self.robot_ids:
            return
        
        joint_angles_rad = np.deg2rad(joint_angles_deg)
        for i in range(NUM_JOINTS):
            if self.joint_indices[arm][i] is not None:
                joint_name = JOINT_NAMES[i]
                urdf_name = None
                for urdf_name_candidate, internal_name in URDF_TO_INTERNAL_NAME_MAP.items():
                    if internal_name == joint_name:
                        urdf_name = urdf_name_candidate
                        break
                
                p.resetJointState(self.robot_ids[arm], self.joint_indices[arm][i], joint_angles_rad[i])
    
    def update_aloha_arm_pose(self, joint_angles_deg: np.ndarray, arm: str = 'left'):
        """更新 AlohaMini 机械臂姿态（带仿真偏移）。
        
        Args:
            joint_angles_deg: SO100 关节角度数组（度）
            arm: 'left' 或 'right'
        """
        if not self.is_connected or self.aloha_id is None:
            return
        
        # 【秘密武器】给 Aloha 的第2、3关节加偏移，对齐 SO100 初始姿态
        adjusted_angles = joint_angles_deg.copy()
        if len(adjusted_angles) >= 3:
            adjusted_angles[1] += 90.0   # 第2关节 +90°
            adjusted_angles[2] -= 90.0   # 第3关节 -90°
        
        joint_angles_rad = np.deg2rad(adjusted_angles)
        
        num_joints = p.getNumJoints(self.aloha_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.aloha_id, i)
            joint_name = info[1].decode('UTF-8')
            
            # 左臂关节
            if arm == 'left' and joint_name.startswith('left_joint'):
                joint_num = int(joint_name.replace('left_joint', '')) - 1  # 0-indexed
                if 0 <= joint_num < 6:
                    p.resetJointState(self.aloha_id, i, joint_angles_rad[joint_num])
            
            # 右臂关节
            elif arm == 'right' and joint_name.startswith('right_joint'):
                joint_num = int(joint_name.replace('right_joint', '')) - 1  # 0-indexed
                if 0 <= joint_num < 6:
                    p.resetJointState(self.aloha_id, i, joint_angles_rad[joint_num])
    
    def set_aloha_height(self, height: float):
        """设置 Aloha 底盘的升降高度。
        
        Args:
            height: 期望的实际高度 (米)
        """
        if not self.is_connected or self.aloha_id is None:
            return
        
        # URDF 中 vertical_move 关节的基础偏移量
        URDF_HEIGHT = 0.45
        
        # 计算关节值：期望高度 - URDF 偏移
        joint_value = height - URDF_HEIGHT

        # 找到 vertical_move 关节索引
        num_joints = p.getNumJoints(self.aloha_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.aloha_id, i)
            joint_name = info[1].decode('UTF-8')
            if joint_name == "vertical_move":
                p.resetJointState(self.aloha_id, i, joint_value)
                break

    def update_mobile_base_simulation(self, action_dict: dict):
        """在仿真中更新移动底盘和升降轴的状态。"""
        if not self.is_connected or self.aloha_id is None:
            return

        # 1. 处理升降轴 (从 Action 字典中提取高度指令)
        if "lift.height_mm" in action_dict:
            height_m = action_dict["lift.height_mm"] / 1000.0
            self.set_aloha_height(height_m)

        # 2. 处理底盘运动（直接使用车身坐标系速度）
        pos, orn = p.getBasePositionAndOrientation(self.aloha_id)
        
        # 提取车身速度指令
        vx = action_dict.get("base.vx", 0)
        vy = action_dict.get("base.vy", 0)
        v_theta = action_dict.get("base.vtheta", 0)
        
        dt = 0.05
        
        euler = p.getEulerFromQuaternion(orn)
        new_yaw = euler[2] + v_theta * dt
        new_orn = p.getQuaternionFromEuler([euler[0], euler[1], new_yaw])
        
        # 将车身坐标系的速度转换到世界坐标系（考虑当前朝向）
        cos_yaw = math.cos(new_yaw)
        sin_yaw = math.sin(new_yaw)
        
        # vx: 车身前后 → world_y
        # vy: 车身左右 → world_x
        delta_x = (vy * cos_yaw - vx * sin_yaw) * dt
        delta_y = (vx * cos_yaw + vy * sin_yaw) * dt
        
        new_x = pos[0] + delta_x
        new_y = pos[1] + delta_y
        
        p.resetBasePositionAndOrientation(self.aloha_id, [new_x, new_y, pos[2]], new_orn)
    
    def update_marker_position(self, marker_name: str, position: np.ndarray, 
                              orientation: Optional[np.ndarray] = None):
        """更新可视化标记点的位置。"""
        if not self.is_connected or marker_name not in self.viz_markers:
            return
        
        if orientation is None:
            orientation = [0, 0, 0, 1]
        
        p.resetBasePositionAndOrientation(
            self.viz_markers[marker_name], 
            position.tolist(), 
            orientation
        )
    
    def update_coordinate_frame(self, frame_name: str, position: np.ndarray, 
                               orientation_quat: Optional[np.ndarray] = None):
        """更新坐标系可视化。"""
        if not self.is_connected or frame_name not in self.viz_markers:
            return
        
        frame_lines = self.viz_markers[frame_name]
        if not frame_lines:
            return
        
        axis_length = 0.05
        
        # 默认为单位旋转
        if orientation_quat is None:
            orientation_quat = [0, 0, 0, 1]
        
        # 将四元数转换为旋转矩阵
        r = R.from_quat(orientation_quat)
        rotation_matrix = r.as_matrix()
        
        # 更新每个坐标轴线（X=红色, Y=绿色, Z=蓝色）
        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for i in range(3):
            if i < len(frame_lines):
                axis_vector = rotation_matrix[:, i] * axis_length
                end_point = position + axis_vector
                
                p.addUserDebugLine(
                    position.tolist(), 
                    end_point.tolist(), 
                    lineColorRGB=axis_colors[i], 
                    lineWidth=3,
                    replaceItemUniqueId=frame_lines[i]
                )
    
    def hide_marker(self, marker_name: str):
        """通过将标记点移出屏幕来隐藏它。"""
        if marker_name in self.viz_markers:
            self.update_marker_position(marker_name, np.array([0, 0, -1]))
    
    def hide_frame(self, frame_name: str):
        """隐藏坐标系。"""
        if frame_name in self.viz_markers:
            frame_lines = self.viz_markers[frame_name]
            for line_id in frame_lines:
                p.addUserDebugLine(
                    [0, 0, -1], [0, 0, -1], 
                    lineColorRGB=[0, 0, 0], 
                    lineWidth=1,
                    replaceItemUniqueId=line_id
                )
    
    def step_simulation(self):
        """推进仿真向前一步。"""
        if self.is_connected:
            p.stepSimulation()
    
    def disconnect(self):
        """断开与 PyBullet 的连接。"""
        if self.is_connected and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
            self.is_connected = False
            if getattr(logging, self.log_level.upper()) <= logging.INFO:
                print("PyBullet disconnected")
    
    @property
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取关节限位（度）。"""
        return self.joint_limits_min_deg.copy(), self.joint_limits_max_deg.copy() 