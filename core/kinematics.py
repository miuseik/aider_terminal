"""
SO100 机器人的运动学工具。
包含使用 PyBullet 的正向和逆向运动学求解器。
"""

import math
import numpy as np
import pybullet as p
from typing import Optional, Tuple, List
import logging
import json
import os
from pathlib import Path

from config.settings import (
    NUM_JOINTS, NUM_IK_JOINTS, 
    USE_REFERENCE_POSES, REFERENCE_POSES_FILE, IK_POSITION_ERROR_THRESHOLD,
    IK_HYSTERESIS_THRESHOLD, IK_MOVEMENT_PENALTY_WEIGHT
)

logger = logging.getLogger(__name__)

class ForwardKinematics:
    """使用 PyBullet 的正向运动学求解器。"""
    
    def __init__(self, physics_client, robot_id: int, joint_indices: list, end_effector_link_index: int):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
    
    def compute(self, joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算给定关节角度的正向运动学。
        
        Args:
            joint_angles_deg: 关节角度（度）
            
        Returns:
            (位置, 四元数) 的元组，表示末端执行器的位姿
        """
        if self.physics_client is None or self.robot_id is None:
            return np.array([0.2, 0.0, 0.15]), np.array([0, 0, 0, 1])
        
        # 使用关节角度但保持夹爪在中立位置进行 FK 计算
        # 以确保 Wrist_Pitch_Roll 位置独立于夹爪状态
        fk_state_angles = joint_angles_deg.copy()
        fk_state_angles[5] = 0.0  # 将夹爪设置为中立（闭合）位置进行 FK 计算
        
        # 设置关节位置
        joint_angles_rad = np.deg2rad(fk_state_angles)
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_angles_rad[i])
        
        # 获取末端执行器位置和方向
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        position = np.array(link_state[0])
        quaternion = np.array(link_state[1])
        
        return position, quaternion


class IKSolver:
    """使用 PyBullet 和多参考位姿的逆向运动学求解器。"""
    
    def __init__(self, physics_client, robot_id: int, joint_indices: list, 
                 end_effector_link_index: int, joint_limits_min_deg: np.ndarray, 
                 joint_limits_max_deg: np.ndarray, arm_name: str = ""):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
        self.joint_limits_min_deg = joint_limits_min_deg
        self.joint_limits_max_deg = joint_limits_max_deg
        self.arm_name = arm_name
        
        # 预计算前 NUM_IK_JOINTS 个关节的 IK 限制
        self.ik_lower_limits = np.deg2rad(joint_limits_min_deg[:NUM_IK_JOINTS])
        self.ik_upper_limits = np.deg2rad(joint_limits_max_deg[:NUM_IK_JOINTS])
        self.ik_ranges = self.ik_upper_limits - self.ik_lower_limits
        
        # 加载参考位姿
        self.reference_poses = self._load_reference_poses()
        
        # 创建 FK 求解器用于评估解的质量
        self.fk_solver = ForwardKinematics(physics_client, robot_id, joint_indices, end_effector_link_index)
    
    def _load_reference_poses(self) -> List[np.ndarray]:
        """从文件中加载此机械臂的参考位姿。"""
        reference_poses = []
        
        # Check if reference poses are enabled
        if not USE_REFERENCE_POSES:
            print("Reference poses disabled in configuration")
            return reference_poses
        
        try:
            from utils.common_utils import get_absolute_path
            cache_file = get_absolute_path(REFERENCE_POSES_FILE)
            if cache_file.exists():
                with open(cache_file, 'r') as f:
                    data = json.load(f)
                
                arm_poses = data.get(self.arm_name, [])
                if arm_poses:
                    # 转换为 numpy 数组并仅提取前 3 个关节用于 IK
                    for pose in arm_poses:
                        pose_array = np.array(pose[:NUM_IK_JOINTS])
                        pose_rad = np.deg2rad(pose_array)
                        reference_poses.append(pose_rad)
                    
                    print(f"为 {self.arm_name} 机械臂加载了 {len(reference_poses)} 个参考位姿")
                else:
                    print(f"未找到 {self.arm_name} 机械臂的参考位姿")
            else:
                print("未找到参考位姿文件。请使用 read_pose.py 记录参考位姿。")
                
        except Exception as e:
            print(f"Failed to load reference poses: {e}")
        
        return reference_poses
    
    def _evaluate_ik_solution(self, solution: np.ndarray, target_position: np.ndarray, 
                             current_joints_rad: Optional[np.ndarray] = None, 
                             hysteresis_threshold: float = 0.05) -> float:
        """
        基于位置误差和关节移动评估 IK 解的质量。
        
        Args:
            solution: IK 解（弧度）
            target_position: 目标末端执行器位置
            current_joints_rad: 当前关节角度（弧度），用于移动惩罚
            hysteresis_threshold: 切换解所需的最小改进量（米）
        """
        try:
            # 将解转换为完整关节数组（其他关节保持为 0）
            full_angles = np.zeros(NUM_JOINTS)
            full_angles[:NUM_IK_JOINTS] = np.rad2deg(solution)
            
            # 计算正向运动学
            achieved_position, _ = self.fk_solver.compute(full_angles)
            
            # 计算位置误差
            position_error = np.linalg.norm(achieved_position - target_position)
            
            # 如果提供了当前关节，添加关节移动惩罚
            movement_penalty = 0.0
            if current_joints_rad is not None:
                # 计算关节空间距离（仅针对 IK 关节）
                joint_diff = solution - current_joints_rad[:NUM_IK_JOINTS]
                joint_movement = np.linalg.norm(joint_diff)
                
                # 将关节移动转换为位置等效惩罚
                movement_penalty = joint_movement * IK_MOVEMENT_PENALTY_WEIGHT
                
            # 总代价结合位置误差和移动惩罚
            total_cost = position_error + movement_penalty
            return total_cost
            
        except Exception as e:
            print(f"Error evaluating IK solution: {e}")
            return float('inf')
    
    def solve(self, target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray], 
              current_angles_deg: np.ndarray) -> np.ndarray:
        """
        使用前 3 个关节求解位置控制的逆向运动学。
        尝试多个参考位姿并返回最佳解。
        
        Args:
            target_position: 目标末端执行器位置
            target_orientation_quat: 目标方向（可选，如果为 None 则仅位置控制）
            current_angles_deg: 当前关节角度（度）
            
        Returns:
            前 NUM_IK_JOINTS 个关节的角度（度）
        """
        if self.physics_client is None or self.robot_id is None:
            return current_angles_deg[:NUM_IK_JOINTS]
        
        # 获取当前实际机器人位置和误差
        current_actual_position, _ = self.fk_solver.compute(current_angles_deg)
        current_actual_error = np.linalg.norm(current_actual_position - target_position)
        
        # 将当前角度转换为弧度并为 IK 状态做准备
        # 保持夹爪在中立位置以防止夹爪运动影响 IK 目标
        ik_state_angles = current_angles_deg.copy()
        ik_state_angles[5] = 0.0  # 将夹爪设置为中立（闭合）位置进行 IK 计算
        current_angles_rad = np.deg2rad(ik_state_angles)
        
        # 状态管理辅助函数
        def set_robot_to_current_state():
            """辅助函数：将机器人设置为精确的当前状态"""
            for i in range(NUM_JOINTS):
                if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                    p.resetJointState(self.robot_id, self.joint_indices[i], current_angles_rad[i])
        
        def set_robot_to_reference_state(ref_pose_rad: np.ndarray):
            """辅助函数：将机器人设置为参考位姿状态"""
            full_ref_state = current_angles_rad.copy()
            full_ref_state[:NUM_IK_JOINTS] = ref_pose_rad
            for i in range(NUM_JOINTS):
                if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                    p.resetJointState(self.robot_id, self.joint_indices[i], full_ref_state[i])
        
        # 准备要尝试的休息位姿列表
        rest_poses_to_try = []
        
        # 1. 当前配置（最可能接近解）
        current_rest_pose = np.deg2rad(current_angles_deg[:NUM_IK_JOINTS])
        rest_poses_to_try.append(('current', current_rest_pose))
        
        # 2. 来自已记录配置的参考位姿
        for i, ref_pose in enumerate(self.reference_poses):
            rest_poses_to_try.append((f'reference_{i+1}', ref_pose))
        
        best_solution = None
        best_error = float('inf')
        best_source = None
        current_solution_error = None
        current_solution_joints = None
        
        # 单独跟踪最佳参考位姿（与整体最佳分开）
        best_reference_solution = None
        best_reference_error = float('inf')
        best_reference_source = None
        best_reference_position_error = float('inf')  # Pure position error without movement penalty
        
        # 尝试每个休息位姿配置
        for source_name, rest_pose in rest_poses_to_try:
            try:
                # 每次 IK 尝试前始终以干净、已知的机器人状态开始
                if source_name == 'current':
                    # 对于当前位姿，使用精确的当前状态
                    set_robot_to_current_state()
                else:
                    # 对于参考位姿，设置为该参考配置
                    set_robot_to_reference_state(rest_pose)
                
                # 使用适当的休息位姿执行 IK
                ik_solution = p.calculateInverseKinematics(
                    bodyUniqueId=self.robot_id,
                    endEffectorLinkIndex=self.end_effector_link_index,
                    targetPosition=target_position.tolist(),
                    lowerLimits=self.ik_lower_limits.tolist(),
                    upperLimits=self.ik_upper_limits.tolist(),
                    jointRanges=self.ik_ranges.tolist(),
                    restPoses=rest_pose.tolist() if isinstance(rest_pose, np.ndarray) else rest_pose,
                    solver=0,                                # 0 = DLS (Damped Least Squares)
                    maxNumIterations=100,
                    residualThreshold=1e-4
                )
                
                # 关键：每次 IK 尝试后始终恢复到精确的当前状态
                # 这防止了尝试之间的状态污染
                set_robot_to_current_state()
                
                # 评估此解
                solution_array = np.array(ik_solution[:NUM_IK_JOINTS])
                
                # PyBullet 有时会忽略关节限制，因此需要钳制解
                # 将限制转换回度进行比较
                joint_limits_min_deg = np.rad2deg(self.ik_lower_limits)
                joint_limits_max_deg = np.rad2deg(self.ik_upper_limits)
                solution_degrees = np.rad2deg(solution_array)
                
                # 检查并包装 shoulder_pan（第一个关节）如果超出限制
                if solution_degrees[0] < joint_limits_min_deg[0] or solution_degrees[0] > joint_limits_max_deg[0]:
                    # 尝试通过 ±360° 包装
                    for offset in [-360.0, 360.0]:
                        wrapped_angle = solution_degrees[0] + offset
                        if joint_limits_min_deg[0] <= wrapped_angle <= joint_limits_max_deg[0]:
                            solution_degrees[0] = wrapped_angle
                            break
                    else:
                        # 如果包装不起作用，则钳制它
                        clamped_angle = np.clip(solution_degrees[0], joint_limits_min_deg[0], joint_limits_max_deg[0])
                        solution_degrees[0] = clamped_angle
                
                # 正常钳制其他关节
                solution_degrees[1:] = np.clip(solution_degrees[1:], joint_limits_min_deg[1:], joint_limits_max_deg[1:])
                
                # 转换回弧度进行评估
                solution_array = np.deg2rad(solution_degrees)
                
                # 对于当前位姿，不惩罚移动（它是基准）
                if source_name == 'current':
                    error = self._evaluate_ik_solution(solution_array, target_position, None)
                    current_solution_error = error
                    current_solution_joints = solution_array.copy()
                else:
                    # 参考位姿：计算位置误差和带移动惩罚的总误差
                    position_only_error = self._evaluate_ik_solution(solution_array, target_position, None)
                    error = self._evaluate_ik_solution(solution_array, target_position, current_angles_rad)
                    
                    # 基于总误差跟踪最佳参考位姿（用于打破平局）
                    if error < best_reference_error:
                        best_reference_error = error
                        best_reference_solution = solution_array
                        best_reference_source = source_name
                        best_reference_position_error = position_only_error
                
                # 跟踪整体最佳解（用于日志记录）
                if error < best_error:
                    best_error = error
                    best_solution = solution_array
                    best_source = source_name
                
                # 仅对极好的当前解使用提前退出
                if source_name == 'current' and error < 0.0001:  # 0.1mm - 非常严格的阈值
                    break
                    
            except Exception as e:
                print(f"IK failed with {source_name} rest pose: {e}")
                # 即使出错也始终恢复状态
                set_robot_to_current_state()
                continue
        
        # 最终状态恢复以确保一致性
        set_robot_to_current_state()
        
        # 简单逻辑：始终优先使用当前位姿，除非参考位姿显著更好（>5cm）
        final_solution = current_solution_joints  # 默认为当前位姿
        final_error = current_solution_error
        final_source = 'current'
        
        # 检查是否有任何参考位姿明显优于当前位姿
        # 比较纯位置误差（无移动惩罚）以判断 5cm 阈值
        if (best_reference_source is not None and current_actual_error is not None):
            position_improvement = current_actual_error - best_reference_position_error
            if position_improvement > IK_HYSTERESIS_THRESHOLD:  # 位置精度需要提高 5cm
                final_solution = best_reference_solution
                final_error = best_reference_error
                final_source = best_reference_source
        
        if final_solution is not None:
            final_angles = np.rad2deg(final_solution)
            return final_angles
        else:
            print("所有 IK 尝试均失败，返回当前角度")
            return current_angles_deg[:NUM_IK_JOINTS]


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    将 VR 控制器位置转换为机器人坐标系。
    
    VR 坐标系：X=右，Y=上，Z=后（朝向用户）
    机器人坐标系：X=前，Y=左，Z=上
    """
    return np.array([
        -vr_pos['x'] * scale,   # VR +Z（后）-> 机器人 +X（前）
        vr_pos['z'] * scale,    # VR +X（右）-> 机器人 -Y（右） 
        vr_pos['y'] * scale     # VR +Y（上）-> 机器人 +Z（上）
    ])


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """计算从 VR 原点到当前位置的相对位置。"""
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'], 
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    return vr_to_robot_coordinates(delta_vr, scale) 