"""
SO100 机器人的运动学工具。
包含使用 PyBullet 的正向和逆向运动学求解器。
"""

import math
import numpy as np
from typing import Optional, Tuple, List
import logging
import json
import os
from pathlib import Path

import src.config.settings as _settings

import pybullet as p

logger = logging.getLogger(__name__)

class ForwardKinematics:
    """使用 PyBullet 的正向运动学求解器。"""

    def __init__(self, physics_client, robot_id: int, joint_indices: list, end_effector_link_index: int):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index

    def compute(self, joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.physics_client is None or self.robot_id is None:
            return np.array([0.2, 0.0, 0.15]), np.array([0, 0, 0, 1])

        fk_state_angles = joint_angles_deg.copy()
        fk_state_angles[5] = 0.0

        joint_angles_rad = np.deg2rad(fk_state_angles)
        for i in range(_settings.NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_angles_rad[i])

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

        self.ik_lower_limits = np.deg2rad(joint_limits_min_deg[:_settings.NUM_IK_JOINTS])
        self.ik_upper_limits = np.deg2rad(joint_limits_max_deg[:_settings.NUM_IK_JOINTS])
        self.ik_ranges = self.ik_upper_limits - self.ik_lower_limits

        self.reference_poses = self._load_reference_poses()
        self.fk_solver = ForwardKinematics(physics_client, robot_id, joint_indices, end_effector_link_index)

    def _load_reference_poses(self) -> List[np.ndarray]:
        reference_poses = []

        if not _settings.USE_REFERENCE_POSES:
            print("Reference poses disabled in configuration")
            return reference_poses

        try:
            from src.utils.common_utils import get_absolute_path
            cache_file = get_absolute_path(_settings.REFERENCE_POSES_FILE)
            if cache_file.exists():
                with open(cache_file, 'r') as f:
                    data = json.load(f)

                arm_poses = data.get(self.arm_name, [])
                if arm_poses:
                    for pose in arm_poses:
                        pose_array = np.array(pose[:_settings.NUM_IK_JOINTS])
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
        try:
            full_angles = np.zeros(_settings.NUM_JOINTS)
            full_angles[:_settings.NUM_IK_JOINTS] = np.rad2deg(solution)
            achieved_position, _ = self.fk_solver.compute(full_angles)
            position_error = np.linalg.norm(achieved_position - target_position)

            movement_penalty = 0.0
            if current_joints_rad is not None:
                joint_diff = solution - current_joints_rad[:_settings.NUM_IK_JOINTS]
                joint_movement = np.linalg.norm(joint_diff)
                movement_penalty = joint_movement * _settings.IK_MOVEMENT_PENALTY_WEIGHT

            total_cost = position_error + movement_penalty
            return total_cost

        except Exception as e:
            print(f"Error evaluating IK solution: {e}")
            return float('inf')

    def solve(self, target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray],
              current_angles_deg: np.ndarray) -> np.ndarray:
        if self.physics_client is None or self.robot_id is None:
            return current_angles_deg[:_settings.NUM_IK_JOINTS]

        current_actual_position, _ = self.fk_solver.compute(current_angles_deg)
        current_actual_error = np.linalg.norm(current_actual_position - target_position)

        ik_state_angles = current_angles_deg.copy()
        ik_state_angles[5] = 0.0
        current_angles_rad = np.deg2rad(ik_state_angles)

        def set_robot_to_current_state():
            for i in range(_settings.NUM_JOINTS):
                if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                    p.resetJointState(self.robot_id, self.joint_indices[i], current_angles_rad[i])

        def set_robot_to_reference_state(ref_pose_rad: np.ndarray):
            full_ref_state = current_angles_rad.copy()
            full_ref_state[:_settings.NUM_IK_JOINTS] = ref_pose_rad
            for i in range(_settings.NUM_JOINTS):
                if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                    p.resetJointState(self.robot_id, self.joint_indices[i], full_ref_state[i])

        rest_poses_to_try = []
        current_rest_pose = np.deg2rad(current_angles_deg[:_settings.NUM_IK_JOINTS])
        rest_poses_to_try.append(('current', current_rest_pose))

        for i, ref_pose in enumerate(self.reference_poses):
            rest_poses_to_try.append((f'reference_{i+1}', ref_pose))

        best_solution = None
        best_error = float('inf')
        best_source = None
        current_solution_error = None
        current_solution_joints = None

        best_reference_solution = None
        best_reference_error = float('inf')
        best_reference_source = None
        best_reference_position_error = float('inf')

        for source_name, rest_pose in rest_poses_to_try:
            try:
                if source_name == 'current':
                    set_robot_to_current_state()
                else:
                    set_robot_to_reference_state(rest_pose)

                ik_solution = p.calculateInverseKinematics(
                    bodyUniqueId=self.robot_id,
                    endEffectorLinkIndex=self.end_effector_link_index,
                    targetPosition=target_position.tolist(),
                    lowerLimits=self.ik_lower_limits.tolist(),
                    upperLimits=self.ik_upper_limits.tolist(),
                    jointRanges=self.ik_ranges.tolist(),
                    restPoses=rest_pose.tolist() if isinstance(rest_pose, np.ndarray) else rest_pose,
                    solver=0,
                    maxNumIterations=100,
                    residualThreshold=1e-4
                )

                set_robot_to_current_state()

                solution_array = np.array(ik_solution[:_settings.NUM_IK_JOINTS])

                joint_limits_min_deg = np.rad2deg(self.ik_lower_limits)
                joint_limits_max_deg = np.rad2deg(self.ik_upper_limits)
                solution_degrees = np.rad2deg(solution_array)

                if solution_degrees[0] < joint_limits_min_deg[0] or solution_degrees[0] > joint_limits_max_deg[0]:
                    for offset in [-360.0, 360.0]:
                        wrapped_angle = solution_degrees[0] + offset
                        if joint_limits_min_deg[0] <= wrapped_angle <= joint_limits_max_deg[0]:
                            solution_degrees[0] = wrapped_angle
                            break
                    else:
                        clamped_angle = np.clip(solution_degrees[0], joint_limits_min_deg[0], joint_limits_max_deg[0])
                        solution_degrees[0] = clamped_angle

                solution_degrees[1:] = np.clip(solution_degrees[1:], joint_limits_min_deg[1:], joint_limits_max_deg[1:])
                solution_array = np.deg2rad(solution_degrees)

                if source_name == 'current':
                    error = self._evaluate_ik_solution(solution_array, target_position, None)
                    current_solution_error = error
                    current_solution_joints = solution_array.copy()
                else:
                    position_only_error = self._evaluate_ik_solution(solution_array, target_position, None)
                    error = self._evaluate_ik_solution(solution_array, target_position, current_angles_rad)

                    if error < best_reference_error:
                        best_reference_error = error
                        best_reference_solution = solution_array
                        best_reference_source = source_name
                        best_reference_position_error = position_only_error

                if error < best_error:
                    best_error = error
                    best_solution = solution_array
                    best_source = source_name

                if source_name == 'current' and error < 0.0001:
                    break

            except Exception as e:
                print(f"IK failed with {source_name} rest pose: {e}")
                set_robot_to_current_state()
                continue

        set_robot_to_current_state()

        final_solution = current_solution_joints
        final_error = current_solution_error
        final_source = 'current'

        if (best_reference_source is not None and current_actual_error is not None):
            position_improvement = current_actual_error - best_reference_position_error
            if position_improvement > _settings.IK_HYSTERESIS_THRESHOLD:
                final_solution = best_reference_solution
                final_error = best_reference_error
                final_source = best_reference_source

        if final_solution is not None:
            final_angles = np.rad2deg(final_solution)
            return final_angles
        else:
            print("所有 IK 尝试均失败，返回当前角度")
            return current_angles_deg[:_settings.NUM_IK_JOINTS]
