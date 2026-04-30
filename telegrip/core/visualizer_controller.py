"""
VisualizerController - 仿真可视化控制器
负责管理 PyBullet 仿真环境的更新
"""

import logging
from typing import Dict

logger = logging.getLogger(__name__)


class VisualizerController:
    """仿真可视化控制器"""
    
    def __init__(self, visualizer=None):
        """
        初始化仿真控制器
        
        Args:
            visualizer: PyBullet 可视化器实例
        """
        self.visualizer = visualizer
    
    def update(self,
               base_velocity: Dict[str, float],
               aloha_height: float,
               robot_interface,
               vr_raw_data: dict,
               config,
               left_arm,
               right_arm):
        """
        更新仿真可视化
        
        Args:
            base_velocity: 底盘速度 {"x": 0.0, "y": 0.0, "theta": 0.0}
            aloha_height: 升降轴高度（米）
            robot_interface: 机器人接口
            vr_raw_data: VR 原始数据
            config: 配置对象
            left_arm: 左臂状态
            right_arm: 右臂状态
        """
        if not self.visualizer:
            return
        
        from telegrip.config import GRIPPER_INDEX
        from telegrip.inputs.base import ControlMode
        
        # 1. 更新仿真中的底盘位置
        if config.aloha_enabled:
            sim_action = {
                "lift.height_mm": int(aloha_height * 1000),
                "base.vx": base_velocity["x"],
                "base.vy": base_velocity["y"],
                "base.vtheta": base_velocity["theta"],
            }
            self.visualizer.update_mobile_base_simulation(sim_action)
        
        # 2. 使用机器人硬件的实际角度更新两个机械臂的姿态
        left_angles = robot_interface.get_actual_arm_angles("left")
        right_angles = robot_interface.get_actual_arm_angles("right")
        
        # 【夹爪线性控制】从 VR 数据中提取 trigger 值,替换夹爪角度
        left_trigger = vr_raw_data.get('leftController', {}).get('trigger', None)
        right_trigger = vr_raw_data.get('rightController', {}).get('trigger', None)
        
        if left_trigger is not None and len(left_angles) > GRIPPER_INDEX:
            left_angles[GRIPPER_INDEX] = -left_trigger * 90.0
        
        if right_trigger is not None and len(right_angles) > GRIPPER_INDEX:
            right_angles[GRIPPER_INDEX] = -right_trigger * 90.0
        
        # 更新 SO100 机器人姿态
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
        # 如果启用了 Aloha,将 SO100 IK 结果映射到 Aloha 双臂
        if config.aloha_enabled and self.visualizer.aloha_id is not None:
            self.visualizer.update_aloha_arm_pose(left_angles, 'left')
            self.visualizer.update_aloha_arm_pose(right_angles, 'right')
        
        # 更新可视化标记点
        if left_arm.mode == ControlMode.POSITION_CONTROL:
            if left_arm.target_position is not None:
                current_pos = robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if left_arm.goal_position is not None:
                self.visualizer.update_marker_position("left_goal", left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", left_arm.goal_position)
        else:
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if right_arm.mode == ControlMode.POSITION_CONTROL:
            if right_arm.target_position is not None:
                current_pos = robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if right_arm.goal_position is not None:
                self.visualizer.update_marker_position("right_goal", right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", right_arm.goal_position)
        else:
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # 推进仿真
        self.visualizer.step_simulation()
    
    def set_visualizer(self, visualizer):
        """设置可视化器"""
        self.visualizer = visualizer
