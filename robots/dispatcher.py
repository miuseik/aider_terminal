"""
Dispatcher - 控制指令分发器
负责将控制指令分发到仿真环境和真机硬件
"""

import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class ControlDispatcher:
    """控制指令分发器"""
    
    def __init__(self, robot_interface=None, visualizer=None):
        """
        初始化分发器
        
        Args:
            robot_interface: 真机机器人接口（可选）
            visualizer: 仿真可视化器（可选）
        """
        self.robot_interface = robot_interface
        self.visualizer = visualizer
    
    def build_alohamini_action(self, 
                               base_velocity: Dict[str, float],
                               aloha_height: float,
                               robot_interface) -> dict:
        """构造发送给 LeRobot/AlohaMini 的完整 Action 字典。"""
        action = {}

        # 1. 机械臂部分 (从 robot_interface 获取最新的 IK 结果)
        if robot_interface:
            for arm in ["left", "right"]:
                angles = robot_interface.get_arm_angles(arm)
                for i, name in enumerate(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]):
                    action[f"{arm}_arm.{name}.pos"] = angles[i]

        # 2. 底盘部分 (调用运动学逆解)
        from telegrip.core.wheels import body_to_wheel_raw
        wheel_speeds = body_to_wheel_raw(
            base_velocity["x"],
            base_velocity["y"],
            base_velocity["theta"]
        )
        action["base.left_wheel.vel"] = wheel_speeds["base_left_wheel"]
        action["base.back_wheel.vel"] = wheel_speeds["base_back_wheel"]
        action["base.right_wheel.vel"] = wheel_speeds["base_right_wheel"]

        # 3. 升降轴部分 (注意：aloha_height 内部单位是米，Action 需要毫米)
        height_mm = int(aloha_height * 1000)
        action["lift.height_mm"] = height_mm

        return action
    
    def update_visualization(self,
                            base_velocity: Dict[str, float],
                            aloha_height: float,
                            robot_interface,
                            vr_raw_data: dict,
                            config,
                            left_arm,
                            right_arm):
        """更新 PyBullet 可视化。"""
        if not self.visualizer:
            return

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
        # 在无机器人模式下,get_arm_angles 返回仿真角度
        left_angles = robot_interface.get_actual_arm_angles("left")
        right_angles = robot_interface.get_actual_arm_angles("right")

        # 【夹爪线性控制】从 VR 数据中提取 trigger 值,替换夹爪角度
        # trigger: 0.0 → -90° (完全打开), 1.0 → 0° (完全闭合)
        from telegrip.config import GRIPPER_INDEX
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
        from telegrip.inputs.base import ControlMode
        if left_arm.mode == ControlMode.POSITION_CONTROL:
            if left_arm.target_position is not None:
                # 显示当前末端执行器位置
                current_pos = robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if left_arm.goal_position is not None:
                # 显示目标位置
                self.visualizer.update_marker_position("left_goal", left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", left_arm.goal_position)
        else:
            # 非位置控制模式时隐藏标记点
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if right_arm.mode == ControlMode.POSITION_CONTROL:
            if right_arm.target_position is not None:
                # 显示当前末端执行器位置
                current_pos = robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if right_arm.goal_position is not None:
                # 显示目标位置
                self.visualizer.update_marker_position("right_goal", right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", right_arm.goal_position)
        else:
            # 非位置控制模式时隐藏标记点
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")

        # 推进仿真
        self.visualizer.step_simulation()
    
    def dispatch_robot_state(self, 
                            base_velocity: Dict[str, float],
                            lift_height_mm: int,
                            left_angles: list,
                            right_angles: list):
        """
        分发机器人状态到仿真和真机
        
        Args:
            base_velocity: 底盘速度 {"x": 0.0, "y": 0.0, "theta": 0.0}
            lift_height_mm: 升降轴高度（毫米）
            left_angles: 左臂关节角度列表
            right_angles: 右臂关节角度列表
        """
        # 1. 发送到真机
        if self.robot_interface:
            self._dispatch_to_hardware(base_velocity, lift_height_mm)
        
        # 2. 发送到仿真
        if self.visualizer:
            self._dispatch_to_simulation(base_velocity, lift_height_mm, left_angles, right_angles)
    
    def _dispatch_to_hardware(self, 
                             base_velocity: Dict[str, float],
                             lift_height_mm: int):
        """
        发送控制指令到真机硬件
        
        Args:
            base_velocity: 底盘速度
            lift_height_mm: 升降轴高度
        """
        try:
            # 更新底盘状态
            if self.robot_interface:
                self.robot_interface.base_velocity_target = base_velocity.copy()
                
                # 更新升降轴状态
                self.robot_interface.lift_height_mm = lift_height_mm
                
                # 发送命令到真机（如果已连接且已使能）
                if self.robot_interface.is_connected and self.robot_interface.is_engaged:
                    self.robot_interface.send_command()
                    
        except Exception as e:
            print(f"发送指令到真机失败: {e}")
    
    def _dispatch_to_simulation(self,
                               base_velocity: Dict[str, float],
                               lift_height_mm: int,
                               left_angles: list,
                               right_angles: list):
        """
        发送控制指令到仿真环境
        
        Args:
            base_velocity: 底盘速度
            lift_height_mm: 升降轴高度
            left_angles: 左臂关节角度
            right_angles: 右臂关节角度
        """
        try:
            # 1. 更新仿真中的底盘位置
            sim_action = {
                "lift.height_mm": lift_height_mm,
                "base.vx": base_velocity["x"],
                "base.vy": base_velocity["y"],
                "base.vtheta": base_velocity["theta"],
            }
            self.visualizer.update_mobile_base_simulation(sim_action)
            
            # 2. 更新机械臂姿态
            self.visualizer.update_robot_pose(left_angles, 'left')
            self.visualizer.update_robot_pose(right_angles, 'right')
            
            # 3. 如果启用了 Aloha，映射到 Aloha 双臂
            if hasattr(self.visualizer, 'aloha_id') and self.visualizer.aloha_id is not None:
                self.visualizer.update_aloha_arm_pose(left_angles, 'left')
                self.visualizer.update_aloha_arm_pose(right_angles, 'right')
            
            # 4. 推进仿真
            self.visualizer.step_simulation()
            
        except Exception as e:
            print(f"发送指令到仿真失败: {e}")
    
    def set_robot_interface(self, robot_interface):
        """设置真机机器人接口"""
        self.robot_interface = robot_interface
    
    def set_visualizer(self, visualizer):
        """设置仿真可视化器"""
        self.visualizer = visualizer
