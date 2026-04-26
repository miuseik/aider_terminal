"""
动作分发器 - 统一分发控制指令到仿真和真机

工作流程:
1. 接收完整的Action字典(包含机械臂/底盘/升降轴)
2. 同时发送到:
   - PyBullet仿真 (visualizer)
   - 真机驱动 (drivers)
3. 确保仿真和真机状态同步
"""

import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class ActionDispatcher:
    """动作分发器 - 双路输出(仿真+真机)"""
    
    def __init__(self, config, visualizer=None):
        """
        初始化动作分发器
        
        Args:
            config: 配置信息
            visualizer: PyBullet可视化器实例
        """
        self.config = config
        self.visualizer = visualizer
        
        # 初始化各个控制器
        from controller.motor_controller import MotorController
        from controller.base_controller import BaseController
        from controller.lift_controller import LiftController
        
        self.motor_controller = MotorController(config)
        self.base_controller = BaseController(config)
        self.lift_controller = LiftController(config)
        
        # 校准管理器
        from controller.calibration_manager import CalibrationManager
        self.calibration_mgr = CalibrationManager(self.motor_controller)
        
        logger.info("✅ ActionDispatcher 初始化完成")
    
    def dispatch_action(self, action_dict: Dict[str, Any]):
        """
        分发完整Action字典到仿真
        
        Args:
            action_dict: 完整的动作字典,包含:
                - left_arm.*.pos: 左臂6个关节角度
                - right_arm.*.pos: 右臂6个关节角度
                - base.*.vel: 底盘3个轮子速度
                - lift.height_mm: 升降轴高度
        """
        # 发送到仿真
        if self.visualizer and self.config.get('enable_pybullet', False):
            self._dispatch_to_simulation(action_dict)
    
    def _dispatch_to_simulation(self, action_dict: Dict[str, Any]):
        """分发到PyBullet仿真"""
        try:
            # 1. 更新机械臂姿态
            for arm in ["left", "right"]:
                angles = []
                for joint in ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                             "wrist_flex", "wrist_roll", "gripper"]:
                    key = f"{arm}_arm.{joint}.pos"
                    if key in action_dict:
                        angles.append(action_dict[key])
                
                if len(angles) == 6:
                    self.visualizer.update_robot_pose(angles, arm)
            
            # 2. 更新底盘位置
            sim_action = {
                "lift.height_mm": action_dict.get("lift.height_mm", 0),
                "base.vx": action_dict.get("base.vx", 0),
                "base.vy": action_dict.get("base.vy", 0),
                "base.vtheta": action_dict.get("base.vtheta", 0),
            }
            self.visualizer.update_mobile_base_simulation(sim_action)
            
            logger.debug("✅ 仿真更新完成")
        except Exception as e:
            logger.error(f"❌ 仿真更新失败: {e}")
    
    def _dispatch_to_hardware(self, action_dict: Dict[str, Any]):
        """
        分发到真机硬件(暂未使用,保留以备将来扩展)
        
        注意: 当前真机发送由 robot_interface.send_command() 负责
        """
        try:
            # 1. 发送机械臂指令
            self._send_arm_commands(action_dict)
            
            # 2. 发送底盘指令
            self._send_base_commands(action_dict)
            
            # 3. 发送升降轴指令
            self._send_lift_commands(action_dict)
            
            logger.debug("✅ 真机指令发送完成")
        except Exception as e:
            logger.error(f"❌ 真机指令发送失败: {e}")
    
    def _send_arm_commands(self, action_dict: Dict[str, Any]):
        """发送机械臂指令到真机"""
        for arm in ["left", "right"]:
            arm_angles = {}
            for joint in ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                         "wrist_flex", "wrist_roll", "gripper"]:
                key = f"{arm}_arm.{joint}.pos"
                if key in action_dict:
                    arm_angles[joint] = action_dict[key]
            
            if arm_angles:
                # 调用motor_controller发送
                self.motor_controller.send_arm_command(arm, arm_angles)
    
    def _send_base_commands(self, action_dict: Dict[str, Any]):
        """发送底盘指令到真机"""
        wheel_speeds = {
            'left': action_dict.get('base.left_wheel.vel', 0),
            'back': action_dict.get('base.back_wheel.vel', 0),
            'right': action_dict.get('base.right_wheel.vel', 0),
        }
        
        # 调用base_controller发送
        self.base_controller.set_wheel_speeds(wheel_speeds)
    
    def _send_lift_commands(self, action_dict: Dict[str, Any]):
        """发送升降轴指令到真机"""
        height_mm = action_dict.get('lift.height_mm', 0)
        
        # 调用lift_controller发送
        self.lift_controller.set_height(height_mm)
    
    # === 校准功能 ===
    
    def calibrate_motor(self, motor_name: str, motor_id: int):
        """校准单个电机"""
        return self.calibration_mgr.calibrate_motor(motor_name, motor_id)
    
    def save_calibration(self, filepath: str = "calibration.json"):
        """保存校准配置"""
        return self.calibration_mgr.save_calibration(filepath)
    
    def load_calibration(self, filepath: str = "calibration.json"):
        """加载校准配置"""
        return self.calibration_mgr.load_calibration(filepath)
    
    # === 传感器读取 ===
    
    def read_motor_sensor(self, arm: str, motor_name: str):
        """读取电机传感器数据"""
        return self.motor_controller.read_sensor_data(arm, motor_name)
    
    # === 摄像头功能 ===
    
    def start_camera_streaming(self):
        """启动摄像头推流"""
        if hasattr(self, 'camera_controller'):
            return self.camera_controller.start_streaming()
        return False
    
    def stop_camera_streaming(self):
        """停止摄像头推流"""
        if hasattr(self, 'camera_controller'):
            return self.camera_controller.stop_streaming()
        return False
    
    def get_camera_status(self):
        """获取摄像头状态"""
        if hasattr(self, 'camera_controller'):
            return self.camera_controller.get_camera_status()
        return {'connected': False, 'streaming': False}
