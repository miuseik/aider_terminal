"""
电机控制器 - 管理机械臂电机控制

职责:
1. 发送机械臂角度指令
2. 读取电机传感器数据(角度/转速/电流/温度)
3. 电机ID设置、模式切换等硬件控制
4. 与底层驱动(Damiao/Feetech)交互
"""

import logging
import numpy as np
from typing import Optional, Dict

logger = logging.getLogger(__name__)


class MotorController:
    """电机控制器"""
    
    def __init__(self, config=None, robot_interface=None):
        """
        初始化电机控制器
        
        Args:
            config: 配置信息(可选)
            robot_interface: 机器人接口实例(兼容旧代码)
        """
        self.config = config
        self.robot_interface = robot_interface  # 兼容旧代码
        
        # 电机名称到索引的映射
        self.motor_index_map = {
            'shoulder_pan': 0,
            'shoulder_lift': 1,
            'elbow_flex': 2,
            'wrist_flex': 3,
            'wrist_roll': 4,
            'gripper': 5
        }
        
        # TODO: 如果传入config,初始化底层驱动
        # if config:
        #     from ..drivers import DamiaoDriver, FeetechDriver
        #     if config.get('motor_type') == 'damiao':
        #         self.driver = DamiaoDriver(config)
        #     elif config.get('motor_type') == 'feetech':
        #         self.driver = FeetechDriver(config)
    
    def control_motor(self, arm: str, motor_name: str, angle: float) -> bool:
        """
        控制单个电机角度
        
        Args:
            arm: 'left' 或 'right'
            motor_name: 电机名称 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
            angle: 目标角度(度)
            
        Returns:
            bool: 是否成功
        """
        if not self.robot_interface or not self.robot_interface.is_connected or not self.robot_interface.is_engaged:
            logger.warning("⚠️ 机器人未连接或未使能")
            return False
        
        try:
            if motor_name not in self.motor_index_map:
                logger.error(f"❌ 未知的电机名称: {motor_name}")
                return False
            
            index = self.motor_index_map[motor_name]
            
            # 更新对应机械臂的角度
            if arm == 'left' and self.robot_interface.left_robot and self.robot_interface.left_arm_connected:
                self.robot_interface.left_arm_angles[index] = angle
                logger.info(f"✅ 左臂 {motor_name} 设置为 {angle}°")
                # 注意: 不立即发送指令,由control_loop统一发送
                return True
            elif arm == 'right' and self.robot_interface.right_robot and self.robot_interface.right_arm_connected:
                self.robot_interface.right_arm_angles[index] = angle
                logger.info(f"✅ 右臂 {motor_name} 设置为 {angle}°")
                # 注意: 不立即发送指令,由control_loop统一发送
                return True
            else:
                logger.error(f"❌ {arm} 臂未连接")
                return False
        except Exception as e:
            logger.error(f"❌ 控制电机异常: {e}")
            return False
    
    def calibrate_motor(self, arm: str, motor_name: str, target_zero: float = 0.0) -> bool:
        """
        校准电机零点
        
        Args:
            arm: 'left' 或 'right'
            motor_name: 电机名称
            target_zero: 目标零点位置(默认0.0)
            
        Returns:
            bool: 是否成功
        """
        if not self.robot_interface or not self.robot_interface.is_connected or not self.robot_interface.is_engaged:
            logger.warning("⚠️ 机器人未连接或未使能")
            return False
        
        try:
            # 获取当前实际位置作为零点偏移
            current_angles = self.robot_interface.get_actual_arm_angles(arm)
            
            if motor_name not in self.motor_index_map:
                logger.error(f"❌ 未知的电机名称: {motor_name}")
                return False
            
            index = self.motor_index_map[motor_name]
            current_position = current_angles[index]
            
            # TODO: 这里需要调用底层的校准API来设置homing_offset
            # 目前只是记录日志，实际需要修改电机配置
            logger.info(f"🎯 校准 {arm} 臂 {motor_name}: 当前位置={current_position}°, 目标零点={target_zero}°")
            logger.info(f"⚠️ 注意: 需要在电机固件层面实现零点校准功能")
            
            # 临时方案: 将当前角度设置为目标零点
            if arm == 'left' and self.robot_interface.left_robot and self.robot_interface.left_arm_connected:
                self.robot_interface.left_arm_angles[index] = target_zero
                logger.info(f"✅ 左臂 {motor_name} 零点已设置为 {target_zero}°")
                return True
            elif arm == 'right' and self.robot_interface.right_robot and self.robot_interface.right_arm_connected:
                self.robot_interface.right_arm_angles[index] = target_zero
                logger.info(f"✅ 右臂 {motor_name} 零点已设置为 {target_zero}°")
                return True
            else:
                logger.error(f"❌ {arm} 臂未连接")
                return False
        except Exception as e:
            logger.error(f"❌ 校准电机异常: {e}")
            return False
    
    def send_arm_command(self, arm: str, angles: Dict[str, float]) -> bool:
        """
        发送机械臂角度指令到真机
        
        Args:
            arm: 'left' 或 'right'
            angles: {joint_name: angle_deg}
            
        Returns:
            bool: 是否成功
        """
        if not self.robot_interface or not self.robot_interface.is_connected:
            logger.warning("⚠️ 机器人未连接")
            return False
        
        try:
            # 更新robot_interface的角度数组
            for joint_name, angle in angles.items():
                if joint_name in self.motor_index_map:
                    index = self.motor_index_map[joint_name]
                    if arm == 'left':
                        self.robot_interface.left_arm_angles[index] = angle
                    elif arm == 'right':
                        self.robot_interface.right_arm_angles[index] = angle
            
            logger.debug(f"📤 {arm}臂角度指令已准备: {angles}")
            return True
        except Exception as e:
            logger.error(f"❌ 发送{arm}臂指令失败: {e}")
            return False
    
    def read_sensor_data(self, arm: str, motor_name: str) -> Optional[Dict[str, float]]:
        """
        读取电机传感器数据
        
        Args:
            arm: 'left' 或 'right'
            motor_name: 电机名称
            
        Returns:
            dict: {
                'position': 角度(度),
                'velocity': 转速(rpm),
                'current': 电流(A),
                'temperature': 温度(°C)
            }, 失败返回None
        """
        if not self.robot_interface:
            return None
        
        try:
            # TODO: 如果有底层驱动,从驱动读取
            # if hasattr(self, 'driver'):
            #     motor_id = self._get_motor_id(arm, motor_name)
            #     return self.driver.read_all_sensors(motor_id)
            
            # 当前方案: 从robot_interface读取实际角度
            actual_angles = self.robot_interface.get_actual_arm_angles(arm)
            
            if motor_name not in self.motor_index_map:
                logger.error(f"❌ 未知的电机名称: {motor_name}")
                return None
            
            index = self.motor_index_map[motor_name]
            
            return {
                'position': float(actual_angles[index]),
                'velocity': 0.0,  # TODO: 从驱动读取
                'current': 0.0,   # TODO: 从驱动读取
                'temperature': 0.0  # TODO: 从驱动读取
            }
        except Exception as e:
            logger.error(f"❌ 读取传感器数据失败: {e}")
            return None
