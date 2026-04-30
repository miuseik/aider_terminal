"""
电机控制处理器 - 处理机械臂、底盘等电机控制
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class MotorControlHandler:
    """电机控制处理器"""
    
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
    
    def control_motor(self, arm: str, motor_name: str, angle: float) -> bool:
        """控制电机角度"""
        if not all([arm, motor_name, angle is not None]):
            logger.error("❌ control_motor 命令缺少必要参数")
            return False
        
        logger.info(f"🦾 控制电机: {arm}臂, {motor_name}, 角度={angle}°")
        
        if not self.motor_controller:
            logger.warning("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.control_motor(arm, motor_name, float(angle))
        if not success:
            logger.error(f"❌ 控制电机{motor_name}失败")
        
        return success
    
    def calibrate_motor(self, arm: str, motor_name: str, target_zero: float = 0.0) -> bool:
        """校准电机"""
        if not all([arm, motor_name]):
            logger.error("❌ calibrate_motor 命令缺少必要参数")
            return False
        
        logger.info(f"🎯 校准电机: {arm}臂, {motor_name}, 目标零点={target_zero}°")
        
        if not self.motor_controller:
            logger.warning("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.calibrate_motor(arm, motor_name, float(target_zero))
        if not success:
            logger.error(f"❌ 校准电机{motor_name}失败")
        
        return success
    
    def control_chassis(self, wheel: str, speed: float) -> bool:
        """控制底盘"""
        if not all([wheel, speed is not None]):
            logger.error("❌ control_chassis 命令缺少必要参数")
            return False
        
        logger.info(f"🚗 控制底盘: {wheel}轮, 速度={speed}%")
        
        if not self.motor_controller:
            logger.warning("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.control_chassis(wheel, float(speed))
        if not success:
            logger.error(f"❌ 控制{wheel}轮失败")
        
        return success
    
    def control_lift(self, speed: float) -> bool:
        """控制升降轴"""
        if speed is None:
            logger.error("❌ control_lift 命令缺少speed参数")
            return False
        
        logger.info(f"⬆️ 控制升降轴: 速度={speed}%")
        
        if not self.motor_controller:
            logger.warning("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.control_lift(float(speed))
        if not success:
            logger.error(f"❌ 控制升降轴失败")
        
        return success
    
    def read_sensor(self, arm: str, motor_name: str) -> bool:
        """读取传感器"""
        if not all([arm, motor_name]):
            logger.error("❌ read_sensor 命令缺少必要参数")
            return False
        
        logger.info(f"📖 读取传感器: {arm}臂, {motor_name}")
        
        if not self.motor_controller:
            logger.warning("⚠️ 电机控制器未初始化")
            return False
        
        sensor_data = self.motor_controller.read_sensor_data(arm, motor_name)
        if sensor_data:
            logger.info(f"✅ 传感器数据: {sensor_data}")
            return True
        else:
            logger.error(f"❌ 读取{motor_name}传感器失败")
            return False
