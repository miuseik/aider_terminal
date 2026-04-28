"""
API命令路由器
处理从Server接收的WebSocket API控制命令
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class APICommandRouter:
    """API命令路由器 - 根据action字段路由到对应的处理方法"""
    
    def __init__(self, motor_controller):
        """
        初始化API命令路由器
        
        Args:
            motor_controller: 电机控制器实例
        """
        self.motor_controller = motor_controller
    
    def route(self, command: Dict[str, Any]) -> bool:
        """
        路由单个API命令到对应的处理器
        
        Args:
            command: 命令字典,包含action和其他参数
            
        Returns:
            bool: 是否成功处理
        """
        action = command.get('action', '')
        
        if not action:
            logger.warning("⚠️ 命令缺少action字段")
            return False
        
        # 路由表: action → 处理方法
        route_map = {
            # === 机械臂控制 ===
            'control_motor': self._route_control_motor,
            'calibrate_motor': self._route_calibrate_motor,
            
            # === 传感器读取 ===
            'read_sensor': self._route_read_sensor,
            
            # === 电机硬件控制 ===
            'set_motor_id': self._route_set_motor_id,
            'edit_motor_id': self._route_set_motor_id,  # 别名，兼容前端调用
            'set_operation_mode': self._route_set_mode,
            'set_velocity': self._route_set_velocity,
            'set_torque': self._route_set_torque,
            
            # === 底盘和升降轴 ===
            'control_chassis': self._route_control_chassis,
            'control_lift': self._route_control_lift,
            
            # === 校准管理 ===
            'save_calibration': self._route_save_calibration,
            'load_calibration': self._route_load_calibration,
        }
        
        handler = route_map.get(action)
        if handler:
            return handler(command)
        else:
            logger.warning(f"⚠️ 未知的命令: {action}")
            return False
    
    def _route_control_motor(self, command: Dict[str, Any]) -> bool:
        """路由电机角度控制命令"""
        arm = command.get('arm')
        motor_name = command.get('motor')
        angle = command.get('angle')
        
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
    
    def _route_calibrate_motor(self, command: Dict[str, Any]) -> bool:
        """路由电机校准命令"""
        arm = command.get('arm')
        motor_name = command.get('motor')
        target_zero = command.get('target_zero', 0.0)
        
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
    
    def _route_control_chassis(self, command: Dict[str, Any]) -> bool:
        """路由底盘控制命令"""
        wheel = command.get('wheel')
        speed = command.get('speed')
        
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
    
    def _route_control_lift(self, command: Dict[str, Any]) -> bool:
        """路由升降轴控制命令"""
        speed = command.get('speed')
        
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
    
    def _route_read_sensor(self, command: Dict[str, Any]) -> bool:
        """路由传感器读取命令"""
        arm = command.get('arm')
        motor_name = command.get('motor')
        
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
            # TODO: 将数据返回给前端
            return True
        else:
            logger.error(f"❌ 读取{motor_name}传感器失败")
            return False
    
    def _route_set_motor_id(self, command: Dict[str, Any]) -> bool:
        """路由设置电机ID命令（纯硬件操作）
        
        支持两种调用方式：
        1. 直接指定串口和舵机类型（推荐）：port, servo_type, old_id, new_id
        2. 通过机械臂信息推断：arm, motor, current_id, new_id
        """
        # 方式1：直接指定串口和舵机类型
        port = command.get('port')
        servo_type = command.get('servo_type')
        old_id = command.get('old_id') or command.get('current_id')
        new_id = command.get('new_id')
        baudrate = command.get('baudrate', 115200)
        
        # 方式2：通过机械臂信息推断（从配置中获取）
        if not port or not servo_type:
            arm = command.get('arm')
            motor_name = command.get('motor')
            
            if arm and motor_name:
                logger.info(f"🔧 通过机械臂信息推断配置: {arm}臂 {motor_name}")
                
                # 从 telegrip 配置中获取串口和舵机类型
                try:
                    from telegrip.config import config, get_config_data
                    config_data = get_config_data()
                    
                    if arm == 'left':
                        port = config.follower_ports.get('left', '/dev/ttyACM0')
                        servo_type = config_data.get('robot', {}).get('left_arm', {}).get('servo_type', 'st3215')
                        baudrate = config_data.get('robot', {}).get('left_arm', {}).get('baudrate', 1000000)
                    elif arm == 'right':
                        port = config.follower_ports.get('right', '/dev/ttyACM1')
                        servo_type = config_data.get('robot', {}).get('right_arm', {}).get('servo_type', 'st3215')
                        baudrate = config_data.get('robot', {}).get('right_arm', {}).get('baudrate', 1000000)
                    else:
                        logger.error(f"❌ 无效的机械臂: {arm}")
                        return False
                    
                    logger.info(f"✅ 配置推断成功: port={port}, servo_type={servo_type}, baudrate={baudrate}")
                    
                except Exception as e:
                    logger.error(f"❌ 配置推断失败: {e}")
                    import traceback
                    logger.error(traceback.format_exc())
                    return False
            else:
                logger.error("❌ set_motor_id 命令缺少必要参数")
                logger.error(f"   方式1需要: port, servo_type, old_id/current_id, new_id")
                logger.error(f"   方式2需要: arm, motor, current_id, new_id")
                logger.error(f"   收到: {command}")
                return False
        
        if not all([port, servo_type, old_id is not None, new_id is not None]):
            logger.error("❌ set_motor_id 命令缺少必要参数")
            logger.error(f"   需要: port, servo_type, old_id/current_id, new_id")
            logger.error(f"   收到: {command}")
            return False
        
        logger.info(f"🔧 设置电机ID: {port} ({servo_type}) ID {old_id} → {new_id}")
        
        # 调用 motor_controller 的设置ID方法（纯硬件操作）
        if hasattr(self.motor_controller, 'set_motor_id'):
            success = self.motor_controller.set_motor_id(
                port=port,
                servo_type=servo_type,
                old_id=old_id,
                new_id=new_id,
                baudrate=baudrate
            )
            if success:
                logger.info(f"✅ 电机ID设置成功: {port} {old_id} → {new_id}")
            else:
                logger.error(f"❌ 电机ID设置失败: {port} {old_id} → {new_id}")
            return success
        else:
            logger.error("❌ motor_controller 没有 set_motor_id 方法")
            return False
    
    def _route_set_mode(self, command: Dict[str, Any]) -> bool:
        """路由设置电机模式命令"""
        motor_id = command.get('motor_id')
        mode = command.get('mode')  # 'position' | 'velocity' | 'torque'
        
        if not all([motor_id is not None, mode]):
            logger.error("❌ set_operation_mode 命令缺少必要参数")
            return False
        
        valid_modes = ['position', 'velocity', 'torque']
        if mode not in valid_modes:
            logger.error(f"❌ 无效的模式: {mode}, 可选: {valid_modes}")
            return False
        
        logger.info(f"🔧 设置电机{motor_id} 模式: {mode}")
        
        # TODO: 调用底层驱动设置模式
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_operation_mode(motor_id, mode)
        
        logger.warning("⚠️ 设置电机模式功能待实现")
        return False
    
    def _route_set_velocity(self, command: Dict[str, Any]) -> bool:
        """路由设置电机转速命令"""
        motor_id = command.get('motor_id')
        velocity = command.get('velocity')  # rpm
        
        if not all([motor_id is not None, velocity is not None]):
            logger.error("❌ set_velocity 命令缺少必要参数")
            return False
        
        logger.info(f"🔄 设置电机{motor_id} 转速: {velocity} rpm")
        
        # TODO: 调用底层驱动设置转速
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_velocity(motor_id, velocity)
        
        logger.warning("⚠️ 设置电机转速功能待实现")
        return False
    
    def _route_set_torque(self, command: Dict[str, Any]) -> bool:
        """路由设置电机力矩命令"""
        motor_id = command.get('motor_id')
        torque = command.get('torque')  # 0-100%
        
        if not all([motor_id is not None, torque is not None]):
            logger.error("❌ set_torque 命令缺少必要参数")
            return False
        
        logger.info(f"⚡ 设置电机{motor_id} 力矩: {torque}%")
        
        # TODO: 调用底层驱动设置力矩
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_torque(motor_id, torque)
        
        logger.warning("⚠️ 设置电机力矩功能待实现")
        return False
    
    def _route_save_calibration(self, command: Dict[str, Any]) -> bool:
        """路由保存校准配置命令"""
        filepath = command.get('filepath', 'calibration.json')
        
        logger.info(f"💾 保存校准配置: {filepath}")
        
        # TODO: 实现校准配置保存功能
        logger.warning("⚠️ 校准配置保存功能待实现")
        return False
    
    def _route_load_calibration(self, command: Dict[str, Any]) -> bool:
        """路由加载校准配置命令"""
        filepath = command.get('filepath', 'calibration.json')
        
        logger.info(f"📂 加载校准配置: {filepath}")
        
        # TODO: 实现校准配置加载功能
        logger.warning("⚠️ 校准配置加载功能待实现")
        return False
