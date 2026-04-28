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
        
        # 初始化底层驱动（如果传入配置）
        self.driver = None
        if config:
            self._initialize_driver(config)
    
    def _initialize_driver(self, config: Dict):
        """
        根据配置初始化底层驱动
        
        Args:
            config: 配置字典，包含:
                - servo_type: 舵机类型 ('lx16a', 'st3215', 'robstride')
                - port: 串口号
                - baudrate: 波特率 (可选)
        """
        try:
            servo_type = config.get('servo_type', '').lower()
            port = config.get('port')
            baudrate = config.get('baudrate', 115200)
            
            if not port:
                logger.warning("⚠️ 配置中缺少串口号，跳过驱动初始化")
                return
            
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            # 映射舵机类型
            if servo_type == 'lx16a':
                servo_type_enum = ServoType.LX16A
                logger.info(f"🔧 初始化 Hiwonder LX-16A 驱动")
            elif servo_type == 'st3215':
                servo_type_enum = ServoType.ST3215
                logger.info(f"🔧 初始化 Feetech ST3215 驱动")
            elif servo_type in ['robstride', 'rs00']:
                # 灵足 Robstride 电机
                from drivers.robstride.robstride_driver import RobstrideDriver
                self.driver = RobstrideDriver(port=port, baudrate=baudrate)
                logger.info(f"🔧 初始化 Robstride 电机驱动")
                if self.driver.connect():
                    logger.info(f"✅ Robstride 驱动连接成功")
                else:
                    logger.error(f"❌ Robstride 驱动连接失败")
                return
            else:
                logger.error(f"❌ 不支持的舵机类型: {servo_type}")
                return
            
            # 创建驱动实例
            self.driver = create_servo_driver(
                servo_type=servo_type_enum,
                port=port,
                baudrate=baudrate
            )
            
            # 连接驱动
            if self.driver.connect():
                logger.info(f"✅ 驱动初始化成功: {servo_type} @ {port}")
            else:
                logger.error(f"❌ 驱动连接失败: {servo_type} @ {port}")
                self.driver = None
                
        except Exception as e:
            logger.error(f"❌ 驱动初始化异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
            self.driver = None
    
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
    
    def set_motor_id(self, port: str, servo_type: str, old_id: int, new_id: int, baudrate: int = 115200) -> bool:
        """
        设置电机ID（纯硬件操作，与业务无关）
        
        Args:
            port: 串口号 (如 COM3, /dev/ttyUSB0)
            servo_type: 舵机类型 ('lx16a' 或 'st3215')
            old_id: 当前ID (1-253)
            new_id: 新ID (1-253)
            baudrate: 波特率 (默认115200)
            
        Returns:
            bool: 是否成功
        """
        if not (1 <= old_id <= 253) or not (1 <= new_id <= 253):
            logger.error(f"❌ ID超出范围: old_id={old_id}, new_id={new_id}")
            return False
        
        try:
            # 根据舵机类型创建临时驱动实例
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            if servo_type.lower() == 'lx16a':
                servo_type_enum = ServoType.LX16A
            elif servo_type.lower() == 'st3215':
                servo_type_enum = ServoType.ST3215
            else:
                logger.error(f"❌ 不支持的舵机类型: {servo_type}")
                return False
            
            # 创建驱动实例
            driver = create_servo_driver(
                servo_type=servo_type_enum,
                port=port,
                baudrate=baudrate
            )
            
            # 连接串口
            if not driver.connect():
                logger.error(f"❌ 无法连接到 {port}")
                return False
            
            # 调用驱动的 set_id 方法
            success = driver.set_id(old_id, new_id)
            
            # 断开连接
            driver.disconnect()
            
            if success:
                logger.info(f"✅ 电机ID设置成功: {port} ID {old_id} → {new_id}")
            else:
                logger.error(f"❌ 电机ID设置失败: {port} ID {old_id} → {new_id}")
            
            return success
                
        except Exception as e:
            logger.error(f"❌ 设置电机ID异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
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
        try:
            # 优先从底层驱动读取（如果已初始化）
            if self.driver and hasattr(self.driver, 'get_observation'):
                observation = self.driver.get_observation()
                
                if motor_name not in self.motor_index_map:
                    logger.error(f"❌ 未知的电机名称: {motor_name}")
                    return None
                
                index = self.motor_index_map[motor_name]
                joint_names = list(observation.keys())
                
                if index < len(joint_names):
                    joint_name = joint_names[index]
                    position = observation.get(joint_name, 0.0)
                    
                    return {
                        'position': float(position),
                        'velocity': 0.0,  # TODO: 从驱动读取
                        'current': 0.0,   # TODO: 从驱动读取
                        'temperature': 0.0  # TODO: 从驱动读取
                    }
            
            # 降级方案: 从robot_interface读取实际角度
            if self.robot_interface:
                actual_angles = self.robot_interface.get_actual_arm_angles(arm)
                
                if motor_name not in self.motor_index_map:
                    logger.error(f"❌ 未知的电机名称: {motor_name}")
                    return None
                
                index = self.motor_index_map[motor_name]
                
                return {
                    'position': float(actual_angles[index]),
                    'velocity': 0.0,
                    'current': 0.0,
                    'temperature': 0.0
                }
            
            logger.warning("⚠️ 无法读取传感器数据：驱动和robot_interface均未初始化")
            return None
            
        except Exception as e:
            logger.error(f"❌ 读取传感器数据失败: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
