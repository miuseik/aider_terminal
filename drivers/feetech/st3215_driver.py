"""
飞特 Feetech ST3215 总线舵机驱动
实现核心功能：
1. 设置ID
2. 设置模式（位置模式/速度模式）
3. 根据模式控制转速或角度
4. 读取舵机状态数据

基于官方 feetech-servo-sdk
"""

import time
import logging
from typing import Dict, Optional
from enum import Enum

# 导入飞特SDK
import scservo_sdk

logger = logging.getLogger(__name__)


class ServoMode(Enum):
    """舵机工作模式"""
    POSITION = 0  # 位置模式 (使用位置环)
    SPEED = 1     # 速度模式 (使用速度环)


class ST3215Driver:
    """
    ST3215 总线舵机驱动
    
    技术规格：
    - 位置范围：0-4095 对应 0-360度
    - 速度范围：-1023 ~ 1023
    - 默认波特率：1000000 (1M)
    
    寄存器地址：
    - GOAL_POSITION: 116 (2字节)
    - PRESENT_POSITION: 132 (2字节)
    - GOAL_SPEED: 120 (2字节)
    - PRESENT_SPEED: 136 (2字节)
    - MODE: 11 (1字节, 0=位置, 1=速度)
    - ID: 5 (1字节)
    - TEMPERATURE: 140 (1字节)
    - VOLTAGE: 142 (2字节)
    """
    
    # 寄存器地址
    ADDR_ID = 5
    ADDR_MODE = 11
    ADDR_GOAL_POSITION = 116
    ADDR_GOAL_SPEED = 120
    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_SPEED = 136
    ADDR_TEMPERATURE = 140
    ADDR_VOLTAGE = 142
    
    # 位置范围
    PULSE_MIN = 0
    PULSE_MAX = 4095
    ANGLE_RANGE = 360.0
    
    # 速度范围
    SPEED_MIN = -1023
    SPEED_MAX = 1023
    
    def __init__(self, port: str, baudrate: int = 1000000):
        """
        初始化驱动
        
        :param port: 串口号 (如 COM3, /dev/ttyUSB0)
        :param baudrate: 波特率 (默认1000000)
        """
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.current_id = 1
        
    def connect(self) -> bool:
        """连接并初始化SDK"""
        try:
            # 创建端口处理器
            self.port_handler = scservo_sdk.PortHandler(self.port)
            # ST3215 使用协议版本 0 (旧协议)
            self.packet_handler = scservo_sdk.PacketHandler(0)
            
            # 打开端口
            if not self.port_handler.openPort():
                logger.error(f"❌ 无法打开端口 {self.port}")
                return False
            
            # 设置波特率
            if not self.port_handler.setBaudRate(self.baudrate):
                logger.error(f"❌ 无法设置波特率 {self.baudrate}")
                return False
            
            logger.info(f"✅ ST3215 连接到 {self.port} @ {self.baudrate}")
            return True
            
        except Exception as e:
            logger.error(f"❌ ST3215 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.port_handler:
            self.port_handler.closePort()
            logger.info("🔌 ST3215 已断开")
    
    def _check_comm_result(self, comm_result: int, dxl_error: int, 
                          operation: str, servo_id: int) -> bool:
        """
        检查通信结果
        
        :param comm_result: 通信结果
        :param dxl_error: 错误码
        :param operation: 操作描述
        :param servo_id: 舵机ID
        :return: 是否成功
        """
        if comm_result != scservo_sdk.COMM_SUCCESS:
            logger.error(f"❌ {operation} 通信失败: {self.packet_handler.getTxRxResult(comm_result)}")
            return False
        
        if dxl_error != 0:
            logger.error(f"❌ {operation} 舵机错误: {self.packet_handler.getRxPacketError(dxl_error)}")
            return False
        
        return True
    
    # ==================== 功能1: 设置ID ====================
    
    def set_id(self, old_id: int, new_id: int) -> bool:
        """
        设置舵机ID
        
        :param old_id: 当前ID
        :param new_id: 新ID (1-253)
        :return: 是否成功
        """
        if not (1 <= new_id <= 253):
            logger.error(f"ID超出范围: {new_id}")
            return False
        
        # 写入新ID
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            old_id,
            self.ADDR_ID,
            new_id
        )
        
        if self._check_comm_result(comm_result, dxl_error, "设置ID", old_id):
            logger.info(f"✅ 舵机ID已从 {old_id} 设置为 {new_id}")
            self.current_id = new_id
            return True
        else:
            return False
    
    # ==================== 功能2: 设置模式 ====================
    
    def set_mode(self, servo_id: int, mode: ServoMode) -> bool:
        """
        设置舵机工作模式
        
        :param servo_id: 舵机ID
        :param mode: 工作模式 (POSITION 或 SPEED)
        :return: 是否成功
        """
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_MODE,
            mode.value
        )
        
        if self._check_comm_result(comm_result, dxl_error, "设置模式", servo_id):
            mode_name = "位置模式" if mode == ServoMode.POSITION else "速度模式"
            logger.info(f"✅ 舵机 {servo_id} 已设置为 {mode_name}")
            return True
        else:
            return False
    
    # ==================== 功能3: 控制转速或角度 ====================
    
    def set_position(self, servo_id: int, angle: float, time_ms: int = 0) -> bool:
        """
        设置目标角度（位置模式）
        
        :param servo_id: 舵机ID
        :param angle: 目标角度 (0-360度)
        :param time_ms: 运动时间(毫秒, 0表示最大速度)
        :return: 是否成功
        """
        # 角度限制
        angle = max(0, min(360, angle))
        
        # 角度转脉冲值
        pulse = int((angle / self.ANGLE_RANGE) * self.PULSE_MAX)
        pulse = max(self.PULSE_MIN, min(self.PULSE_MAX, pulse))
        
        # 写入目标位置
        comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_GOAL_POSITION,
            pulse
        )
        
        if self._check_comm_result(comm_result, dxl_error, "设置位置", servo_id):
            logger.debug(f"舵机 {servo_id} 移动到 {angle:.1f}° (脉冲:{pulse})")
            return True
        else:
            return False
    
    def set_speed(self, servo_id: int, speed: int) -> bool:
        """
        设置旋转速度（速度模式）
        
        :param servo_id: 舵机ID
        :param speed: 速度 (-1023 ~ 1023, 正值正转，负值反转)
        :return: 是否成功
        """
        speed = max(self.SPEED_MIN, min(self.SPEED_MAX, speed))
        
        # 处理负数（飞特协议中最高位为符号位）
        if speed < 0:
            speed_value = 0x8000 | (-speed)
        else:
            speed_value = speed
        
        # 写入目标速度
        comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_GOAL_SPEED,
            speed_value
        )
        
        if self._check_comm_result(comm_result, dxl_error, "设置速度", servo_id):
            logger.debug(f"舵机 {servo_id} 速度设置为 {speed}")
            return True
        else:
            return False
    
    def move_to_position(self, servo_id: int, angle: float, time_ms: int = 0) -> bool:
        """
        便捷方法：移动到指定角度（自动确保位置模式）
        
        :param servo_id: 舵机ID
        :param angle: 目标角度
        :param time_ms: 运动时间
        :return: 是否成功
        """
        # 先设置为位置模式
        self.set_mode(servo_id, ServoMode.POSITION)
        # 再移动
        return self.set_position(servo_id, angle, time_ms)
    
    def rotate_at_speed(self, servo_id: int, speed: int) -> bool:
        """
        便捷方法：以指定速度旋转（自动确保速度模式）
        
        :param servo_id: 舵机ID
        :param speed: 速度 (-1023 ~ 1023)
        :return: 是否成功
        """
        # 先设置为速度模式
        self.set_mode(servo_id, ServoMode.SPEED)
        # 再设置速度
        return self.set_speed(servo_id, speed)
    
    # ==================== 功能4: 读取舵机数据 ====================
    
    def get_position(self, servo_id: int) -> Optional[float]:
        """
        读取当前位置
        
        :param servo_id: 舵机ID
        :return: 角度值 (0-360度) 或 None
        """
        present_pos, comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_PRESENT_POSITION
        )
        
        if self._check_comm_result(comm_result, dxl_error, "读取位置", servo_id):
            # 脉冲转角度
            angle = (present_pos / self.PULSE_MAX) * self.ANGLE_RANGE
            logger.debug(f"舵机 {servo_id} 当前位置: {angle:.1f}° (脉冲:{present_pos})")
            return angle
        else:
            return None
    
    def get_speed(self, servo_id: int) -> Optional[int]:
        """
        读取当前速度
        
        :param servo_id: 舵机ID
        :return: 速度值 或 None
        """
        present_speed, comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_PRESENT_SPEED
        )
        
        if self._check_comm_result(comm_result, dxl_error, "读取速度", servo_id):
            # 处理符号位
            if present_speed & 0x8000:
                speed = -(present_speed & 0x7FFF)
            else:
                speed = present_speed
            
            logger.debug(f"舵机 {servo_id} 当前速度: {speed}")
            return speed
        else:
            return None
    
    def get_temperature(self, servo_id: int) -> Optional[float]:
        """
        读取温度
        
        :param servo_id: 舵机ID
        :return: 温度(摄氏度) 或 None
        """
        temp, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_TEMPERATURE
        )
        
        if self._check_comm_result(comm_result, dxl_error, "读取温度", servo_id):
            logger.debug(f"舵机 {servo_id} 温度: {temp}°C")
            return float(temp)
        else:
            return None
    
    def get_voltage(self, servo_id: int) -> Optional[float]:
        """
        读取电压
        
        :param servo_id: 舵机ID
        :return: 电压(伏特) 或 None
        """
        voltage, comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            servo_id,
            self.ADDR_VOLTAGE
        )
        
        if self._check_comm_result(comm_result, dxl_error, "读取电压", servo_id):
            # 电压值为实际值的10倍
            voltage_value = voltage / 10.0
            logger.debug(f"舵机 {servo_id} 电压: {voltage_value:.1f}V")
            return voltage_value
        else:
            return None
    
    def get_status(self, servo_id: int) -> Optional[Dict]:
        """
        读取舵机状态（通过错误寄存器）
        
        :param servo_id: 舵机ID
        :return: 状态字典 或 None
        """
        # 读取硬件错误寄存器 (地址 70)
        hw_error, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            servo_id,
            70
        )
        
        if self._check_comm_result(comm_result, dxl_error, "读取状态", servo_id):
            status = {
                'id': servo_id,
                'error_code': hw_error,
                'input_voltage_error': bool(hw_error & 0x01),
                'motor_encoder_error': bool(hw_error & 0x02),
                'overheating_error': bool(hw_error & 0x04),
                'range_error': bool(hw_error & 0x08),
                'checksum_error': bool(hw_error & 0x10),
                'overload_error': bool(hw_error & 0x20),
                'normal': hw_error == 0
            }
            logger.debug(f"舵机 {servo_id} 状态: {status}")
            return status
        else:
            return None
    
    def get_all_data(self, servo_id: int) -> Optional[Dict]:
        """
        读取舵机所有可用数据
        
        :param servo_id: 舵机ID
        :return: 完整数据字典 或 None
        """
        data = {
            'id': servo_id,
            'position': self.get_position(servo_id),
            'speed': self.get_speed(servo_id),
            'temperature': self.get_temperature(servo_id),
            'voltage': self.get_voltage(servo_id),
            'status': self.get_status(servo_id)
        }
        return data
    
    # ==================== 辅助功能 ====================
    
    def enable_torque(self, servo_id: int) -> bool:
        """使能力矩"""
        # 扭矩使能寄存器地址 64
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            servo_id,
            64,
            1
        )
        
        if self._check_comm_result(comm_result, dxl_error, "使能力矩", servo_id):
            logger.info(f"✅ 舵机 {servo_id} 力矩已使能")
            return True
        return False
    
    def disable_torque(self, servo_id: int) -> bool:
        """失能力矩"""
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            servo_id,
            64,
            0
        )
        
        if self._check_comm_result(comm_result, dxl_error, "失能力矩", servo_id):
            logger.info(f"✅ 舵机 {servo_id} 力矩已失能")
            return True
        return False
    
    def ping(self, servo_id: int) -> bool:
        """测试舵机是否在线"""
        model_number, comm_result, dxl_error = self.packet_handler.ping(
            self.port_handler,
            servo_id
        )
        
        if comm_result == scservo_sdk.COMM_SUCCESS:
            logger.debug(f"舵机 {servo_id} 在线, 型号: {model_number}")
            return True
        else:
            logger.warning(f"舵机 {servo_id} 不在线")
            return False
