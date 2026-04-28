"""
幻尔 LewanSoul LX-16A 总线舵机驱动
实现核心功能：
1. 设置ID
2. 设置模式（位置模式/速度模式）
3. 根据模式控制转速或角度
4. 读取舵机状态数据
"""

import serial
import time
import logging
from typing import Dict, Optional
from enum import Enum

logger = logging.getLogger(__name__)


class ServoMode(Enum):
    """舵机工作模式"""
    POSITION = 0  # 位置模式
    SPEED = 1     # 速度模式


class LX16ADriver:
    """
    LX-16A 总线舵机驱动
    
    协议说明：
    - 帧头：0x55 0x55
    - 长度：数据长度 + 3
    - 指令：见 CMD_* 常量
    - 参数：可变
    - 校验和：取反(求和 & 0xFF)
    
    位置范围：0-1000 对应 0-240度
    速度范围：0-1000 (正值正转，负值反转)
    """
    
    # 协议常量
    FRAME_HEADER = 0x55
    CMD_SERVO_MOVE = 1           # 移动到指定位置
    CMD_SET_ID = 2               # 设置ID
    CMD_READ_POS = 28            # 读取位置
    CMD_READ_SPEED = 29          # 读取速度
    CMD_LOAD_UNLOAD = 31         # 使能/失能力矩
    CMD_READ_TEMP = 32           # 读取温度
    CMD_READ_VOLTAGE = 33        # 读取电压
    CMD_READ_DEV_STATUS = 34     # 读取状态
    CMD_SET_MODE = 35            # 设置模式（位置/速度）
    CMD_SET_SPEED = 36           # 设置速度
    
    # 位置范围
    PULSE_MIN = 0
    PULSE_MAX = 1000
    ANGLE_RANGE = 240.0
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        初始化驱动
        
        :param port: 串口号 (如 COM3, /dev/ttyUSB0)
        :param baudrate: 波特率 (默认115200)
        :param timeout: 超时时间(秒)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.current_id = 1  # 当前操作的舵机ID
        
    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            logger.info(f"✅ LX-16A 连接到 {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            logger.error(f"❌ LX-16A 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("🔌 LX-16A 已断开")
    
    def _calculate_checksum(self, data: bytes) -> int:
        """计算校验和"""
        return (~sum(data)) & 0xFF
    
    def _build_frame(self, servo_id: int, cmd: int, params: list = None) -> bytes:
        """
        构建通信帧
        
        :param servo_id: 舵机ID
        :param cmd: 指令
        :param params: 参数列表
        :return: 完整的帧数据
        """
        if params is None:
            params = []
        
        length = len(params) + 3  # ID + CMD + 校验和
        frame = [
            self.FRAME_HEADER,
            self.FRAME_HEADER,
            length,
            servo_id,
            cmd
        ] + params
        
        checksum = self._calculate_checksum(bytes(frame[2:]))
        frame.append(checksum)
        
        return bytes(frame)
    
    def _send_command(self, servo_id: int, cmd: int, params: list = None, 
                     read_response: bool = False, response_length: int = 0) -> Optional[bytes]:
        """
        发送命令并可选读取响应
        
        :param servo_id: 舵机ID
        :param cmd: 指令
        :param params: 参数
        :param read_response: 是否读取响应
        :param response_length: 期望的响应长度
        :return: 响应数据或None
        """
        if not self.serial or not self.serial.is_open:
            logger.error("串口未连接")
            return None
        
        frame = self._build_frame(servo_id, cmd, params)
        
        try:
            # 清空缓冲区
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # 发送数据
            self.serial.write(frame)
            time.sleep(0.01)  # 等待数据传输
            
            if read_response:
                # 读取响应
                response = self.serial.read(response_length)
                if len(response) == response_length:
                    return response
                else:
                    logger.warning(f"响应长度不匹配: 期望{response_length}, 实际{len(response)}")
                    return None
            return None
            
        except Exception as e:
            logger.error(f"通信错误: {e}")
            return None
    
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
        
        params = [new_id]
        response = self._send_command(old_id, self.CMD_SET_ID, params, 
                                     read_response=True, response_length=6)
        
        if response:
            logger.info(f"✅ 舵机ID已从 {old_id} 设置为 {new_id}")
            self.current_id = new_id
            return True
        else:
            logger.error(f"❌ 设置ID失败")
            return False
    
    # ==================== 功能2: 设置模式 ====================
    
    def set_mode(self, servo_id: int, mode: ServoMode) -> bool:
        """
        设置舵机工作模式
        
        :param servo_id: 舵机ID
        :param mode: 工作模式 (POSITION 或 SPEED)
        :return: 是否成功
        """
        params = [mode.value]
        response = self._send_command(servo_id, self.CMD_SET_MODE, params,
                                     read_response=True, response_length=6)
        
        if response:
            mode_name = "位置模式" if mode == ServoMode.POSITION else "速度模式"
            logger.info(f"✅ 舵机 {servo_id} 已设置为 {mode_name}")
            return True
        else:
            logger.error(f"❌ 设置模式失败")
            return False
    
    # ==================== 功能3: 控制转速或角度 ====================
    
    def set_position(self, servo_id: int, angle: float, time_ms: int = 1000) -> bool:
        """
        设置目标角度（位置模式）
        
        :param servo_id: 舵机ID
        :param angle: 目标角度 (0-240度)
        :param time_ms: 运动时间(毫秒)
        :return: 是否成功
        """
        # 角度限制
        angle = max(0, min(240, angle))
        
        # 角度转脉冲值
        pulse = int((angle / self.ANGLE_RANGE) * self.PULSE_MAX)
        
        # 时间分解为低字节和高字节
        time_low = time_ms & 0xFF
        time_high = (time_ms >> 8) & 0xFF
        
        # 脉冲值分解
        pulse_low = pulse & 0xFF
        pulse_high = (pulse >> 8) & 0xFF
        
        params = [pulse_low, pulse_high, time_low, time_high]
        response = self._send_command(servo_id, self.CMD_SERVO_MOVE, params,
                                     read_response=True, response_length=6)
        
        if response:
            logger.debug(f"舵机 {servo_id} 移动到 {angle:.1f}° (脉冲:{pulse}, 时间:{time_ms}ms)")
            return True
        else:
            logger.error(f"❌ 设置位置失败")
            return False
    
    def set_speed(self, servo_id: int, speed: int) -> bool:
        """
        设置旋转速度（速度模式）
        
        :param servo_id: 舵机ID
        :param speed: 速度 (-1000 ~ 1000, 正值正转，负值反转)
        :return: 是否成功
        """
        speed = max(-1000, min(1000, speed))
        
        # 处理负数
        if speed < 0:
            speed = 0x8000 | (-speed)
        
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF
        
        params = [speed_low, speed_high]
        response = self._send_command(servo_id, self.CMD_SET_SPEED, params,
                                     read_response=True, response_length=6)
        
        if response:
            logger.debug(f"舵机 {servo_id} 速度设置为 {speed}")
            return True
        else:
            logger.error(f"❌ 设置速度失败")
            return False
    
    def move_to_position(self, servo_id: int, angle: float, time_ms: int = 1000) -> bool:
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
        :param speed: 速度 (-1000 ~ 1000)
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
        :return: 角度值 (0-240度) 或 None
        """
        response = self._send_command(servo_id, self.CMD_READ_POS,
                                     read_response=True, response_length=6)
        
        if response and len(response) >= 7:
            # 解析脉冲值
            pulse_low = response[5]
            pulse_high = response[6]
            pulse = pulse_low | (pulse_high << 8)
            
            # 脉冲转角度
            angle = (pulse / self.PULSE_MAX) * self.ANGLE_RANGE
            logger.debug(f"舵机 {servo_id} 当前位置: {angle:.1f}° (脉冲:{pulse})")
            return angle
        else:
            logger.warning(f"读取位置失败")
            return None
    
    def get_speed(self, servo_id: int) -> Optional[int]:
        """
        读取当前速度
        
        :param servo_id: 舵机ID
        :return: 速度值 或 None
        """
        response = self._send_command(servo_id, self.CMD_READ_SPEED,
                                     read_response=True, response_length=6)
        
        if response and len(response) >= 7:
            speed_low = response[5]
            speed_high = response[6]
            speed = speed_low | (speed_high << 8)
            
            # 处理负数
            if speed & 0x8000:
                speed = -(speed & 0x7FFF)
            
            logger.debug(f"舵机 {servo_id} 当前速度: {speed}")
            return speed
        else:
            logger.warning(f"读取速度失败")
            return None
    
    def get_temperature(self, servo_id: int) -> Optional[float]:
        """
        读取温度
        
        :param servo_id: 舵机ID
        :return: 温度(摄氏度) 或 None
        """
        response = self._send_command(servo_id, self.CMD_READ_TEMP,
                                     read_response=True, response_length=6)
        
        if response and len(response) >= 6:
            temp = response[5]
            logger.debug(f"舵机 {servo_id} 温度: {temp}°C")
            return float(temp)
        else:
            logger.warning(f"读取温度失败")
            return None
    
    def get_voltage(self, servo_id: int) -> Optional[float]:
        """
        读取电压
        
        :param servo_id: 舵机ID
        :return: 电压(伏特) 或 None
        """
        response = self._send_command(servo_id, self.CMD_READ_VOLTAGE,
                                     read_response=True, response_length=6)
        
        if response and len(response) >= 7:
            voltage_low = response[5]
            voltage_high = response[6]
            voltage = (voltage_low | (voltage_high << 8)) / 100.0
            logger.debug(f"舵机 {servo_id} 电压: {voltage:.2f}V")
            return voltage
        else:
            logger.warning(f"读取电压失败")
            return None
    
    def get_status(self, servo_id: int) -> Optional[Dict]:
        """
        读取舵机完整状态
        
        :param servo_id: 舵机ID
        :return: 状态字典 或 None
        """
        response = self._send_command(servo_id, self.CMD_READ_DEV_STATUS,
                                     read_response=True, response_length=8)
        
        if response and len(response) >= 6:
            status_code = response[5]
            status = {
                'id': servo_id,
                'status_code': status_code,
                'overheating': bool(status_code & 0x01),
                'overloaded': bool(status_code & 0x02),
                'normal': status_code == 0
            }
            logger.debug(f"舵机 {servo_id} 状态: {status}")
            return status
        else:
            logger.warning(f"读取状态失败")
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
        params = [1]  # 1=使能
        response = self._send_command(servo_id, self.CMD_LOAD_UNLOAD, params,
                                     read_response=True, response_length=6)
        if response:
            logger.info(f"✅ 舵机 {servo_id} 力矩已使能")
            return True
        return False
    
    def disable_torque(self, servo_id: int) -> bool:
        """失能力矩"""
        params = [0]  # 0=失能
        response = self._send_command(servo_id, self.CMD_LOAD_UNLOAD, params,
                                     read_response=True, response_length=6)
        if response:
            logger.info(f"✅ 舵机 {servo_id} 力矩已失能")
            return True
        return False
    
    def ping(self, servo_id: int) -> bool:
        """测试舵机是否在线"""
        # LX-16A 读取位置命令返回 6 字节
        response = self._send_command(servo_id, self.CMD_READ_POS,
                                     read_response=True, response_length=6)
        return response is not None
