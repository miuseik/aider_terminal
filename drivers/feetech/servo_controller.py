"""
舵机控制器 - scservo_sdk 的封装
"""
import threading
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, asdict
from scservo_sdk import *

# SCS 舵机寄存器地址 (EEPROM - 持久化，写入需要解锁)
ADDR_SCS_ID = 5                    # 舵机ID (1-253)
ADDR_SCS_BAUD_RATE = 6             # 波特率
ADDR_SCS_MIN_ANGLE_LIMIT = 9       # 顺时针角度限制 (2字节)
ADDR_SCS_MAX_ANGLE_LIMIT = 11      # 逆时针角度限制 (2字节)
ADDR_SCS_OVERCURRENT_PROT = 38     # 过流保护 (2字节)
ADDR_SCS_VELOCITY_I_GAIN = 39      # 速度I增益 (1字节)

# SCS 舵机寄存器地址 (RAM - 易失性)
ADDR_SCS_TORQUE_ENABLE = 40        # 扭矩使能 (1字节)
ADDR_SCS_GOAL_ACCELERATION = 41    # 目标加速度 (1字节)
ADDR_SCS_GOAL_POSITION = 42        # 目标位置 (2字节)
ADDR_SCS_GOAL_TIME = 46            # 目标时间/速度 (2字节)
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_TORQUE_LIMIT = 48         # 扭矩限制 (2字节)
ADDR_SCS_LOCK = 55                 # EEPROM锁 (0=解锁, 1=锁定)
ADDR_SCS_PRESENT_POSITION = 56     # 当前位置 (2字节, 只读)
ADDR_SCS_PRESENT_SPEED = 58        # 当前速度 (2字节, 只读)
ADDR_SCS_PRESENT_LOAD = 60         # 当前负载 (2字节, 只读)
ADDR_SCS_PRESENT_VOLTAGE = 62      # 当前电压 (1字节, 只读)
ADDR_SCS_PRESENT_TEMPERATURE = 63  # 当前温度 (1字节, 只读)

# SCS 舵机寄存器地址 (DEFAULT - 运动曲线参数)
ADDR_SCS_MOVING_THRESHOLD = 80     # 移动阈值 (1字节)
ADDR_SCS_DTS = 81                  # DTs 毫秒 (1字节)
ADDR_SCS_VK = 82                   # Vk 毫秒 (1字节)  
ADDR_SCS_VMIN = 83                 # Vmin (1字节)
ADDR_SCS_VMAX = 84                 # Vmax (1字节)
ADDR_SCS_AMAX = 85                 # Amax - 最大加速度 (1字节)
ADDR_SCS_KACC = 86                 # KAcc (1字节)

# 寄存器定义（带描述）- 来自 FT SCServo Debug 的完整列表
SERVO_REGISTERS = {
    # ============ EPROM 寄存器 (持久化，写入需要解锁) ============
    'firmware_main': {
        'addr': 0, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Firmware Main Version',
        'desc': '主固件版本号（只读）。', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'firmware_secondary': {
        'addr': 1, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Firmware Secondary Version',
        'desc': '次固件版本号（只读）。', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'servo_main_ver': {
        'addr': 3, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Servo Main Version',
        'desc': '舵机硬件主版本（只读）。', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'servo_sub_ver': {
        'addr': 4, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Servo Sub Version',
        'desc': '舵机硬件次版本（只读）。', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'id': {
        'addr': 5, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'ID',
        'desc': '唯一舵机ID (1-253)。总线上的每个舵机必须有不同的ID。',
        'unit': '', 'range': '1-253', 'default': '1'
    },
    'baud_rate': {
        'addr': 6, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Baud Rate',
        'desc': '串口速度。0=1Mbps, 1=500K, 2=250K, 3=128K, 4=115200, 5=76800, 6=57600, 7=38400',
        'unit': '', 'range': '0-7', 'default': '0'
    },
    'reserved': {
        'addr': 7, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Reserved',
        'desc': '保留寄存器。', 'unit': '', 'range': '0-255', 'default': '0'
    },
    'status_return_level': {
        'addr': 8, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Status Return Level',
        'desc': '响应模式。0=无响应, 1=仅响应READ, 2=响应所有命令',
        'unit': '', 'range': '0-2', 'default': '1'
    },
    'min_angle': {
        'addr': 9, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Min Position Limit',
        'desc': '最小角度限制。舵机不会旋转超过此位置。',
        'unit': 'steps', 'range': '0-4095', 'default': '0'
    },
    'max_angle': {
        'addr': 11, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Position Limit',
        'desc': '最大角度限制。舵机不会旋转超过此位置。',
        'unit': 'steps', 'range': '0-4095', 'default': '4095'
    },
    'max_temp_limit': {
        'addr': 13, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Temperature Limit',
        'desc': '最高温度限制。超过时舵机会禁用。',
        'unit': '°C', 'range': '0-100', 'default': '70'
    },
    'max_voltage': {
        'addr': 14, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Input Voltage',
        'desc': '最大输入电压限制。值÷10 = 伏特。',
        'unit': '×0.1V', 'range': '0-255', 'default': '140'
    },
    'min_voltage': {
        'addr': 15, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Min Input Voltage',
        'desc': '最小输入电压限制。值÷10 = 伏特。',
        'unit': '×0.1V', 'range': '0-255', 'default': '40'
    },
    'max_torque': {
        'addr': 16, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Torque Limit',
        'desc': '最大扭矩输出限制。1000=100%。',
        'unit': '‰', 'range': '0-1000', 'default': '1000'
    },
    'setting_byte': {
        'addr': 18, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Setting Byte',
        'desc': '配置标志。Bit0=方向, Bit1=模式等。',
        'unit': '', 'range': '0-255', 'default': '12'
    },
    'protection_switch': {
        'addr': 19, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Protection Switch',
        'desc': '启用/禁用各种保护功能（位掩码）。',
        'unit': '', 'range': '0-255', 'default': '44'
    },
    'led_alarm': {
        'addr': 20, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'LED Alarm Condition',
        'desc': 'LED报警触发条件（位掩码）。',
        'unit': '', 'range': '0-255', 'default': '47'
    },
    'position_p_gain': {
        'addr': 21, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position P Gain',
        'desc': '位置PID控制器的比例增益 (P)。',
        'unit': '', 'range': '0-255', 'default': '32'
    },
    'position_d_gain': {
        'addr': 22, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position D Gain',
        'desc': '位置PID控制器的微分增益 (D)。',
        'unit': '', 'range': '0-255', 'default': '32'
    },
    'position_i_gain': {
        'addr': 23, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position I Gain',
        'desc': '位置PID控制器的积分增益 (I)。',
        'unit': '', 'range': '0-255', 'default': '0'
    },
    'punch': {
        'addr': 24, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Punch',
        'desc': '施加到电机的最小PWM值。有助于克服静摩擦力。',
        'unit': '', 'range': '0-1000', 'default': '16'
    },
    'max_i': {
        'addr': 25, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'MAX I',
        'desc': 'PID控制器的最大积分值。',
        'unit': '', 'range': '0-255', 'default': '0'
    },
    'cw_dead_band': {
        'addr': 26, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'CW Dead Band',
        'desc': '顺时针死区。此范围内的位置误差将被忽略。',
        'unit': 'steps', 'range': '0-255', 'default': '1'
    },
    'ccw_dead_band': {
        'addr': 27, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'CCW Dead Band',
        'desc': '逆时针死区。此范围内的位置误差将被忽略。',
        'unit': 'steps', 'range': '0-255', 'default': '1'
    },
    'overload_current': {
        'addr': 28, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overload Current',
        'desc': '过载电流保护阈值。',
        'unit': 'mA', 'range': '0-1000', 'default': '310'
    },
    'angular_resolution': {
        'addr': 30, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Angular Resolution',
        'desc': '位置分辨率倍数。',
        'unit': '', 'range': '0-255', 'default': '1'
    },
    'position_offset': {
        'addr': 31, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position Offset Value',
        'desc': '用于校准的位置偏移量。添加到实际位置。',
        'unit': 'steps', 'range': '-2048 to 2047', 'default': '0'
    },
    'work_mode': {
        'addr': 33, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Work Mode',
        'desc': '工作模式。0=位置伺服, 1=轮式模式, 2=PWM模式, 3=步进模式',
        'unit': '', 'range': '0-3', 'default': '0'
    },
    'protect_torque': {
        'addr': 34, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Protect Torque',
        'desc': '触发保护时的扭矩级别。',
        'unit': '%', 'range': '0-100', 'default': '20'
    },
    'overload_protection_time': {
        'addr': 35, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overload Protection Time',
        'desc': '过载保护触发前的时间。',
        'unit': '×20ms', 'range': '0-255', 'default': '200'
    },
    'overload_torque': {
        'addr': 36, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overload Torque',
        'desc': '过载检测的扭矩阈值。',
        'unit': '%', 'range': '0-100', 'default': '80'
    },
    'velocity_p_gain': {
        'addr': 37, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Velocity P Gain',
        'desc': '速度PID控制器的比例增益 (P)。',
        'unit': '', 'range': '0-255', 'default': '10'
    },
    'overcurrent': {
        'addr': 38, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overcurrent Protection',
        'desc': '过流保护阈值。',
        'unit': 'mA', 'range': '0-1000', 'default': '200'
    },
    'velocity_i_gain': {
        'addr': 39, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Velocity I Gain',
        'desc': '速度PID控制器的积分增益 (I)。',
        'unit': '', 'range': '0-255', 'default': '200'
    },
    
    # ============ SRAM 寄存器 (易失性，断电重置) ============
    'torque_enable': {
        'addr': 40, 'size': 1, 'area': 'SRAM', 'rw': 'rw', 'name': 'Torque Enable',
        'desc': '启用/禁用扭矩。0=关闭（自由）, 1=开启（保持位置）',
        'unit': '', 'range': '0-1', 'default': '0'
    },
    'goal_acceleration': {
        'addr': 41, 'size': 1, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal Acceleration',
        'desc': '目标加速度。0=无限制，越高=加速越快。',
        'unit': '', 'range': '0-255', 'default': '0'
    },
    'goal_position': {
        'addr': 42, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal Position',
        'desc': '舵机要移动到的目标位置。',
        'unit': 'steps', 'range': '0-4095', 'default': '—'
    },
    'goal_pwm': {
        'addr': 44, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal PWM',
        'desc': '目标PWM值（用于PWM模式）。',
        'unit': '', 'range': '-1000 to 1000', 'default': '0'
    },
    'goal_speed': {
        'addr': 46, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal Velocity',
        'desc': '目标速度或到达位置的时间。',
        'unit': 'steps/s or ms', 'range': '0-65535', 'default': '0'
    },
    'torque_limit': {
        'addr': 48, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Torque Limit',
        'desc': '运行时扭矩限制。1000=100%。',
        'unit': '‰', 'range': '0-1000', 'default': '1000'
    },
    'lock': {
        'addr': 55, 'size': 1, 'area': 'SRAM', 'rw': 'rw', 'name': 'Lock',
        'desc': 'EEPROM锁。0=未锁定（允许写入）, 1=锁定（保护）',
        'unit': '', 'range': '0-1', 'default': '1'
    },
    'present_position': {
        'addr': 56, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Position',
        'desc': '当前舵机位置。',
        'unit': 'steps', 'range': '0-4095', 'default': '—'
    },
    'present_speed': {
        'addr': 58, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Velocity',
        'desc': '当前舵机速度。负数=反向。',
        'unit': 'steps/s', 'range': '±32767', 'default': '—'
    },
    'present_pwm': {
        'addr': 60, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present PWM',
        'desc': '当前PWM输出值。',
        'unit': '', 'range': '±1000', 'default': '—'
    },
    'present_voltage': {
        'addr': 62, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Input Voltage',
        'desc': '当前输入电压。值÷10 = 伏特。',
        'unit': '×0.1V', 'range': '0-255', 'default': '—'
    },
    'present_temp': {
        'addr': 63, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Temperature',
        'desc': '当前温度。',
        'unit': '°C', 'range': '0-100', 'default': '—'
    },
    'sync_write_flag': {
        'addr': 64, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Sync Write Flag',
        'desc': '指示是否接收到同步写入命令。',
        'unit': '', 'range': '0-1', 'default': '—'
    },
    'hardware_error': {
        'addr': 65, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Hardware Error Status',
        'desc': '硬件错误标志（位掩码）。',
        'unit': '', 'range': '0-255', 'default': '—'
    },
    'moving_status': {
        'addr': 66, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Moving Status',
        'desc': '0=停止, 1=正在移动到目标位置。',
        'unit': '', 'range': '0-1', 'default': '—'
    },
    'present_current': {
        'addr': 69, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Current',
        'desc': '当前电流消耗。',
        'unit': 'mA', 'range': '0-65535', 'default': '—'
    },
    
    # ============ DEFAULT 寄存器 (运动曲线参数) ============
    'moving_threshold': {
        'addr': 80, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Moving Threshold',
        'desc': '检测舵机是否在移动的阈值。',
        'unit': '', 'range': '0-255', 'default': '1'
    },
    'dts': {
        'addr': 81, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'DTs(ms)',
        'desc': '移动前的死区时间。平滑方向变化。',
        'unit': 'ms', 'range': '0-255', 'default': '20'
    },
    'vk': {
        'addr': 82, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Vk(ms)',
        'desc': '速度常数。影响速度曲线的平滑度。',
        'unit': 'ms', 'range': '0-255', 'default': '50'
    },
    'vmin': {
        'addr': 83, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Vmin',
        'desc': '最小速度。舵机不会以低于此速度移动。',
        'unit': 'steps/s', 'range': '0-255', 'default': '1'
    },
    'vmax': {
        'addr': 84, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Vmax',
        'desc': '最大速度。舵机不会以高于此速度移动。',
        'unit': 'steps/s ×50', 'range': '0-255', 'default': '65'
    },
    'amax': {
        'addr': 85, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Amax',
        'desc': '最大加速度。高=急剧，低=平滑运动。',
        'unit': '', 'range': '0-254', 'default': '50'
    },
    'kacc': {
        'addr': 86, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'KAcc',
        'desc': '加速度系数。与Amax相乘。',
        'unit': '', 'range': '0-255', 'default': '1'
    },
}


@dataclass
class ServoInfo:
    id: int
    model_number: int
    position: int = 0
    speed: int = 0
    load: int = 0
    voltage: float = 0.0
    temperature: int = 0
    present_current: int = 0
    moving_status: int = 0
    goal_position: int = 0
    is_online: bool = True
    min_position: int = 0
    max_position: int = 4095
    center_position: int = 2048


class ServoController:
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler: Optional[PortHandler] = None
        self.packet_handler: Optional[PacketHandler] = None
        self.servos: Dict[int, ServoInfo] = {}
        self.is_connected = False
        self._lock = threading.Lock()
    
    def connect(self) -> Tuple[bool, str]:
        """连接到串口"""
        try:
            self.port_handler = PortHandler(self.port)
            self.packet_handler = PacketHandler(0)
            
            if not self.port_handler.openPort():
                return False, f"Cannot open port {self.port}"
            
            if not self.port_handler.setBaudRate(self.baudrate):
                return False, f"Cannot set baudrate {self.baudrate}"
            
            self.is_connected = True
            return True, "Connected successfully"
        except Exception as e:
            return False, str(e)
    
    def disconnect(self):
        """断开口连接"""
        if self.port_handler:
            self.port_handler.closePort()
        self.is_connected = False
        self.servos.clear()
    
    def scan_servos(self, start_id: int = 1, end_id: int = 20) -> List[ServoInfo]:
        """在ID范围内扫描连接的舵机"""
        if not self.is_connected:
            return []
        
        found_servos = []
        with self._lock:
            for servo_id in range(start_id, end_id + 1):
                model_number, comm_result, error = self.packet_handler.ping(
                    self.port_handler, servo_id
                )
                if comm_result == COMM_SUCCESS:
                    servo = ServoInfo(id=servo_id, model_number=model_number)
                    self.servos[servo_id] = servo
                    found_servos.append(servo)
        
        return found_servos
    
    def ping(self, servo_id: int) -> Tuple[bool, str]:
        """Ping特定舵机"""
        if not self.is_connected:
            return False, "Not connected"
        
        with self._lock:
            model_number, comm_result, error = self.packet_handler.ping(
                self.port_handler, servo_id
            )
            if comm_result == COMM_SUCCESS:
                return True, f"Servo {servo_id} online, model: {model_number}"
            else:
                return False, f"Servo {servo_id} not responding"
    
    def read_position(self, servo_id: int) -> Optional[int]:
        """读取舵机的当前位置"""
        if not self.is_connected:
            return None
        
        with self._lock:
            position, comm_result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_POSITION
            )
            if comm_result == COMM_SUCCESS:
                # 处理有符号值（某些舵机的位置可以为负）
                if position > 32767:
                    position = position - 65536
                if servo_id in self.servos:
                    self.servos[servo_id].position = position
                return position
            return None
    
    def read_status(self, servo_id: int) -> Optional[Dict]:
        """读取舵机的完整状态"""
        if not self.is_connected or servo_id not in self.servos:
            return None
        
        servo = self.servos[servo_id]
        
        with self._lock:
            # 读取位置
            pos, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_POSITION
            )
            if res == COMM_SUCCESS:
                if pos > 32767:
                    pos = pos - 65536
                servo.position = pos
            
            # 读取速度
            speed, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_SPEED
            )
            if res == COMM_SUCCESS:
                if speed > 32767:
                    speed = speed - 65536
                servo.speed = speed
            
            # 读取负载
            load, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_LOAD
            )
            if res == COMM_SUCCESS:
                if load > 32767:
                    load = load - 65536
                servo.load = load
            
            # 读取电压
            voltage, res, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_VOLTAGE
            )
            if res == COMM_SUCCESS:
                servo.voltage = voltage / 10.0
            
            # 读取温度
            temp, res, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_TEMPERATURE
            )
            if res == COMM_SUCCESS:
                servo.temperature = temp
            
            # 读取电流
            current, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, 69  # ADDR_SCS_PRESENT_CURRENT
            )
            if res == COMM_SUCCESS:
                servo.present_current = current
            
            # 读取移动状态
            moving, res, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, 66  # ADDR_SCS_MOVING_STATUS
            )
            if res == COMM_SUCCESS:
                servo.moving_status = moving
            
            # 读取目标位置
            goal, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION
            )
            if res == COMM_SUCCESS:
                servo.goal_position = goal
        
        return asdict(servo)
    
    def set_position(self, servo_id: int, position: int, time_ms: int = 500) -> Tuple[bool, str]:
        """设置舵机位置"""
        if not self.is_connected:
            return False, "Not connected"
        
        # 限制位置范围
        position = max(0, min(4095, position))
        
        with self._lock:
            # 写入目标时间
            self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_TIME, time_ms
            )
            # 写入目标位置
            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION, position
            )
            
            if comm_result == COMM_SUCCESS:
                return True, f"Moving servo {servo_id} to {position}"
            else:
                return False, f"Failed to move servo {servo_id}"
    
    def set_torque(self, servo_id: int, enable: bool) -> Tuple[bool, str]:
        """启用/禁用舵机扭矩"""
        if not self.is_connected:
            return False, "Not connected"
        
        with self._lock:
            comm_result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_TORQUE_ENABLE, 1 if enable else 0
            )
            
            if comm_result == COMM_SUCCESS:
                return True, f"Torque {'enabled' if enable else 'disabled'} for servo {servo_id}"
            else:
                return False, f"Failed to set torque for servo {servo_id}"
    
    def center_servo(self, servo_id: int) -> Tuple[bool, str]:
        """将舵机移动到中心位置 (2048)"""
        return self.set_position(servo_id, 2048, 1000)
    
    def test_range(self, servo_id: int, min_pos: int = 0, max_pos: int = 4095) -> Tuple[bool, str]:
        """测试舵机的全范围运动"""
        if not self.is_connected:
            return False, "Not connected"
        
        # 移动到最小位置
        success, msg = self.set_position(servo_id, min_pos, 1500)
        if not success:
            return False, msg
        
        return True, f"Testing range {min_pos} -> {max_pos} for servo {servo_id}"
    
    def get_all_servos(self) -> List[Dict]:
        """获取所有已知舵机的列表"""
        return [asdict(s) for s in self.servos.values()]
    
    def set_calibration(self, servo_id: int, min_pos: int, max_pos: int, center_pos: int):
        """设置舵机的校准值"""
        if servo_id in self.servos:
            self.servos[servo_id].min_position = min_pos
            self.servos[servo_id].max_position = max_pos
            self.servos[servo_id].center_position = center_pos
    
    def unlock_eeprom(self, servo_id: int) -> bool:
        """解锁EEPROM以进行写入（更改ID前必需）"""
        with self._lock:
            comm_result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_LOCK, 0
            )
            return comm_result == COMM_SUCCESS
    
    def lock_eeprom(self, servo_id: int) -> bool:
        """写入后锁定EEPROM"""
        with self._lock:
            comm_result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_LOCK, 1
            )
            return comm_result == COMM_SUCCESS
    
    def change_servo_id(self, old_id: int, new_id: int) -> Tuple[bool, str]:
        """
        更改舵机ID。
        ⚠️ 重要：更改ID时只连接一个舵机！
        """
        if not self.is_connected:
            return False, "Not connected"
        
        if new_id < 1 or new_id > 253:
            return False, "ID must be between 1 and 253"
        
        if old_id == new_id:
            return False, "New ID is same as old ID"
        
        # 首先ping以确保old_id存在
        success, _ = self.ping(old_id)
        if not success:
            return False, f"Servo ID {old_id} not found"
        
        # 检查new_id是否已存在
        success, _ = self.ping(new_id)
        if success:
            return False, f"ID {new_id} already in use by another servo!"
        
        try:
            # 先禁用扭矩
            self.set_torque(old_id, False)
            
            # 解锁EEPROM
            if not self.unlock_eeprom(old_id):
                return False, "Failed to unlock EEPROM"
            
            # 写入新ID
            with self._lock:
                comm_result, _ = self.packet_handler.write1ByteTxRx(
                    self.port_handler, old_id, ADDR_SCS_ID, new_id
                )
            
            if comm_result != COMM_SUCCESS:
                self.lock_eeprom(old_id)
                return False, "Failed to write new ID"
            
            # 用新ID锁定EEPROM
            self.lock_eeprom(new_id)
            
            # 验证新ID是否有效
            import time
            time.sleep(0.1)
            success, _ = self.ping(new_id)
            if success:
                # 更新内部舵机列表
                if old_id in self.servos:
                    servo = self.servos.pop(old_id)
                    servo.id = new_id
                    self.servos[new_id] = servo
                return True, f"Successfully changed ID from {old_id} to {new_id}"
            else:
                return False, "ID changed but verification failed. Try scanning again."
                
        except Exception as e:
            return False, f"Error changing ID: {str(e)}"
    
    def read_servo_id(self, servo_id: int) -> Optional[int]:
        """从舵机读取存储的ID"""
        if not self.is_connected:
            return None
        
        with self._lock:
            read_id, comm_result, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_ID
            )
            if comm_result == COMM_SUCCESS:
                return read_id
            return None
    
    def read_register(self, servo_id: int, addr: int, size: int) -> Optional[int]:
        """从舵机读取寄存器值"""
        if not self.is_connected:
            return None
        
        with self._lock:
            if size == 1:
                value, comm_result, _ = self.packet_handler.read1ByteTxRx(
                    self.port_handler, servo_id, addr
                )
            else:  # size == 2
                value, comm_result, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, servo_id, addr
                )
            
            if comm_result == COMM_SUCCESS:
                return value
            return None
    
    def write_register(self, servo_id: int, addr: int, size: int, value: int, unlock_eeprom: bool = False) -> Tuple[bool, str]:
        """向舵机寄存器写入值"""
        if not self.is_connected:
            return False, "Not connected"
        
        try:
            # 对于EEPROM寄存器，先解锁
            if unlock_eeprom:
                self.unlock_eeprom(servo_id)
            
            with self._lock:
                if size == 1:
                    comm_result, _ = self.packet_handler.write1ByteTxRx(
                        self.port_handler, servo_id, addr, value
                    )
                else:  # size == 2
                    comm_result, _ = self.packet_handler.write2ByteTxRx(
                        self.port_handler, servo_id, addr, value
                    )
            
            # 写入后锁定EEPROM
            if unlock_eeprom:
                self.lock_eeprom(servo_id)
            
            if comm_result == COMM_SUCCESS:
                return True, f"Written {value} to address {addr}"
            else:
                return False, f"Failed to write to address {addr}"
        except Exception as e:
            return False, str(e)
    
    def read_all_registers(self, servo_id: int) -> Dict:
        """从舵机读取所有定义的寄存器"""
        if not self.is_connected:
            return {}
        
        result = {}
        for key, reg in SERVO_REGISTERS.items():
            value = self.read_register(servo_id, reg['addr'], reg['size'])
            result[key] = {
                'addr': reg['addr'],
                'name': reg['name'],
                'value': value,
                'area': reg['area'],
                'rw': reg['rw'],
                'size': reg['size'],
                'desc': reg.get('desc', ''),
                'unit': reg.get('unit', ''),
                'range': reg.get('range', ''),
                'default': reg.get('default', '')
            }
        
        return result
    
    def write_motion_params(self, servo_id: int, amax: int = None, vmax: int = None, 
                           vmin: int = None, dts: int = None, vk: int = None) -> Tuple[bool, str]:
        """写入运动曲线参数"""
        if not self.is_connected:
            return False, "Not connected"
        
        results = []
        if amax is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_AMAX, 1, amax)
            results.append(f"Amax: {'OK' if success else 'FAIL'}")
        
        if vmax is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_VMAX, 1, vmax)
            results.append(f"Vmax: {'OK' if success else 'FAIL'}")
        
        if vmin is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_VMIN, 1, vmin)
            results.append(f"Vmin: {'OK' if success else 'FAIL'}")
        
        if dts is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_DTS, 1, dts)
            results.append(f"DTs: {'OK' if success else 'FAIL'}")
        
        if vk is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_VK, 1, vk)
            results.append(f"Vk: {'OK' if success else 'FAIL'}")
        
        return True, ", ".join(results)


# 全局控制器实例
controller = ServoController()

