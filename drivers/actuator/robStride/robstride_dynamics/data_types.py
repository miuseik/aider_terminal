"""
Robstride 电机数据结构定义

仅包含电机级数据类型，不含 Arm/末端位姿/轨迹等上层结构。
来源: EL-A3 SDK (灵足时代官方)
"""

import struct as _st
from dataclasses import dataclass
import time


@dataclass
class MotorFeedback:
    """单个电机反馈数据（来自 Type 2 反馈帧）"""
    motor_id: int = 0
    position: float = 0.0       # rad（电机坐标系）
    velocity: float = 0.0       # rad/s
    torque: float = 0.0         # Nm
    temperature: float = 0.0    # °C
    mode_state: int = 0         # 0=Reset, 1=Cali, 2=Motor
    fault_code: int = 0         # 6 位故障码
    is_valid: bool = False
    timestamp: float = 0.0      # 反馈时间戳


@dataclass
class MotorHighSpdInfo:
    """电机高速反馈信息（从 Type 2 反馈帧直接获取）"""
    motor_id: int = 0
    speed: float = 0.0        # rad/s
    current: float = 0.0      # A（需通过参数读取获得精确值）
    position: float = 0.0     # rad
    torque: float = 0.0       # Nm
    timestamp: float = 0.0


@dataclass
class MotorLowSpdInfo:
    """电机低速反馈信息（需通过参数读取 Type 17 获取）"""
    motor_id: int = 0
    voltage: float = 0.0       # V (VBUS)
    driver_temp: float = 0.0   # °C（暂不支持，Robstride 仅反馈绕组温度）
    motor_temp: float = 0.0    # °C
    fault_code: int = 0
    bus_current: float = 0.0   # A
    timestamp: float = 0.0


@dataclass
class MotorAngleLimitMaxVel:
    """电机角度限制与最大速度"""
    motor_num: int = 0
    max_angle_limit: float = 0.0   # rad
    min_angle_limit: float = 0.0   # rad
    max_joint_spd: float = 0.0     # rad/s


@dataclass
class MotorMaxAccLimit:
    """电机最大加速度限制"""
    motor_num: int = 0
    max_joint_acc: float = 0.0     # rad/s²


@dataclass
class ParamReadResult:
    """参数读取结果"""
    motor_id: int = 0
    param_index: int = 0
    value: float = 0.0
    success: bool = False
    timestamp: float = 0.0
    raw_bytes: bytes = b"\x00\x00\x00\x00"

    @property
    def value_uint8(self) -> int:
        return self.raw_bytes[0] if self.raw_bytes else 0

    @property
    def value_uint16(self) -> int:
        return _st.unpack_from("<H", self.raw_bytes, 0)[0] if len(self.raw_bytes) >= 2 else 0

    @property
    def value_uint32(self) -> int:
        return _st.unpack_from("<I", self.raw_bytes, 0)[0] if len(self.raw_bytes) >= 4 else 0


@dataclass
class FirmwareVersion:
    """固件版本信息"""
    motor_id: int = 0
    version_bytes: bytes = b""
    version_str: str = ""
    timestamp: float = 0.0
