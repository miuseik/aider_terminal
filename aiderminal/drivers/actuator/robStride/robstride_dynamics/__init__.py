"""
Robstride 电机驱动子包

基于灵足时代官方 EL-A3 SDK，仅包含电机级功能：
- CAN/SLCAN 底层驱动
- 电机协议、数据类型、参数读写
"""

from .can_driver import RobstrideCanDriver
from .data_types import (
    MotorFeedback,
    MotorHighSpdInfo,
    MotorLowSpdInfo,
    MotorAngleLimitMaxVel,
    MotorMaxAccLimit,
    ParamReadResult,
    FirmwareVersion,
)
from .protocol import (
    CommType,
    MotorType,
    RunMode,
    ControlMode,
    ModeState,
    FaultBit,
    ParamIndex,
    MotorParams,
    MOTOR_PARAMS,
    DEFAULT_MOTOR_TYPE_MAP,
    DEFAULT_JOINT_DIRECTIONS,
    DEFAULT_JOINT_OFFSETS,
    DEFAULT_JOINT_LIMITS,
)
from .utils import (
    float_to_uint16,
    uint16_to_float,
    rad_to_deg,
    deg_to_rad,
    clamp,
)


def get_slcan_driver():
    """延迟导入 SlcanCanDriver（避免无 pyserial 环境下 import 失败）"""
    from .slcan_can_driver import SlcanCanDriver
    return SlcanCanDriver
