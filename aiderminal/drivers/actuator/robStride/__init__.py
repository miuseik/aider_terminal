"""
Robstride 电机驱动包

基于灵足时代官方 EL-A3 SDK，仅包含电机级功能：
- CAN/SLCAN 底层驱动
- 电机协议、数据类型、参数读写
"""

from aiderminal.drivers.actuator.robStride.robstride_dynamics.can_driver import RobstrideCanDriver
from aiderminal.drivers.actuator.robStride.robstride_dynamics.data_types import (
    MotorFeedback,
    MotorHighSpdInfo,
    MotorLowSpdInfo,
    MotorAngleLimitMaxVel,
    MotorMaxAccLimit,
    ParamReadResult,
    FirmwareVersion,
)
from aiderminal.drivers.actuator.robStride.robstride_dynamics.protocol import (
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
    DEFAULT_JOINT_LIMITS,
)
from aiderminal.drivers.actuator.robStride.robstride_dynamics.utils import (
    float_to_uint16,
    uint16_to_float,
    rad_to_deg,
    deg_to_rad,
    clamp,
)
from aiderminal.drivers.actuator.robStride.robstride_driver import (
    RobStrideOfficialDriver,
    RobStrideMotor,
)


def get_slcan_driver():
    """延迟导入 SlcanCanDriver（避免无 pyserial 环境下 import 失败）"""
    from aiderminal.drivers.actuator.robStride.robstride_dynamics.slcan_can_driver import SlcanCanDriver
    return SlcanCanDriver


__version__ = "2.0.0"
