"""
驱动接口 — 文档契约。

JointActuatorInterface — 关节执行器统一接口
    Feetech ST3215Driver (串口) 和 RobStride RobStrideOfficialDriver (CAN)
    都是机器人关节执行器，提供相同控制原语。Python 鸭子类型，不强制继承。
"""
from abc import ABC, abstractmethod
from typing import List


class JointActuatorInterface(ABC):
    """关节执行器接口 — Feetech 舵机 & RobStride 电机共用."""

    # ── 生命周期 ──

    @abstractmethod
    def connect(self, port: str = "", **kwargs) -> bool:
        """连接通信接口."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """断开连接."""
        ...

    @abstractmethod
    def ping(self, device_id: int) -> bool:
        """检测执行器是否在线."""
        ...

    # ── 位置控制 ──

    @abstractmethod
    def set_position(self, device_id: int, position: float, time_ms: int = 500) -> bool:
        """设置目标位置."""
        ...

    @abstractmethod
    def get_position(self, device_id: int) -> float:
        """读取当前位置 (步进值)."""
        ...

    # ── ID 管理 ──

    @abstractmethod
    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改设备 ID (写入 EEPROM/Flash)."""
        ...

    def scan(self, start_id: int = 1, end_id: int = 253) -> List[int]:
        """扫描总线上在线的设备 ID 列表."""
        ...

    # ── 速度 / 力矩 / 使能 ──

    def set_velocity(self, device_id: int, velocity: float) -> bool:
        """速度模式."""
        ...

    def set_torque(self, device_id: int, torque: float) -> None:
        """设置力矩 / 开关扭矩."""
        ...

    def enable(self, device_id: int) -> None:
        """使能 (上电)."""
        ...

    def disable(self, device_id: int) -> None:
        """失能 (掉电)."""
        ...

    def emergency_stop(self) -> None:
        """总线级急停."""
        ...

    # ── 状态 ──

    def get_state(self, device_id: int) -> dict:
        """返回 {position, velocity, torque, temperature, voltage, ...}."""
        ...
