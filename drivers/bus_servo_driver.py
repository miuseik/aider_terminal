"""
统一总线舵机驱动调度层
负责根据配置选择合适的品牌驱动，提供统一的工厂接口
支持扩展：未来可轻松添加新品牌（如灵足、其他总线舵机）
"""

import logging
from typing import Dict, Optional, List
from enum import Enum

logger = logging.getLogger(__name__)


class ServoType(Enum):
    """舵机类型枚举"""
    LX16A = "lx16a"
    ST3215 = "st3215"
    # 未来可扩展:
    # RS00 = "rs00"      # 灵足 Robstride
    # EL05 = "el05"      # 灵足 EduLite
    # FEETECH_SCS = "scs" # 飞特 SCS系列


def create_servo_driver(servo_type: ServoType, port: str, baudrate: int = 115200, 
                       servo_ids: List[int] = None):
    """
    工厂函数：创建舵机驱动实例（调度层）
    
    :param servo_type: 舵机类型 (LX16A, ST3215, 或未来扩展的其他类型)
    :param port: 串口号
    :param baudrate: 波特率
    :param servo_ids: 舵机ID列表
    :return: 舵机驱动实例
    
    说明：
    - 本函数作为调度层，根据类型委托给具体的品牌驱动实现
    - 新增舵机类型时，只需在此添加分支并导入对应驱动即可
    """
    if servo_type == ServoType.LX16A:
        # 委托给 Hiwonder LX-16A 驱动
        from drivers.Hiwonder.lx16a_driver import LX16ADriver
        print(f"📦 创建 LX-16A 驱动 (端口: {port}, 波特率: {baudrate})")
        return LX16ADriver(port=port, baudrate=baudrate)
        
    elif servo_type == ServoType.ST3215:
        # 委托给 Feetech ST3215 驱动
        from drivers.feetech.st3215_driver import ST3215Driver
        print(f"📦 创建 ST3215 驱动 (端口: {port}, 波特率: {baudrate})")
        return ST3215Driver(port=port, baudrate=baudrate)
        
    else:
        raise ValueError(f"不支持的舵机类型: {servo_type}")
        # 未来扩展示例:
        # elif servo_type == ServoType.RS00:
        #     from drivers.robstride.rs_driver import RSDriver
        #     return RSDriver(port=port, baudrate=baudrate)
