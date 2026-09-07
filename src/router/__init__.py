"""
命令路由器模块
处理从Server接收的各类控制命令路由
"""

from .actuator_router import ActuatorRouter

__all__ = ['ActuatorRouter']
