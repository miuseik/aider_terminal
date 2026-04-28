"""
机器人模块
包含各类机器人硬件抽象和封装
"""

from .so_follower import SOFollower, SOFollowerRobotConfig

__all__ = [
    'SOFollower',
    'SOFollowerRobotConfig'
]
