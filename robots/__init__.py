"""
机器人适配器模块。

每个子目录代表一种机器人类型，封装该机器人的全部控制逻辑:
  - 机械臂 IK/FK 解算
  - 底盘运动学（轮子/升降轴）
  - 仿真可视化更新

架构原则:
  主程序 (robot_interface / control_loop) 只做流程编排和硬件通信，
  具体计算逻辑全部委托给此目录下的适配器。
"""

from .aloha.adapter import AlohaAdapter

__all__ = ["AlohaAdapter"]
