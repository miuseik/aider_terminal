"""
Aloha 机器人适配器。

统一封装:
  - SO100 IK/FK 解算（双机械臂）- 为 Aloha 服务
  - SO100 → Aloha 关节角度映射
  - 底盘麦克纳姆轮运动学
  - 升降轴高度积分
  - PyBullet 仿真可视化更新
"""

from .adapter import AlohaAdapter

__all__ = ["AlohaAdapter"]
