"""
PyBullet 可视化器工厂。
根据机器人类型返回对应的可视化器实例。

每种机器人有自己独立的可视化器实现:
  - robots/aider/visualizer.py → AiderVisualizer (8-DOF, 4轮底盘)
  - robots/aloha/visualizer.py → AlohaVisualizer (6-DOF SO100, Aloha基底)
"""

from robots.aider.visualizer import AiderVisualizer
from robots.aloha.visualizer import AlohaVisualizer


def create_visualizer(robot_type: str, urdf_path: str, use_gui: bool = True,
                      log_level: str = "warning", aloha_urdf_path: str = None):
    """根据机器人类型创建对应的 PyBullet 可视化器。

    Args:
        robot_type: 'aider' 或 'aloha'
        urdf_path: URDF 文件路径（Aider 完整 URDF 或 SO100 臂 URDF）
        use_gui: 是否使用 GUI 模式
        log_level: 日志级别
        aloha_urdf_path: Aloha 基底 URDF 路径（仅 Aloha 使用）
    """
    if robot_type == "aloha":
        return AlohaVisualizer(
            urdf_path=urdf_path,
            use_gui=use_gui,
            log_level=log_level,
            aloha_urdf_path=aloha_urdf_path,
        )
    else:
        return AiderVisualizer(
            urdf_path=urdf_path,
            use_gui=use_gui,
            log_level=log_level,
        )


# 向后兼容别名
PyBulletVisualizer = create_visualizer
