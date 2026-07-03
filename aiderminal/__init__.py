"""
aiderminal — Aider 遥操作机器人共享 Python 库。

目录结构:
    config/      全局配置（robot params, joint limits...）
    core/        控制层（control_loop, robot_interface, kinematic）
    robots/      机器人适配（aider/adapter, aloha/adapter...）
    controller/  执行器抽象控制
    drivers/     硬件驱动（actuator, camera, audio, webrtc）
    comm/        通信（api client, websocket）
    router/      路由调度
    inputs/      输入（VR, 键盘）
    utils/       工具函数
"""
