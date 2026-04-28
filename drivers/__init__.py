"""
硬件驱动模块
提供真机电机、传感器、摄像头的底层驱动接口
"""

from .camera.camera_driver import CameraDriver

__all__ = [
    'CameraDriver'
]
