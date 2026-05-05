"""
硬件驱动模块
提供真机电机、传感器、摄像头的底层驱动接口
"""

from .camera.opencv_camera_driver import OpenCVCameraDriver

__all__ = [
    'OpenCVCameraDriver'
]
