"""
aider_camera — 独立摄像头驱动包 (零 ROS 依赖)

用法:
    from aider_camera import OpenCVCameraDriver
    cam = OpenCVCameraDriver({"index_or_path": 0, "width": 640, "height": 480})
    cam.connect()
    frame = cam.read()
    cam.disconnect()
"""
from .opencv_camera_driver import OpenCVCameraDriver

__all__ = ["OpenCVCameraDriver"]
