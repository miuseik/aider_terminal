"""
aider_camera — 独立摄像头驱动包 (零 ROS 依赖)

用法:
    from aider_camera import OpenCVCameraDriver, AstraCameraDriver
    cam = OpenCVCameraDriver({"index_or_path": 0, "width": 640, "height": 480})
    cam.connect()
    frame = cam.read()
    cam.disconnect()

    # Astra RGB-D (Orbbec, pyorbbecsdk2)
    astra = AstraCameraDriver({"color_w": 640, "color_h": 480, "align": True})
    astra.connect()
    f = astra.read()   # {"color": ..., "depth": ..., "depth_scale": ...}
    astra.disconnect()
"""
from .opencv_camera_driver import OpenCVCameraDriver
from .astra_camera_driver import AstraCameraDriver

__all__ = ["OpenCVCameraDriver", "AstraCameraDriver"]
