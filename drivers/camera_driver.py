"""
摄像头驱动基类 - 定义摄像头驱动的统一接口
"""

import logging
from abc import abstractmethod
from typing import Optional
import numpy as np
from .base_driver import BaseDriver

logger = logging.getLogger(__name__)


class CameraDriver(BaseDriver):
    """摄像头驱动基类"""
    
    def __init__(self, config):
        """
        初始化摄像头驱动
        
        Args:
            config: 配置信息
                - camera_id: 摄像头ID (如 0 表示 /dev/video0)
                - width: 分辨率宽度
                - height: 分辨率高度
                - fps: 帧率
        """
        super().__init__(config)
        self.camera_id = config.get('camera_id', 0)
        self.width = config.get('width', 640)
        self.height = config.get('height', 480)
        self.fps = config.get('fps', 30)
    
    @abstractmethod
    def read_frame(self) -> Optional[np.ndarray]:
        """
        读取一帧图像
        
        Returns:
            np.ndarray: BGR格式图像(H x W x 3), 失败返回None
        """
        pass
    
    def get_resolution(self) -> tuple:
        """获取分辨率"""
        return (self.width, self.height)
    
    def get_fps(self) -> int:
        """获取帧率"""
        return self.fps
