"""
驱动基类 - 定义所有硬件驱动的统一接口

所有具体驱动(达妙/飞特/摄像头等)都应继承此类
"""

import logging
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class BaseDriver(ABC):
    """硬件驱动基类"""
    
    def __init__(self, config):
        """
        初始化驱动
        
        Args:
            config: 驱动配置信息
        """
        self.config = config
        self.is_connected = False
    
    @abstractmethod
    def connect(self) -> bool:
        """
        连接硬件
        
        Returns:
            bool: 是否成功
        """
        pass
    
    @abstractmethod
    def disconnect(self):
        """断开硬件连接"""
        pass
    
    @abstractmethod
    def is_ready(self) -> bool:
        """
        检查硬件是否就绪
        
        Returns:
            bool: 是否就绪
        """
        pass
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
        return False
