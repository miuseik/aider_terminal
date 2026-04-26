"""
底盘控制器 - 管理Aloha底盘的三轮全向移动
"""

import logging
from typing import Dict
from drivers.aloha_base_driver import AlohaBaseDriver

logger = logging.getLogger(__name__)


class BaseController:
    """底盘控制器"""
    
    def __init__(self, config):
        """
        初始化底盘控制器
        
        Args:
            config: 配置信息
        """
        self.config = config
        
        # 初始化底盘驱动
        if config.get('enable_base', False):
            self.driver = AlohaBaseDriver(config.get('base_config', {}))
        else:
            self.driver = None
    
    def connect(self) -> bool:
        """连接底盘"""
        if self.driver:
            return self.driver.connect()
        return True
    
    def disconnect(self):
        """断开底盘"""
        if self.driver:
            self.driver.disconnect()
    
    def set_wheel_speeds(self, speeds: Dict[str, float]) -> bool:
        """
        设置三个轮子的速度
        
        Args:
            speeds: {'left': rpm, 'back': rpm, 'right': rpm}
            
        Returns:
            bool: 是否成功
        """
        if not self.driver:
            logger.debug("⚠️ 底盘驱动未初始化")
            return False
        
        return self.driver.set_wheel_speeds(speeds)
    
    def stop(self) -> bool:
        """停止底盘"""
        if not self.driver:
            return False
        return self.driver.stop_wheels()
