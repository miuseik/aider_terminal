"""
升降轴控制器 - 管理Aloha升降轴高度
"""

import logging
from drivers.aloha_base_driver import AlohaBaseDriver

logger = logging.getLogger(__name__)


class LiftController:
    """升降轴控制器"""
    
    def __init__(self, config):
        """
        初始化升降轴控制器
        
        Args:
            config: 配置信息
        """
        self.config = config
        
        # 初始化底盘驱动(升降轴集成在底盘中)
        if config.get('enable_base', False):
            self.driver = AlohaBaseDriver(config.get('base_config', {}))
        else:
            self.driver = None
    
    def connect(self) -> bool:
        """连接升降轴"""
        if self.driver:
            return self.driver.connect()
        return True
    
    def disconnect(self):
        """断开升降轴"""
        if self.driver:
            self.driver.disconnect()
    
    def set_height(self, height_mm: int) -> bool:
        """
        设置升降轴高度
        
        Args:
            height_mm: 目标高度(毫米)
            
        Returns:
            bool: 是否成功
        """
        if not self.driver:
            logger.debug("⚠️ 升降轴驱动未初始化")
            return False
        
        return self.driver.set_lift_height(height_mm)
    
    def get_height(self) -> int:
        """
        获取当前高度
        
        Returns:
            int: 当前高度(毫米), 失败返回0
        """
        if not self.driver:
            return 0
        
        height = self.driver.read_lift_height()
        return height if height is not None else 0
