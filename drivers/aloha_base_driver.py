"""
Aloha底盘驱动 - 控制3个轮子和升降轴

功能:
1. 三轮全向移动控制
2. 升降轴高度控制
"""

import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class AlohaBaseDriver:
    """Aloha底盘驱动"""
    
    def __init__(self, config):
        """
        初始化底盘驱动
        
        Args:
            config: 配置信息
                - port: 串口号
                - wheel_motor_ids: 轮子电机ID {left, back, right}
                - lift_motor_id: 升降轴电机ID
        """
        self.config = config
        self.port = config.get('port', '/dev/ttyUSB2')
        self.wheel_motor_ids = config.get('wheel_motor_ids', {
            'left': 10,
            'back': 11,
            'right': 12
        })
        self.lift_motor_id = config.get('lift_motor_id', 13)
        self.is_connected = False
    
    def connect(self) -> bool:
        """连接到底盘控制器"""
        try:
            # TODO: 实现串口连接
            logger.info(f"✅ Aloha底盘驱动已连接: {self.port}")
            self.is_connected = True
            return True
        except Exception as e:
            logger.error(f"❌ Aloha底盘连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.is_connected:
            try:
                logger.info("🔌 Aloha底盘驱动已断开")
                self.is_connected = False
            except Exception as e:
                logger.error(f"❌ 断开Aloha底盘失败: {e}")
    
    def is_ready(self) -> bool:
        """检查驱动是否就绪"""
        return self.is_connected
    
    # === 底盘控制 ===
    
    def set_wheel_speeds(self, speeds: Dict[str, float]) -> bool:
        """
        设置三个轮子的速度
        
        Args:
            speeds: {
                'left': 左轮速度(rpm),
                'back': 后轮速度(rpm),
                'right': 右轮速度(rpm)
            }
            
        Returns:
            bool: 是否成功
        """
        if not self.is_connected:
            logger.warning("⚠️ 底盘驱动未连接")
            return False
        
        try:
            left_speed = speeds.get('left', 0)
            back_speed = speeds.get('back', 0)
            right_speed = speeds.get('right', 0)
            
            # TODO: 发送速度指令到三个轮子电机
            logger.debug(f"🛞 底盘速度: L={left_speed}, B={back_speed}, R={right_speed}")
            return True
        except Exception as e:
            logger.error(f"❌ 设置底盘速度失败: {e}")
            return False
    
    def stop_wheels(self) -> bool:
        """停止所有轮子"""
        return self.set_wheel_speeds({'left': 0, 'back': 0, 'right': 0})
    
    # === 升降轴控制 ===
    
    def set_lift_height(self, height_mm: int) -> bool:
        """
        设置升降轴高度
        
        Args:
            height_mm: 目标高度(毫米), 范围 0-1000
            
        Returns:
            bool: 是否成功
        """
        if not self.is_connected:
            logger.warning("⚠️ 底盘驱动未连接")
            return False
        
        height_mm = max(0, min(1000, height_mm))
        
        try:
            # TODO: 发送高度指令到升降轴电机
            logger.debug(f"⬆️ 升降轴高度: {height_mm} mm")
            return True
        except Exception as e:
            logger.error(f"❌ 设置升降轴高度失败: {e}")
            return False
    
    def read_lift_height(self) -> Optional[int]:
        """
        读取当前升降轴高度
        
        Returns:
            int: 当前高度(毫米), 失败返回None
        """
        if not self.is_connected:
            return None
        
        try:
            # TODO: 读取升降轴位置
            logger.debug("📖 读取升降轴高度")
            return 500  # 示例返回值
        except Exception as e:
            logger.error(f"❌ 读取升降轴高度失败: {e}")
            return None
    
    def enable_motors(self) -> bool:
        """使能所有电机电机"""
        if not self.is_connected:
            return False
        try:
            logger.info("⚡ 底盘电机已使能")
            return True
        except Exception as e:
            logger.error(f"❌ 使能底盘电机失败: {e}")
            return False
    
    def disable_motors(self) -> bool:
        """禁用所有电机电机"""
        if not self.is_connected:
            return False
        try:
            logger.info("🔌 底盘电机已禁用")
            return True
        except Exception as e:
            logger.error(f"❌ 禁用底盘电机失败: {e}")
            return False
