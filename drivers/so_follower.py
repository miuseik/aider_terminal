"""
SO Follower 机器人驱动 - LeRobot SOFollower 占位符

注意: 这是空实现，用于避免安装 lerobot 依赖
实际使用时需要替换为真正的 LeRobot SOFollower
"""

import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class SOFollowerRobotConfig:
    """SO Follower 机器人配置（空实现）"""
    
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        id: str = "follower",
        use_degrees: bool = True,
        disable_torque_on_disconnect: bool = True,
        **kwargs
    ):
        self.port = port
        self.id = id
        self.use_degrees = use_degrees
        self.disable_torque_on_disconnect = disable_torque_on_disconnect


class SOFollower:
    """SO Follower 机器人（空实现）"""
    
    def __init__(self, config: SOFollowerRobotConfig):
        self.config = config
        self.is_connected = False
        logger.warning(f"⚠️ Using placeholder SOFollower for {config.id}")
    
    def connect(self) -> bool:
        """连接机器人（空实现）"""
        logger.warning(f"⚠️ SOFollower.connect() called but not implemented")
        self.is_connected = False
        return False
    
    def disconnect(self):
        """断开连接（空实现）"""
        self.is_connected = False
    
    def send_action(self, action: Dict[str, float]):
        """
        发送关节角度指令（空实现）
        
        Args:
            action: 关节角度字典，如 {"shoulder_pan.pos": 45.0, ...}
        """
        if not self.is_connected:
            logger.debug(f"SOFollower.send_action() skipped (not connected): {action}")
        else:
            logger.debug(f"SOFollower.send_action() called: {action}")
    
    def get_observation(self) -> Dict[str, Any]:
        """
        读取当前状态（空实现）
        
        Returns:
            空字典
        """
        return {}
    
    def is_ready(self) -> bool:
        """检查是否就绪"""
        return self.is_connected
