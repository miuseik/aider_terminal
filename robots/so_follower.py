"""
SO Follower 机器人驱动 - 支持 LX-16A 和 ST3215 总线舵机
"""

import logging
from typing import Dict, Any, Optional
from drivers.bus_servo_driver import create_servo_driver, ServoType

logger = logging.getLogger(__name__)


class SOFollowerRobotConfig:
    """SO Follower 机器人配置"""
    
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        id: str = "follower",
        use_degrees: bool = True,
        disable_torque_on_disconnect: bool = True,
        servo_type: str = "st3215",  # "lx16a" or "st3215"
        baudrate: int = 115200,
        **kwargs
    ):
        self.port = port
        self.id = id
        self.use_degrees = use_degrees
        self.disable_torque_on_disconnect = disable_torque_on_disconnect
        self.servo_type = servo_type
        self.baudrate = baudrate


class SOFollower:
    """SO Follower 机器人 - 基于总线舵机驱动"""
    
    def __init__(self, config: SOFollowerRobotConfig):
        self.config = config
        self.is_connected = False
        self.driver = None
        
        # 根据配置创建舵机驱动
        if config.servo_type.lower() == "lx16a":
            servo_type = ServoType.LX16A
            logger.info(f"🔧 创建 LX-16A 舵机驱动 (端口: {config.port})")
        else:
            servo_type = ServoType.ST3215
            logger.info(f"🔧 创建 ST3215 舵机驱动 (端口: {config.port})")
        
        self.driver = create_servo_driver(
            servo_type=servo_type,
            port=config.port,
            baudrate=config.baudrate
        )
    
    def connect(self) -> bool:
        """连接机器人"""
        try:
            success = self.driver.connect()
            if success:
                self.is_connected = True
                logger.info(f"✅ {self.config.id} 连接成功")
            else:
                logger.error(f"❌ {self.config.id} 连接失败")
            return success
        except Exception as e:
            logger.error(f"❌ {self.config.id} 连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.driver:
            self.driver.disconnect()
        self.is_connected = False
        logger.info(f"🔌 {self.config.id} 已断开")
    
    def send_action(self, action: Dict[str, float]):
        """
        发送关节角度指令
        
        Args:
            action: 关节角度字典，如 {"shoulder_pan.pos": 45.0, ...}
        """
        if not self.is_connected:
            print(f"{self.config.id}.send_action() 跳过 (未连接)----------------: {action}")
            return
        try:
            print(f"📤 [{self.config.id}] 发送动作数据----------------: {action}")
            self.driver.send_action(action, time_ms=50)
        except Exception as e:
            logger.error(f"{self.config.id} 发送指令失败----------------: {e}")
    
    def get_observation(self) -> Dict[str, Any]:
        """
        读取当前状态
        
        Returns:
            关节位置字典
        """
        if not self.is_connected:
            return {}
        
        try:
            return self.driver.get_observation()
        except Exception as e:
            logger.error(f"{self.config.id} 读取状态失败: {e}")
            return {}
    
    def is_ready(self) -> bool:
        """检查是否就绪"""
        return self.is_connected
