"""
SO Follower 机器人驱动 - 支持 ST3215 总线舵机
"""

import logging
from typing import Dict, Any, Optional
from drivers.feetech.st3215_driver import ST3215Driver

logger = logging.getLogger(__name__)


class SOFollowerRobotConfig:
    """SO Follower 机器人配置"""
    
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        id: str = "follower",
        use_degrees: bool = True,
        disable_torque_on_disconnect: bool = True,
        baudrate: int = 1000000,  # ST3215 默认波特率
        **kwargs
    ):
        self.port = port
        self.id = id
        self.use_degrees = use_degrees
        self.disable_torque_on_disconnect = disable_torque_on_disconnect
        self.baudrate = baudrate


class SOFollower:
    """SO Follower 机器人 - 基于总线舵机驱动"""
    
    def __init__(self, config: SOFollowerRobotConfig):
        self.config = config
        self.is_connected = False
        
        # 创建 ST3215 舵机驱动
        print(f"🔧 创建 ST3215 舵机驱动 (端口: {config.port}, 波特率: {config.baudrate})")
        self.driver = ST3215Driver(
            port=config.port,
            baudrate=config.baudrate
        )
    
    def connect(self) -> bool:
        """连接机器人"""
        try:
            success = self.driver.connect()
            if success:
                self.is_connected = True
                print(f"✅ {self.config.id} 连接成功")
            else:
                print(f"❌ {self.config.id} 连接失败")
            return success
        except Exception as e:
            print(f"❌ {self.config.id} 连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.driver:
            self.driver.disconnect()
        self.is_connected = False
        print(f"🔌 {self.config.id} 已断开")
    
    def send_action(self, action: Dict[str, float]):
        """
        发送关节角度指令
        
        Args:
            action: 关节角度字典，如 {"shoulder_pan.pos": 45.0, ...}
        """
        if not self.is_connected:
            print(f"⚠️ [{self.config.id}] 未连接，跳过发送动作")
            print(f"   数据: {action}")
            return
        
        try:
            # Print 发送的动作数据
            print(f"📤 [{self.config.id}] 发送手臂动作 → {len(action)}个关节, Port={self.config.port}")
            for joint_name, angle in action.items():
                print(f"   ├─ {joint_name}: {angle}°")
            
            # 调用底层驱动的 send_action（如果存在）
            if hasattr(self.driver, 'send_action'):
                self.driver.send_action(action, time_ms=50)
            else:
                print(f"   ⚠️ 驱动不支持 send_action 方法，跳过实际发送")
        except Exception as e:
            print(f"❌ [{self.config.id}] 发送指令失败: {e}")
    
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
            print(f"{self.config.id} 读取状态失败: {e}")
            return {}
    
    def is_ready(self) -> bool:
        """检查是否就绪"""
        return self.is_connected
    
    @property
    def bus(self):
        """暴露底层驱动，用于直接控制底盘和升降轴"""
        return self.driver
