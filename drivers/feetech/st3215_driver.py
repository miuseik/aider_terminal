"""
飞特 ST3215 舵机驱动 - 基于 ServoController 封装
精简接口，兼容原有调用方式
支持多串口独立控制（左臂、右臂、底盘）
"""

from typing import Optional
from .servo_controller import ServoController


class ST3215Driver:
    """ST3215 舵机驱动（独立实例，支持多串口）"""
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.is_connected = False
        # 每个实例拥有独立的 ServoController
        self.controller = ServoController(port, baudrate)
    
    def connect(self) -> bool:
        """连接舵机"""
        success, _ = self.controller.connect()
        self.is_connected = success
        return success
    
    def disconnect(self):
        """断开连接"""
        self.controller.disconnect()
        self.is_connected = False
    
    def ping(self, servo_id: int) -> bool:
        """Ping舵机"""
        if not self.is_connected:
            return False
        success, _ = self.controller.ping(servo_id)
        return success
    
    def get_position(self, servo_id: int) -> Optional[int]:
        """读取当前位置"""
        if not self.is_connected:
            return None
        return self.controller.read_position(servo_id)
    
    def set_position(self, servo_id: int, position: int, time_ms: int = 500) -> bool:
        """设置目标位置"""
        if not self.is_connected:
            return False
        success, _ = self.controller.set_position(servo_id, position, time_ms)
        return success
    
    def set_torque(self, servo_id: int, enable: bool) -> bool:
        """启用/禁用扭矩"""
        if not self.is_connected:
            return False
        success, _ = self.controller.set_torque(servo_id, enable)
        return success
    
    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改舵机ID"""
        if not self.is_connected:
            return False
        success, _ = self.controller.change_servo_id(old_id, new_id)
        return success
    
    def move_to_angle(self, servo_id: int, angle: float, time_ms: int = 500) -> bool:
        """便捷方法：角度转位置并移动"""
        # 角度转脉冲值 (0-360° -> 0-4095)
        position = int((angle / 360.0) * 4095)
        return self.set_position(servo_id, position, time_ms)
    
    def set_velocity_mode(self, servo_id: int) -> bool:
        """设置为速度模式（轮式模式，连续旋转）"""
        if not self.is_connected:
            return False
        try:
            success, _ = self.controller.write_register(servo_id, 33, 1, 1)  # work_mode = 1
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到速度模式")
            return success
        except Exception as e:
            print(f"❌ 切换模式失败: {e}")
            return False
    
    def set_position_mode(self, servo_id: int) -> bool:
        """设置为位置模式（默认）"""
        if not self.is_connected:
            return False
        try:
            success, _ = self.controller.write_register(servo_id, 33, 1, 0)  # work_mode = 0
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到位置模式")
            return success
        except Exception as e:
            print(f"❌ 切换模式失败: {e}")
            return False
    
    def set_speed(self, servo_id: int, speed: int) -> bool:
        """
        设置速度（速度模式下使用）
        
        Args:
            servo_id: 舵机ID
            speed: 速度 (-1000 ~ 1000)
                   正数=顺时针，负数=逆时针
                   0=停止
        """
        if not self.is_connected:
            return False
        try:
            # 写入 goal_speed 寄存器 (地址46, 2字节)
            self.controller.write_register(servo_id, 46, 2, abs(speed))
            
            # 设置方向（通过 goal_position 的高位）
            if speed < 0:
                self.controller.write_register(servo_id, 42, 2, 0x8000)  # 逆时针
            else:
                self.controller.write_register(servo_id, 42, 2, 0)  # 顺时针
            
            return True
        except Exception as e:
            print(f"❌ 设置速度失败: {e}")
            return False
    
    def stop(self, servo_id: int) -> bool:
        """停止舵机（速度模式下）"""
        return self.set_speed(servo_id, 0)
