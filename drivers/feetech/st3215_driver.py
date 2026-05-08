"""
飞特 ST3215 舵机驱动 - 基于 ServoController 封装
精简接口，兼容原有调用方式
支持多串口独立控制（左臂、右臂、底盘）
"""

from typing import Optional, Dict
import numpy as np
from .servo_controller import ServoController


class ST3215Driver:
    """ST3215 舵机驱动（独立实例，支持多串口）"""
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 1000000, servo_config: Optional[Dict] = None):
        self.port = port
        self.baudrate = baudrate
        self.servo_config = servo_config
        self.is_connected = False
        # 每个实例拥有独立的 ServoController
        self.controller = ServoController(port, baudrate)
        
        # 建立 ID 到 offset 的映射（角度）
        self.id_to_offset = self._build_id_offset_map(servo_config)
    
    def _build_id_offset_map(self, servo_config: Optional[Dict]) -> Dict[int, float]:
        """从配置中构建 ID 到偏移量的映射（角度）"""
        id_to_offset = {}
        
        if not servo_config:
            return id_to_offset
        
        try:
            # 遍历所有总线配置
            for bus_name, bus_config in servo_config.items():
                if not isinstance(bus_config, dict):
                    continue
                
                for part_name, part_config in bus_config.items():
                    if not isinstance(part_config, dict):
                        continue
                    
                    for joint_name, joint_info in part_config.items():
                        if isinstance(joint_info, dict) and 'id' in joint_info:
                            servo_id = joint_info['id']
                            zero_offset_deg = joint_info.get('zero_offset', 0)
                            id_to_offset[servo_id] = zero_offset_deg
        except Exception as e:
            print(f"⚠️ 构建 ID-offset 映射失败: {e}")
        
        return id_to_offset
    
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
            print(f"⚠️ [ST3215] 未连接，跳过发送 - ID={servo_id}, Position={position}, Time={time_ms}ms")
            return False
        
        # Print 发送的指令（模拟模式）
        # print(f"📤 [ST3215] 发送位置指令 → ID={servo_id}, Position={position}, Time={time_ms}ms, Port={self.port}")
        
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
        # 应用零点偏移
        offset = self.id_to_offset.get(servo_id, 0.0)
        angle_with_offset = angle + offset
        
        # print(f"   [DEBUG] ID={servo_id}, 角度={angle}°, 偏移={offset:.2f}°, 实际={angle_with_offset:.2f}°")
        
        # ✅ 角度转脉冲值 (-180°~180° -> 0-4095)
        # 先将 -180~180 映射到 0~360，再转换为脉冲
        normalized_angle = angle_with_offset + 180  # -180~180 -> 0~360
        position = int((normalized_angle / 360.0) * 4095)
        
        # 限制位置范围在 0-4095 之间
        position = max(0, min(4095, position))
        
        if position != int(((angle_with_offset + 180) / 360.0) * 4095):
            print(f"   ⚠️ 位置已限制到有效范围: {position}")
        
        # Print 发送的指令
        # print(f"📤 [ST3215] 发送角度指令 → ID={servo_id}, Angle={angle}°, Position={position}, Time={time_ms}ms, Port={self.port}")
        
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
        设置速度(速度模式下使用)
            
        Args:
            servo_id: 舵机ID
            speed: 速度 (-1000 ~ 1000)
                   正数=顺时针,负数=逆时针
                   0=停止
        """
        if not self.is_connected:
            print(f"⚠️ [ST3215] 未连接,跳过发送 - ID={servo_id}, Speed={speed}, Port={self.port}")
            return False
            
        # Print 发送的指令
        direction = "顺时针" if speed >= 0 else "逆时针"

        try:
            # ✅ 不再每次启用扭矩，扭矩应在连接时统一启用
                
            # ST3215 速度寄存器格式:bit15=方向(0=顺,1=逆), bit0-14=速度值
            if speed >= 0:
                speed_value = speed & 0x7FFF  # 顺时针,清除方向位
            else:
                speed_value = (abs(speed) & 0x7FFF) | 0x8000  # 逆时针,设置方向位
                
            success, _ = self.controller.write_register(servo_id, 46, 2, speed_value)
            return success
        except Exception as e:
            print(f"❌ 设置速度失败: {e}")
            return False
    
    def stop(self, servo_id: int) -> bool:
        """停止舵机（速度模式下）"""
        return self.set_speed(servo_id, 0)
    
    def sync_write_velocity(self, targets: dict) -> bool:
        """
        同步写入多个舵机的速度
        
        Args:
            targets: {servo_id: speed} 字典，如 {8: 100, 9: -50, 10: 200}
        """
        if not self.is_connected:
            print(f"⚠️ [ST3215] 未连接，跳过同步速度发送 - Targets={targets}, Port={self.port}")
            return False
        
        # Print 批量发送的指令
        for servo_id, speed in targets.items():
            direction = "顺" if speed >= 0 else "逆"
            print(f"   ├─ ID={servo_id}: Speed={speed} ({direction})")
        
        try:
            for servo_id, speed in targets.items():
                self.set_speed(servo_id, speed)
            return True
        except Exception as e:
            print(f"❌ 同步写入速度失败: {e}")
            return False
    
    # ✅ 别名方法，兼容 motor_controller 的调用
    sync_write_speeds = sync_write_velocity
    
    def sync_write_positions(self, targets: Dict[int, float], time_ms: int = 50) -> bool:
        """
        同步写入多个舵机的位置（所有舵机同时开始运动）
        
        Args:
            targets: {servo_id: angle_deg} 字典，角度值（度）
            time_ms: 运动时间（毫秒），默认 50ms 实现流畅连续运动
        """
        try:
            # ✅ 关键：为每个舵机设置相同的运动时间，确保平滑移动并保持扭矩
            for servo_id, angle in targets.items():
                self.move_to_angle(servo_id, angle, time_ms)
            return True
        except Exception as e:
            print(f"❌ 同步写入位置失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def write_position(self, servo_id: int, position: int) -> bool:
        """
        直接写入位置（不指定时间，立即执行）
        
        Args:
            servo_id: 舵机ID
            position: 目标位置 (0-4095)
        """
        if not self.is_connected:
            print(f"⚠️ [ST3215] 未连接，跳过发送 - ID={servo_id}, Position={position}, Port={self.port}")
            return False
        
        # Print 发送的指令
        print(f"📤 [ST3215] 直接写入位置 → ID={servo_id}, Position={position}, Port={self.port}")
        
        try:
            success, _ = self.controller.write_register(servo_id, 42, 2, position)
            return success
        except Exception as e:
            print(f"❌ 写入位置失败: {e}")
            return False
    
    def reset_servo(self, servo_id: int) -> bool:
        """
        重置舵机（恢复出厂设置，ID 变为 1）
        
        Args:
            servo_id: 当前舵机ID
        """
        if not self.is_connected:
            return False
        try:
            # 写入复位命令到寄存器 0x1E
            success, _ = self.controller.write_register(servo_id, 0x1E, 1, 0x01)
            if success:
                print(f"✅ 舵机 ID={servo_id} 已重置，请重新上电")
            return success
        except Exception as e:
            print(f"❌ 重置舵机失败: {e}")
            return False
