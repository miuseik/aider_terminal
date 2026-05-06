"""
舵机控制处理器 - 处理所有舵机相关操作
"""

import logging
from typing import Dict, Any, Optional
from drivers.bus_servo_driver import ServoType, create_servo_driver

logger = logging.getLogger(__name__)


class ServoHandler:
    """舵机处理器 - 封装所有舵机操作"""
    
    @staticmethod
    def set_angle(motor_controller, servo_id: int, angle: float, port: str = '/dev/ttyACM0') -> bool:
        """设置舵机角度"""
        print(f"🎯 设置舵机 ID={servo_id} 角度={angle}° (端口: {port})")
        
        if hasattr(motor_controller, 'set_servo_angle'):
            success = motor_controller.set_servo_angle(
                port=port,
                servo_id=servo_id,
                angle=angle
            )
            if success:
                print(f"✅ 舵机 {servo_id} 角度设置成功")
            else:
                print(f"❌ 舵机 {servo_id} 角度设置失败")
            return success
        else:
            print("⚠️ motor_controller 没有 set_servo_angle 方法")
            return False
    
    @staticmethod
    def reset(servo_id: int, port: str = '/dev/ttyACM0') -> bool:
        """重置舵机"""
        print(f"🔄 重置舵机 ID={servo_id} (端口: {port})")
        
        try:
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.reset_servo(servo_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 重置成功，请断电重启")
            else:
                print(f"❌ 舵机 {servo_id} 重置失败")
            
            return success
        except Exception as e:
            print(f"❌ 重置舵机异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    @staticmethod
    def set_id(old_id: int, new_id: int, port: str = '/dev/ttyACM0') -> bool:
        """修改舵机ID"""
        print(f"🔢 修改舵机 ID: {old_id} → {new_id} (端口: {port})")
        
        try:
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_id(old_id, new_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 ID 从 {old_id} 改为 {new_id}，请断电重启")
            else:
                print(f"❌ 舵机 ID 修改失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置舵机ID异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    @staticmethod
    def set_position_mode(servo_id: int, port: str = '/dev/ttyACM0') -> bool:
        """设置为位置模式"""
        print(f"📍 设置舵机 ID={servo_id} 为位置模式 (端口: {port})")
        
        try:
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_position_mode(servo_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到位置模式")
            else:
                print(f"❌ 舵机 {servo_id} 模式切换失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置位置模式异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    @staticmethod
    def set_speed_mode(servo_id: int, port: str = '/dev/ttyACM0') -> bool:
        """设置为速度模式"""
        print(f"⚡ 设置舵机 ID={servo_id} 为速度模式 (端口: {port})")
        
        try:
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_velocity_mode(servo_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到速度模式")
            else:
                print(f"❌ 舵机 {servo_id} 模式切换失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置速度模式异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    @staticmethod
    def set_speed(servo_id: int, speed: int, port: str = '/dev/ttyACM0') -> bool:
        """设置舵机速度"""
        print(f"🚀 设置舵机 ID={servo_id} 速度={speed} (端口: {port})")
        
        try:
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_speed(servo_id, int(speed))
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 速度设置为 {speed}")
            else:
                print(f"❌ 舵机 {servo_id} 速度设置失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置舵机速度异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    @staticmethod
    def get_info(servo_id: int, port: str = '/dev/ttyACM0') -> Optional[Dict[str, Any]]:
        """获取舵机信息"""
        print(f"📊 获取舵机 ID={servo_id} 信息 (端口: {port})")
        
        try:
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return None
            
            # 读取舵机寄存器
            position = driver.controller.read_register(servo_id, 56, 2)  # 位置 (修正为56)
            voltage_raw = driver.controller.read_register(servo_id, 62, 1)  # 电压 (1字节)
            temperature = driver.controller.read_register(servo_id, 63, 1)  # 温度 (1字节)
            current = driver.controller.read_register(servo_id, 69, 2)      # 电流 (2字节)
            mode = driver.controller.read_register(servo_id, 33, 1)
            torque = driver.controller.read_register(servo_id, 40, 1)
            
            driver.disconnect()
            
            # 检查是否读取成功
            if position is None:
                print(f"❌ 舵机 {servo_id} 通信失败")
                return None
            
            # 转换数据
            angle = (position / 4095.0 * 360.0) - 180.0
            voltage = voltage_raw / 10.0 if voltage_raw else 0.0
            
            if temperature is None:
                temperature = 0
            
            if current is None:
                current = 0
            
            if mode is None:
                mode = 0
            
            if torque is None:
                torque = 0
            
            info_data = {
                "servo_id": servo_id,
                "position": position,
                "angle": round(angle, 1),
                "voltage": round(voltage, 1),
                "temperature": temperature,
                "current": current,
                "mode": mode,
                "torque_enabled": torque == 1
            }
            
            print(f"✅ 舵机 {servo_id} 信息: {info_data}")
            return info_data
            
        except Exception as e:
            print(f"❌ 获取舵机信息异常: {e}")
            import traceback
            print(traceback.format_exc())
            return None
