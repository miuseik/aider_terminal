"""
Feetech STS3215 舵机驱动 - 基于 pypot (功能全面)

直接从 Open Duck Mini Runtime 项目复制
原始文件: scripts/configure_motor.py

特点:
- Pollen Robotics 开发的纯 Python 库
- 功能全面，易于使用
- 支持配置和调试
- 适用于非实时场景
"""

from pypot.feetech import FeetechSTS3215IO
import time
from typing import Optional, Dict, List


class PypotConfigurator:
    """
    Pypot 配置器 - 直接来自 Open Duck Mini
    
    用于配置舵机参数（ID、PID、模式等）
    """
    
    DEFAULT_ID = 1  # A brand new motor should have id 1
    
    def __init__(self, port: str = "/dev/ttyACM0"):
        """
        初始化配置器
        
        Args:
            port: 串口号
        """
        self.port = port
        self.io = FeetechSTS3215IO(port)
        self.current_id = self.DEFAULT_ID
        
        print(f"✅ Pypot 配置器初始化成功: {port}")
    
    def scan(self):
        """
        扫描在线舵机
        
        Returns:
            int: 找到的第一个舵机ID，未找到返回 None
        """
        for i in range(255):
            print(f"scanning for id {i} ...")
            try:
                self.io.get_present_position([i])
                print(f"Found motor with id {i}")
                return i
            except Exception:
                pass
        return None
    
    def detect_motor(self):
        """检测舵机并获取当前ID"""
        try:
            self.io.get_present_position([self.DEFAULT_ID])
            self.current_id = self.DEFAULT_ID
        except Exception:
            print(f"Could not find motor with default id ({self.DEFAULT_ID}). Scanning for motor ...")
            res = self.scan()
            if res is not None:
                self.current_id = res
            else:
                print("Could not find motor. Exiting ...")
                raise Exception("Motor not found")
        
        return self.current_id
    
    def configure_motor(self, new_id: int, kp: int = 32, ki: int = 0, kd: int = 0):
        """
        配置舵机参数
        
        Args:
            new_id: 新ID
            kp: P 系数
            ki: I 系数
            kd: D 系数
        """
        # 检测当前ID
        current_id = self.detect_motor()
        
        # 读取当前参数
        current_kp = self.io.get_P_coefficient([current_id])
        current_ki = self.io.get_I_coefficient([current_id])
        current_kd = self.io.get_D_coefficient([current_id])
        max_acceleration = self.io.get_maximum_acceleration([current_id])
        acceleration = self.io.get_acceleration([current_id])
        mode = self.io.get_mode([current_id])
        
        # 配置参数
        self.io.set_lock({current_id: 0})
        self.io.set_mode({current_id: 0})
        self.io.set_maximum_acceleration({current_id: 0})
        self.io.set_acceleration({current_id: 0})
        self.io.set_P_coefficient({current_id: kp})
        self.io.set_I_coefficient({current_id: ki})
        self.io.set_D_coefficient({current_id: kd})
        self.io.change_id({current_id: new_id})
        
        self.current_id = new_id
        
        time.sleep(1)
        
        self.io.set_goal_position({self.current_id: 0})
        
        time.sleep(1)
        
        print("===")
        print("Done configuring motor.")
        print(f"Motor id: {self.current_id}")
        print(f"P coefficient : {self.io.get_P_coefficient([self.current_id])}")
        print(f"I coefficient : {self.io.get_I_coefficient([self.current_id])}")
        print(f"D coefficient : {self.io.get_D_coefficient([self.current_id])}")
        print(f"acceleration: {self.io.get_acceleration([self.current_id])}")
        print(f"max_acceleration: {self.io.get_maximum_acceleration([self.current_id])}")
        print(f"mode: {self.io.get_mode([self.current_id])}")
        print("===")


# ==================== 兼容 motor_controller_new.py 的适配器 ====================

class PypotDriver:
    """
    Pypot 驱动适配器
    
    封装 FeetechSTS3215IO，提供与 motor_controller_new.py 兼容的接口
    """
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 1000000):
        """初始化驱动"""
        self.port = port
        self.baudrate = baudrate
        self.io = None
        self.is_connected = False
        
        print(f"🔧 Pypot 驱动初始化: {port} @ {baudrate}")
    
    def connect(self) -> bool:
        """连接舵机"""
        try:
            self.io = FeetechSTS3215IO(self.port)
            self.is_connected = True
            print(f"✅ Pypot 连接成功: {self.port}")
            return True
        except Exception as e:
            print(f"❌ Pypot 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.is_connected = False
        print(f"🔌 Pypot 已断开: {self.port}")
    
    def ping(self, servo_id: int) -> bool:
        """Ping 检测舵机"""
        if not self.is_connected:
            return False
        
        try:
            pos = self.io.get_present_position([servo_id])
            return pos is not None and len(pos) > 0
        except:
            return False
    
    def scan_servos(self, start_id: int = 1, end_id: int = 253) -> List[int]:
        """扫描指定范围内的在线舵机"""
        if not self.is_connected:
            return []
        
        found_ids = []
        
        for servo_id in range(start_id, end_id + 1):
            try:
                pos = self.io.get_present_position([servo_id])
                if pos is not None and len(pos) > 0:
                    found_ids.append(servo_id)
                    print(f"✅ 发现舵机 ID={servo_id}")
            except:
                pass
            
            time.sleep(0.01)
        
        print(f"✅ 扫描完成，找到 {len(found_ids)} 个舵机")
        return found_ids
    
    def set_position(self, servo_id: int, angle_deg: float, time_ms: int = 500) -> bool:
        """
        设置单个舵机位置
        
        Args:
            servo_id: 舵机ID
            angle_deg: 目标角度（度）
            time_ms: 到达时间（毫秒），pypot 会自动处理
        """
        if not self.is_connected:
            return False
        
        try:
            # pypot 直接使用角度（度）
            self.io.set_goal_position({servo_id: angle_deg})
            return True
        except Exception as e:
            print(f"❌ 设置位置失败 ID={servo_id}: {e}")
            return False
    
    def set_positions(self, targets: dict, time_ms: int = 500) -> bool:
        """
        批量设置多个舵机位置
        
        Args:
            targets: {servo_id: angle_deg}
        """
        if not self.is_connected:
            return False
        
        try:
            # pypot 使用字典格式 {id: position}
            self.io.set_goal_position(targets)
            print(f"✅ 批量设置 {len(targets)} 个舵机位置")
            return True
        except Exception as e:
            print(f"❌ 批量设置位置失败: {e}")
            return False
    
    def set_speed(self, servo_id: int, speed: int) -> bool:
        """设置速度（连续旋转）"""
        if not self.is_connected:
            return False
        
        try:
            self.io.set_mode({servo_id: 1})  # 1=速度模式
            self.io.set_goal_speed({servo_id: speed})
            return True
        except Exception as e:
            print(f"❌ 设置速度失败 ID={servo_id}: {e}")
            return False
    
    def set_velocity_mode(self, servo_id: int) -> bool:
        """切换到速度模式"""
        if not self.is_connected:
            return False
        try:
            self.io.set_mode({servo_id: 1})
            return True
        except:
            return False
    
    def set_position_mode(self, servo_id: int) -> bool:
        """切换到位置模式"""
        if not self.is_connected:
            return False
        try:
            self.io.set_mode({servo_id: 0})
            return True
        except:
            return False
    
    def read_position(self, servo_id: int):
        """读取当前位置（度）"""
        if not self.is_connected:
            return None
        try:
            positions = self.io.get_present_position([servo_id])
            if positions and len(positions) > 0:
                return positions[0]
            return None
        except Exception as e:
            print(f"❌ 读取位置失败: {e}")
            return None
    
    def read_velocity(self, servo_id: int):
        """读取当前速度"""
        if not self.is_connected:
            return None
        try:
            velocities = self.io.get_present_velocity([servo_id])
            if velocities and len(velocities) > 0:
                return velocities[0]
            return None
        except:
            return None
    
    def read_status(self, servo_id: int):
        """读取舵机状态"""
        if not self.is_connected:
            return None
        try:
            position = self.read_position(servo_id)
            velocity = self.read_velocity(servo_id)
            return {
                'position': position if position else 0.0,
                'velocity': velocity if velocity else 0.0,
                'current': 0.0,
                'temperature': 0.0,
                'voltage': 0.0,
                'load': 0.0,
                'moving': False
            }
        except:
            return None
    
    def set_torque(self, servo_id: int, enable: bool) -> bool:
        """启用/禁用扭矩"""
        if not self.is_connected:
            return False
        try:
            if enable:
                self.io.enable_torque({servo_id: True})
            else:
                self.io.disable_torque({servo_id: True})
            return True
        except Exception as e:
            print(f"❌ 设置扭矩失败: {e}")
            return False
    
    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改舵机ID"""
        if not self.is_connected:
            return False
        try:
            self.io.set_lock({old_id: 0})  # 解锁
            self.io.change_id({old_id: new_id})
            print(f"✅ ID 修改成功: {old_id} → {new_id}")
            return True
        except Exception as e:
            print(f"❌ ID 修改失败: {e}")
            return False
    
    def configure_servo(self, servo_id: int, kp: int = 32, ki: int = 0, kd: int = 0) -> bool:
        """配置舵机参数（PID、模式等）"""
        if not self.is_connected:
            return False
        try:
            self.io.set_lock({servo_id: 0})
            self.io.set_mode({servo_id: 0})
            self.io.set_maximum_acceleration({servo_id: 0})
            self.io.set_acceleration({servo_id: 0})
            self.io.set_P_coefficient({servo_id: kp})
            self.io.set_I_coefficient({servo_id: ki})
            self.io.set_D_coefficient({servo_id: kd})
            print(f"✅ 舵机配置完成 ID={servo_id}, PID=({kp}, {ki}, {kd})")
            return True
        except Exception as e:
            print(f"❌ 配置舵机失败: {e}")
            return False
    
    def read_pid(self, servo_id: int):
        """读取 PID 参数"""
        if not self.is_connected:
            return None
        try:
            kp = self.io.get_P_coefficient([servo_id])[0]
            ki = self.io.get_I_coefficient([servo_id])[0]
            kd = self.io.get_D_coefficient([servo_id])[0]
            return {'kp': kp, 'ki': ki, 'kd': kd}
        except:
            return None
    
    def sync_write_positions(self, targets: dict, time_ms: int = 500) -> bool:
        """同步写入位置"""
        return self.set_positions(targets, time_ms)
    
    def get_model(self, servo_id: int) -> str:
        """获取舵机型号"""
        return "STS3215"
