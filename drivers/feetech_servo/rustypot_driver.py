"""
Feetech STS3215 舵机驱动 - 基于 rustypot (高性能)

直接从 Open Duck Mini Runtime 项目复制
原始文件: mini_bdx_runtime/rustypot_position_hwi.py

特点:
- 基于 Rust 实现的高性能 Python 绑定
- 支持 ~1kHz 控制频率
- 批量同步写入
- 适用于实时控制场景
"""

import time
import numpy as np
import rustypot


class RustypotHWI:
    """
    硬件接口层 - 直接来自 Open Duck Mini
    
    注意: 使用弧度作为单位
    """
    
    def __init__(self, usb_port: str = "/dev/ttyACM0"):
        """
        初始化硬件接口
        
        Args:
            usb_port: 串口号
        """
        self.usb_port = usb_port
        
        # 关节映射 (示例配置，可根据需要修改)
        self.joints = {
            "joint_1": 1,
            "joint_2": 2,
            "joint_3": 3,
            "joint_4": 4,
            "joint_5": 5,
            "joint_6": 6,
        }
        
        # 零点位置
        self.zero_pos = {joint: 0 for joint in self.joints.keys()}
        
        # 初始位置 (弧度)
        self.init_pos = {joint: 0 for joint in self.joints.keys()}
        
        # 关节偏移量
        self.joints_offsets = {joint: 0 for joint in self.joints.keys()}
        
        # PID 参数
        self.kps = np.ones(len(self.joints)) * 32  # default kp
        self.kds = np.ones(len(self.joints)) * 0   # default kd
        self.low_torque_kps = np.ones(len(self.joints)) * 2
        
        # 初始化 rustypot
        self.io = rustypot.feetech(usb_port, 1000000)
        
        print(f"✅ Rustypot HWI 初始化成功: {usb_port}")
    
    def set_kps(self, kps):
        """设置所有关节的 Kp"""
        self.kps = kps
        self.io.set_kps(list(self.joints.values()), self.kps)
    
    def set_kds(self, kds):
        """设置所有关节的 Kd"""
        self.kds = kds
        self.io.set_kds(list(self.joints.values()), self.kds)
    
    def set_kp(self, id, kp):
        """设置单个关节的 Kp"""
        self.io.set_kps([id], [kp])
    
    def turn_on(self):
        """开启扭矩并移动到初始位置"""
        self.io.set_kps(list(self.joints.values()), self.low_torque_kps)
        print("turn on : low KPS set")
        time.sleep(1)
        
        self.set_position_all(self.init_pos)
        print("turn on : init pos set")
        
        time.sleep(1)
        
        self.io.set_kps(list(self.joints.values()), self.kps)
        print("turn on : high kps")
    
    def turn_off(self):
        """关闭扭矩"""
        self.io.disable_torque(list(self.joints.values()))
    
    def set_position(self, joint_name, pos):
        """
        设置单个关节位置
        
        Args:
            joint_name: 关节名称
            pos: 位置（弧度）
        """
        id = self.joints[joint_name]
        pos = pos + self.joints_offsets[joint_name]
        self.io.write_goal_position([id], [pos])
    
    def set_position_all(self, joints_positions):
        """
        批量设置所有关节位置
        
        Args:
            joints_positions: 字典 {joint_name: position_rad}
        
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: position + self.joints_offsets[joint]
            for joint, position in joints_positions.items()
        }
        
        self.io.write_goal_position(
            list(self.joints.values()), list(ids_positions.values())
        )
    
    def get_present_positions(self, ignore=[]):
        """
        读取当前位置
        
        Returns:
            numpy array: 当前位置数组（弧度）
        """
        try:
            present_positions = self.io.read_present_position(
                list(self.joints.values())
            )
        except Exception as e:
            print(e)
            return None
        
        present_positions = [
            pos - self.joints_offsets[joint]
            for joint, pos in zip(self.joints.keys(), present_positions)
            if joint not in ignore
        ]
        return np.array(np.around(present_positions, 3))
    
    def get_present_velocities(self, rad_s=True, ignore=[]):
        """
        读取当前速度
        
        Args:
            rad_s: True=rad/s, False=rev/min
            
        Returns:
            numpy array: 当前速度数组
        """
        try:
            present_velocities = self.io.read_present_velocity(
                list(self.joints.values())
            )
        except Exception as e:
            print(e)
            return None
        
        present_velocities = [
            vel
            for joint, vel in zip(self.joints.keys(), present_velocities)
            if joint not in ignore
        ]
        
        return np.array(np.around(present_velocities, 3))


# ==================== 兼容 motor_controller_new.py 的适配器 ====================

class RustypotDriver:
    """
    Rustypot 驱动适配器
    
    封装 RustypotHWI，提供与 motor_controller_new.py 兼容的接口
    """
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 1000000):
        """初始化驱动"""
        self.port = port
        self.baudrate = baudrate
        self.hwi = None
        self.is_connected = False
        
        print(f"🔧 Rustypot 驱动初始化: {port} @ {baudrate}")
    
    def connect(self) -> bool:
        """连接舵机"""
        try:
            self.hwi = RustypotHWI(usb_port=self.port)
            self.is_connected = True
            print(f"✅ Rustypot 连接成功: {self.port}")
            return True
        except Exception as e:
            print(f"❌ Rustypot 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.hwi:
            try:
                self.hwi.turn_off()
            except:
                pass
        self.is_connected = False
        print(f"🔌 Rustypot 已断开: {self.port}")
    
    def ping(self, servo_id: int) -> bool:
        """Ping 检测舵机"""
        if not self.is_connected:
            return False
        
        try:
            pos = self.hwi.io.read_present_position([servo_id])
            return pos is not None and len(pos) > 0
        except:
            return False
    
    def set_position(self, servo_id: int, angle_deg: float, time_ms: int = 500) -> bool:
        """
        设置单个舵机位置
        
        Args:
            servo_id: 舵机ID
            angle_deg: 目标角度（度）
            time_ms: 到达时间（毫秒）
        """
        if not self.is_connected:
            return False
        
        try:
            # 角度转弧度
            angle_rad = angle_deg * np.pi / 180.0
            self.hwi.io.write_goal_position([servo_id], [angle_rad])
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
            ids = list(targets.keys())
            angles_rad = [angle * np.pi / 180.0 for angle in targets.values()]
            self.hwi.io.write_goal_position(ids, angles_rad)
            print(f"✅ 同步设置 {len(ids)} 个舵机位置")
            return True
        except Exception as e:
            print(f"❌ 批量设置位置失败: {e}")
            return False
    
    def set_speed(self, servo_id: int, speed: int) -> bool:
        """设置速度（连续旋转）"""
        if not self.is_connected:
            return False
        
        try:
            self.hwi.io.write_control_mode([servo_id], [1])  # 1=速度模式
            self.hwi.io.write_goal_speed([servo_id], [speed])
            return True
        except Exception as e:
            print(f"❌ 设置速度失败: {e}")
            return False
    
    def set_velocity_mode(self, servo_id: int) -> bool:
        """切换到速度模式"""
        if not self.is_connected:
            return False
        try:
            self.hwi.io.write_control_mode([servo_id], [1])
            return True
        except:
            return False
    
    def set_position_mode(self, servo_id: int) -> bool:
        """切换到位置模式"""
        if not self.is_connected:
            return False
        try:
            self.hwi.io.write_control_mode([servo_id], [0])
            return True
        except:
            return False
    
    def read_position(self, servo_id: int):
        """读取当前位置（度）"""
        if not self.is_connected:
            return None
        try:
            positions = self.hwi.io.read_present_position([servo_id])
            if positions and len(positions) > 0:
                return positions[0] * 180.0 / np.pi
            return None
        except:
            return None
    
    def read_velocity(self, servo_id: int):
        """读取当前速度"""
        if not self.is_connected:
            return None
        try:
            velocities = self.hwi.io.read_present_velocity([servo_id])
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
                self.hwi.io.enable_torque([servo_id])
            else:
                self.hwi.io.disable_torque([servo_id])
            return True
        except Exception as e:
            print(f"❌ 设置扭矩失败: {e}")
            return False
    
    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改舵机ID"""
        if not self.is_connected:
            return False
        try:
            self.hwi.io.write_lock([old_id], [0])  # 解锁
            self.hwi.io.write_id([old_id], [new_id])
            print(f"✅ ID 修改成功: {old_id} → {new_id}")
            return True
        except Exception as e:
            print(f"❌ ID 修改失败: {e}")
            return False
    
    def sync_write_positions(self, targets: dict, time_ms: int = 500) -> bool:
        """同步写入位置"""
        return self.set_positions(targets, time_ms)
    
    def get_model(self, servo_id: int) -> str:
        """获取舵机型号"""
        return "STS3215"
