"""
RobStride 电机驱动 - 基于官方 robstride_dynamics SDK
提供与原有 API 兼容的接口，但使用官方 SDK 实现
"""

import time
from typing import Optional, Dict, List
from dataclasses import dataclass

# 导入官方 SDK（已集成到项目中）
from .robstride_dynamics import RobstrideBus, Motor as RSDMotor, ParameterType

@dataclass
class MotorState:
    """电机状态数据类"""
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    enabled: bool = False

class RobStrideOfficialDriver:
    """
    RobStride 电机驱动 - 使用官方 SDK
    保持与原 API 的兼容性
    """
    
    def __init__(self, can_interface: str = "can0", host_can_id: int = 0xFF):
        self.can_interface = can_interface
        self.host_can_id = host_can_id
        self._bus: Optional[RobstrideBus] = None
        self._motors: Dict[str, RSDMotor] = {}
        self._states: Dict[int, MotorState] = {}
        self._last_targets: Dict[int, dict] = {}  # ✅ 记录上次控制指令 {motor_id: {position, kp, kd, ...}}
        self._connected = False
        
    def add_motor(self, motor_id: int, model: str = "rs-00"):
        """添加电机到驱动"""
        motor_key = f"motor_{motor_id}"
        rsd_motor = RSDMotor(id=motor_id, model=model)
        self._motors[motor_key] = rsd_motor
        
        # 初始化状态
        self._states[motor_id] = MotorState()
        
    def connect(self) -> bool:
        """连接到 CAN 总线"""
        try:
            self._bus = RobstrideBus(
                channel=self.can_interface,
                motors=self._motors,
                bitrate=1000000
            )
            self._bus.connect()
            self._connected = True
            return True
        except Exception as e:
            print(f"❌ RobStride 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self._bus and self._connected:
            try:
                # 禁用所有电机
                for motor_key in self._motors.keys():
                    self._bus.disable(motor_key)
            except:
                pass
            self._bus.disconnect()
            self._connected = False
    
    def enable_motor(self, motor_id: int) -> bool:
        """使能电机"""
        if not self._connected:
            return False
        try:
            motor_key = f"motor_{motor_id}"
            self._bus.enable(motor_key)
            self._states[motor_id].enabled = True
            return True
        except Exception as e:
            return False
    
    def disable_motor(self, motor_id: int) -> bool:
        """禁用电机"""
        if not self._connected:
            return False
        try:
            motor_key = f"motor_{motor_id}"
            self._bus.disable(motor_key)
            self._states[motor_id].enabled = False
            return True
        except Exception as e:
            return False
    
    def send_motion_control(self, motor_id: int, position: float, velocity: float = 0.0,
                          kp: float = 80.0, kd: float = 4.0, torque: float = 0.0) -> bool:
        """发送运动控制指令(记录目标位置)"""
        if not self._connected:
            return False
        try:
            motor_key = f"motor_{motor_id}"
            # ✅ 记录上次目标位置
            self._last_targets[motor_id] = {
                'position': position,
                'velocity': velocity,
                'kp': kp,
                'kd': kd,
                'torque': torque
            }
            
            # 发送控制指令
            self._bus.write_operation_frame(
                motor_key,
                position=position,
                velocity=velocity,
                kp=kp,
                kd=kd,
                torque=torque
            )
            
            # ✅ 立即读取响应，确认指令已接收
            pos, vel, tor, temp = self._bus.read_operation_frame(motor_key)
            return True
        except Exception as e:
            print(f"发送运动控制指令失败: {e}")
            return False
    
    def set_speed(self, motor_id: int, speed_raw: int) -> bool:
        """
        设置电机速度(兼容接口)
            
        Args:
            motor_id: 电机ID
            speed_raw: 原始速度值 (-1000 ~ 1000)
            
        Returns:
            bool: 是否成功
        """
        if not self._connected:
            return False
            
        try:
            # ✅ 关键:速度模式也需要先使能电机
            if not self.is_motor_enabled(motor_id):
                self.enable_motor(motor_id)
                import time
                time.sleep(0.1)
                
            # ✅ 将原始速度值转换为 rad/s
            # RS-00 最大速度: 33 rad/s (约 1890 °/s)
            # 映射: -1000~1000 -> -33~33 rad/s
            velocity_rad_s = (speed_raw / 1000.0) * 33.0
                        
            # 限制范围
            velocity_rad_s = max(-33.0, min(33.0, velocity_rad_s))
                
            motor_key = f"motor_{motor_id}"
                
            # ✅ 速度模式:kp=0, 使用 velocity 参数
            self._bus.write_operation_frame(
                motor_key,
                position=0.0,  # 位置不重要
                velocity=velocity_rad_s,
                kp=0.0,  # 速度模式下 kp=0
                kd=4.0,
                torque=0.0
            )
                
            return True
        except Exception as e:
            print(f"设置速度失败: {e}")
            return False
    
    def ping(self, motor_id: int) -> bool:
        """Ping 电机（使用官方 SDK）"""
        if not self._connected:
            print(f"   ⚠️ Ping {motor_id}: 驱动未连接")
            return False
        try:
            # ✅ 临时注册电机到 bus.motors
            from .robstride_dynamics import Motor as RSDMotor
            motor_key = f"motor_{motor_id}"
            
            # 保存旧 motors，避免影响其他操作
            old_motors = dict(self._bus.motors)
            self._bus.motors.clear()
            self._bus.motors[motor_key] = RSDMotor(id=motor_id, model='rs-02')
            
            # 使用官方 SDK 的 ping_by_id
            result = self._bus.ping_by_id(motor_id, timeout=0.05)
            
            # 恢复旧 motors
            self._bus.motors.clear()
            self._bus.motors.update(old_motors)
            
            success = result is not None
            if success:
                print(f"   ✅ Ping {motor_id}: 成功")
            return success
        except Exception as e:
            print(f"   ❌ Ping {motor_id} 异常: {e}")
            return False
    
    def scan_motors(self, start_id: int = 1, end_id: int = 253) -> list:
        """扫描电机（直接 ping，不注册到 motors）"""
        found_ids = []
        
        # 扫描指定范围
        for motor_id in range(start_id, end_id + 1):
            if self.ping(motor_id):
                # ✅ 找到后获取反馈信息验证
                state = self.get_feedback(motor_id)
                if state:
                    found_ids.append(motor_id)
                    print(f"   🎯 找到电机 ID: {motor_id}")
        
        return found_ids
    
    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改电机 ID（兼容接口）"""
        if not self._connected:
            return False
        try:
            motor_key = f"motor_{old_id}"
            # ✅ 使用官方 SDK 的 write_id
            result = self._bus.write_id(motor_key, new_id)
            if result:
                # 更新内部记录
                if old_id in self._states:
                    self._states[new_id] = self._states.pop(old_id)
                if old_id in self._last_targets:
                    self._last_targets[new_id] = self._last_targets.pop(old_id)
                print(f"✅ 电机 ID 修改成功: {old_id} → {new_id}")
                return True
            else:
                print(f"⚠️ 修改 ID 超时，未收到响应")
            return False
        except Exception as e:
            print(f"修改电机 ID 失败: {e}")
            return False
    
    def is_motor_enabled(self, motor_id: int) -> bool:
        """检查电机是否已使能"""
        return self._states.get(motor_id, MotorState()).enabled
    
    def get_feedback(self, motor_id: int) -> Optional[MotorState]:
        """获取电机反馈"""
        if not self._connected:
            return None
        try:
            motor_key = f"motor_{motor_id}"
            
            # ✅ 先使能电机（如果需要）
            if not self.is_motor_enabled(motor_id):
                self.enable_motor(motor_id)
                import time
                time.sleep(0.05)
            
            # ✅ 发送空指令触发响应
            self._bus.write_operation_frame(
                motor_key,
                position=0.0,
                kp=0.0,
                kd=0.0,
                velocity=0.0,
                torque=0.0
            )
            
            # 读取响应
            pos, vel, tor, temp = self._bus.read_operation_frame(motor_key)
            
            state = MotorState(
                position=pos,
                velocity=vel,
                torque=tor,
                temperature=temp,
                enabled=True
            )
            self._states[motor_id] = state
            return state
        except Exception as e:
            return None
    
    def set_angle(self, motor_id: int, angle_deg: float, time_ms: int = 500) -> bool:
        """统一的角度控制接口（品牌特定逻辑由驱动自己处理）"""
        if not self._connected:
            return False
        
        try:
            # ✅ 先使能电机
            if not self.is_motor_enabled(motor_id):
                self.enable_motor(motor_id)
                import time
                time.sleep(0.1)
            
            # ✅ 获取当前位置
            current_state = None
            for retry in range(3):
                current_state = self.get_feedback(motor_id)
                if current_state:
                    break
                import time
                time.sleep(0.05)
            
            if not current_state:
                return False
            
            # ✅ 角度转弧度
            current_pos = current_state.position
            target_rad = angle_deg * 3.14159265 / 180.0
            
            # ✅ 将目标位置调整到当前位置附近（处理多圈问题）
            best_position = None
            best_distance = float('inf')
            
            for offset in [-2, -1, 0, 1, 2]:
                candidate = target_rad + offset * 2 * 3.14159265
                if -12.5 <= candidate <= 12.5:
                    distance = abs(candidate - current_pos)
                    if distance < best_distance:
                        best_distance = distance
                        best_position = candidate
            
            if best_position is None:
                return False
            
            # ✅ 发送控制指令
            return self.send_motion_control(
                motor_id=motor_id,
                position=best_position,
                kp=150.0,
                kd=3.0,
                torque=0.0
            )
        except Exception:
            return False
    
    def get_status(self, motor_id: int) -> Optional[Dict]:
        """统一的状态查询接口（返回标准化格式）"""
        state = self.get_feedback(motor_id)
        if not state:
            return None
        
        # ✅ 将弧度转换为角度，并归一化到 -180° ~ 180°
        angle_raw = state.position * 180.0 / 3.14159265
        angle_normalized = ((angle_raw + 180) % 360) - 180
        
        return {
            'servo_id': motor_id,
            'port': self.can_interface,
            'position': state.position,
            'angle': round(angle_normalized, 2),
            'voltage': 0,
            'temperature': state.temperature,
            'current': state.torque,
            'speed': state.velocity,
            'load': 0,
            'mode': 'position',
            'torque_enabled': state.enabled
        }
    
    def read_parameter(self, motor_id: int, param_type: ParameterType):
        """读取参数"""
        if not self._connected:
            return None
        try:
            motor_key = f"motor_{motor_id}"
            return self._bus.read(motor_key, param_type)
        except Exception as e:
            return None
    
    def write_parameter(self, motor_id: int, param_type: ParameterType, value):
        """写入参数"""
        if not self._connected:
            return False
        try:
            motor_key = f"motor_{motor_id}"
            self._bus.write(motor_key, param_type, value)
            return True
        except Exception as e:
            return False
    
    @property
    def is_connected(self) -> bool:
        return self._connected

# 以下是与原有 API 兼容的类
class RobStrideMotor:
    """
    RobStride 单电机控制类（使用官方 SDK）
    保持与原 API 兼容
    """
    
    def __init__(self, motor_id: int, can_interface: str = "can0", 
                 motor_model: str = "rs-00", host_can_id: int = 0xFF):
        self.motor_id = motor_id
        self.can_interface = can_interface
        self.motor_model = motor_model
        self.driver = RobStrideOfficialDriver(can_interface, host_can_id)
        self.driver.add_motor(motor_id, motor_model.replace('RS', 'rs-').lower())
        
        self._connected = False
        self._enabled = False
    
    def connect(self) -> bool:
        """连接 CAN 总线"""
        if self.driver.connect():
            self._connected = True
            return True
        else:
            return False
    
    def disconnect(self):
        """断开连接"""
        if self._connected:
            self.driver.disconnect()
            self._connected = False
    
    def enable_torque(self, enable: bool = True) -> bool:
        """使能/禁用扭矩"""
        if not self._connected:
            return False
        
        try:
            success = self.driver.enable_motor(self.motor_id) if enable else self.driver.disable_motor(self.motor_id)
            if success:
                self._enabled = enable
            return success
            
        except Exception as e:
            return False
    
    def set_position(self, position_rad: float, kp: float = 80.0, kd: float = 4.0,
                    feedforward_torque: float = 0.0) -> bool:
        """
        设置目标位置（PD 控制）
        """
        if not self._connected:
            return False
        
        try:
            success = self.driver.send_motion_control(
                motor_id=self.motor_id,
                position=position_rad,
                kp=kp,
                kd=kd,
                torque=feedforward_torque
            )
            return success
            
        except Exception as e:
            return False
    
    def set_velocity(self, velocity_rad_s: float, kp: float = 0.0, kd: float = 4.0,
                    feedforward_torque: float = 0.0) -> bool:
        """
        设置目标速度
        """
        if not self._connected:
            return False
        
        try:
            success = self.driver.send_motion_control(
                motor_id=self.motor_id,
                position=0.0,
                velocity=velocity_rad_s,
                kp=kp,
                kd=kd,
                torque=feedforward_torque
            )
            return success
            
        except Exception as e:
            return False
    
    def read_state(self) -> Optional[Dict]:
        """
        读取电机状态
        """
        if not self._connected:
            return None
        
        feedback = self.driver.get_feedback(self.motor_id)
        if feedback:
            return {
                'position': feedback.position,
                'velocity': feedback.velocity,
                'torque': feedback.torque,
                'temperature': feedback.temperature,
                'enabled': feedback.enabled
            }
        return None
    
    @property
    def is_connected(self) -> bool:
        return self._connected and self.driver.is_connected
