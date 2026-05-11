"""
RobStride 电机驱动 - 基于官方 robstride_dynamics SDK
提供与原有 API 兼容的接口，但使用官方 SDK 实现
"""

import time
import logging
from typing import Optional, Dict, List
from dataclasses import dataclass

# 导入官方 SDK（已集成到项目中）
from .robstride_dynamics import RobstrideBus, Motor as RSDMotor, ParameterType

logger = logging.getLogger(__name__)

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
            logger.info(f"✅ RobStride 官方驱动连接成功: {self.can_interface}")
            return True
        except Exception as e:
            logger.error(f"❌ RobStride 连接失败: {e}")
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
            logger.info("RobStride 驱动已断开")
    
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
            logger.error(f"使能电机 {motor_id} 失败: {e}")
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
            logger.error(f"禁用电机 {motor_id} 失败: {e}")
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
            logger.debug(f"电机 {motor_id} 控制成功: pos={pos:.3f}, target={position:.3f}")
            return True
        except Exception as e:
            logger.error(f"发送运动控制指令失败: {e}")
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
                logger.info(f"  🔌 使能 RobStride 电机 {motor_id} (速度模式)...")
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
                
            logger.info(f"✅ RobStride 电机 {motor_id} 速度设置为 {speed_raw} ({velocity_rad_s:.2f} rad/s)")
            return True
        except Exception as e:
            logger.error(f"设置速度失败: {e}")
            return False
    
    def ping(self, motor_id: int) -> bool:
        """Ping 电机（快速检测是否存在）"""
        if not self._connected:
            return False
        try:
            # ✅ 使用官方 SDK 的 ping_by_id，快速且不需要使能
            result = self._bus.ping_by_id(motor_id, timeout=0.1)
            return result is not None
        except Exception as e:
            logger.debug(f"Ping 电机 {motor_id} 失败: {e}")
            return False
    
    def is_motor_enabled(self, motor_id: int) -> bool:
        """检查电机是否已使能"""
        return self._states.get(motor_id, MotorState()).enabled
    
    def get_feedback(self, motor_id: int) -> Optional[MotorState]:
        """获取电机反馈(重新发送上次目标以触发响应)"""
        if not self._connected:
            return None
        try:
            motor_key = f"motor_{motor_id}"
            
            # ✅ 如果有记录的目标，重新发送以保持运动并触发响应
            if motor_id in self._last_targets:
                target = self._last_targets[motor_id]
                self._bus.write_operation_frame(
                    motor_key,
                    position=target['position'],
                    velocity=target.get('velocity', 0.0),
                    kp=target['kp'],
                    kd=target['kd'],
                    torque=target['torque']
                )
            else:
                # ✅ 首次调用，发送空指令触发响应
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
                enabled=self._states[motor_id].enabled
            )
            self._states[motor_id] = state
            return state
        except Exception as e:
            logger.warning(f"读取电机 {motor_id} 状态失败: {e}")
            return None
    
    def read_parameter(self, motor_id: int, param_type: ParameterType):
        """读取参数"""
        if not self._connected:
            return None
        try:
            motor_key = f"motor_{motor_id}"
            return self._bus.read(motor_key, param_type)
        except Exception as e:
            logger.error(f"读取参数失败: {e}")
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
            logger.error(f"写入参数失败: {e}")
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
        
        logger.info(f"RobStride 电机初始化: ID={motor_id}, 型号={motor_model}")
    
    def connect(self) -> bool:
        """连接 CAN 总线"""
        if self.driver.connect():
            self._connected = True
            logger.info(f"✅ RobStride 电机 {self.motor_id} 已连接")
            return True
        else:
            logger.error(f"❌ 连接失败")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self._connected:
            self.driver.disconnect()
            self._connected = False
            logger.info(f"RobStride 电机 {self.motor_id} 已断开")
    
    def enable_torque(self, enable: bool = True) -> bool:
        """使能/禁用扭矩"""
        if not self._connected:
            logger.warning("电机未连接")
            return False
        
        try:
            success = self.driver.enable_motor(self.motor_id) if enable else self.driver.disable_motor(self.motor_id)
            if success:
                self._enabled = enable
            logger.debug(f"电机 {self.motor_id} 扭矩{'使能' if enable else '禁用'}")
            return success
            
        except Exception as e:
            logger.error(f"使能扭矩失败: {e}")
            return False
    
    def set_position(self, position_rad: float, kp: float = 80.0, kd: float = 4.0,
                    feedforward_torque: float = 0.0) -> bool:
        """
        设置目标位置（PD 控制）
        """
        if not self._connected:
            logger.warning("电机未连接")
            return False
        
        try:
            success = self.driver.send_motion_control(
                motor_id=self.motor_id,
                position=position_rad,
                kp=kp,
                kd=kd,
                torque=feedforward_torque
            )
            logger.debug(f"电机 {self.motor_id} 位置指令: {position_rad:.3f} rad")
            return success
            
        except Exception as e:
            logger.error(f"设置位置失败: {e}")
            return False
    
    def set_velocity(self, velocity_rad_s: float, kp: float = 0.0, kd: float = 4.0,
                    feedforward_torque: float = 0.0) -> bool:
        """
        设置目标速度
        """
        if not self._connected:
            logger.warning("电机未连接")
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
            logger.debug(f"电机 {self.motor_id} 速度指令: {velocity_rad_s:.3f} rad/s")
            return success
            
        except Exception as e:
            logger.error(f"设置速度失败: {e}")
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
