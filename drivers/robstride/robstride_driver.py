"""
灵足 Robstride 总线电机驱动
基于官方 el_a3_sdk 的薄封装层

实现核心功能：
1. 设置ID（通过底层CAN协议）
2. 设置模式（位置/速度/电流等）
3. 根据模式控制转速或角度
4. 返回电机状态数据

注意：此驱动依赖 Linux SocketCAN，需要在 Linux 系统上运行
"""

import sys
import logging
from typing import Dict, Optional, List
from enum import Enum

# 尝试导入官方SDK
try:
    from el_a3_sdk.can_driver import RobstrideCanDriver as OfficialDriver
    from el_a3_sdk.protocol import RunMode as OfficialRunMode
    from el_a3_sdk.protocol import ParamIndex
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    OfficialDriver = None
    OfficialRunMode = None
    ParamIndex = None

logger = logging.getLogger(__name__)


class RunMode(Enum):
    """电机运行模式（映射到官方SDK）"""
    MOTION_CONTROL = 0   # 运控模式（PD + 前馈力矩）
    POSITION_PP = 1      # 位置模式 (PP，梯形规划)
    VELOCITY = 2         # 速度模式
    CURRENT = 3          # 电流模式
    POSITION_CSP = 5     # 位置模式 (CSP，连续位置)


class RobstrideDriver:
    """
    Robstride 电机驱动封装
    
    技术规格：
    - 通信方式：CAN 2.0 扩展帧，29位 ID，1Mbps
    - 位置范围：取决于电机型号（RS00: ±2.79 rad, EL05/RS05: ±1.57 rad）
    - 速度范围：RS00: ±33 rad/s, EL05/RS05: ±50 rad/s
    - 力矩范围：RS00: ±14 Nm, EL05: ±6 Nm, RS05: ±5.5 Nm
    
    注意：需要 Linux SocketCAN 支持
    """
    
    def __init__(self, can_name: str = "can0", motor_ids: List[int] = None):
        """
        初始化驱动
        
        :param can_name: CAN接口名称 (如 can0)
        :param motor_ids: 电机ID列表
        """
        if not SDK_AVAILABLE:
            print("❌ el_a3_sdk 未安装，无法使用 Robstride 驱动")
            print("   请安装: cd /path/to/EDULITE_A3/el_a3_sdk && pip install -e .")
            raise ImportError("el_a3_sdk not available")
        
        self.can_name = can_name
        self.motor_ids = motor_ids or [1, 2, 3, 4, 5, 6, 7]
        self.driver = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """连接CAN接口并启动接收线程"""
        try:
            self.driver = OfficialDriver(can_name=self.can_name)
            
            if not self.driver.connect():
                print(f"❌ 无法连接 CAN 接口 {self.can_name}")
                return False
            
            self.driver.start_receive_thread()
            self.is_connected = True
            print(f"✅ Robstride 连接到 {self.can_name}")
            return True
            
        except Exception as e:
            print(f"❌ Robstride 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开CAN连接"""
        if self.driver:
            self.driver.stop_receive_thread()
            self.driver.disconnect()
            self.is_connected = False
            print("🔌 Robstride 已断开")
    
    # ==================== 功能1: 设置ID ====================
    
    def set_id(self, old_id: int, new_id: int) -> bool:
        """
        设置电机ID
        
        :param old_id: 当前ID
        :param new_id: 新ID (1-253)
        :return: 是否成功
        
        注意：此功能通过底层CAN协议实现，官方SDK未直接暴露
        """
        if not self.is_connected or not self.driver:
            print("驱动未连接")
            return False
        
        try:
            # 构建 SET_CAN_ID 命令 (CommType = 7)
            can_id = self.driver._build_extended_can_id(7, self.driver.host_can_id, old_id)
            data = bytes([new_id]) + bytes(7)
            
            success = self.driver._send_frame(can_id, data)
            
            if success:
                print(f"✅ 电机ID已从 {old_id} 设置为 {new_id}")
            else:
                print(f"❌ 设置ID失败")
            
            return success
            
        except Exception as e:
            print(f"❌ 设置ID异常: {e}")
            return False
    
    # ==================== 功能2: 设置模式 ====================
    
    def set_mode(self, motor_id: int, mode: RunMode) -> bool:
        """
        设置电机工作模式
        
        :param motor_id: 电机ID
        :param mode: 工作模式
        :return: 是否成功
        """
        if not self.is_connected or not self.driver:
            print("驱动未连接")
            return False
        
        try:
            # 映射到官方SDK的RunMode
            official_mode_map = {
                RunMode.MOTION_CONTROL: OfficialRunMode.MOTION_CONTROL,
                RunMode.POSITION_PP: OfficialRunMode.POSITION_PP,
                RunMode.VELOCITY: OfficialRunMode.VELOCITY,
                RunMode.CURRENT: OfficialRunMode.CURRENT,
                RunMode.POSITION_CSP: OfficialRunMode.POSITION_CSP,
            }
            
            official_mode = official_mode_map.get(mode)
            if official_mode is None:
                print(f"无效的模式: {mode}")
                return False
            
            success = self.driver.set_run_mode(motor_id, official_mode)
            
            if success:
                print(f"✅ 电机 {motor_id} 已设置为 {mode.name}")
            else:
                print(f"❌ 设置模式失败")
            
            return success
            
        except Exception as e:
            print(f"❌ 设置模式异常: {e}")
            return False
    
    # ==================== 功能3: 控制转速或角度 ====================
    
    def set_position(self, motor_id: int, position: float, mode: RunMode = RunMode.POSITION_PP) -> bool:
        """
        设置目标位置
        
        :param motor_id: 电机ID
        :param position: 目标位置 (弧度)
        :param mode: 位置模式 (POSITION_PP 或 POSITION_CSP)
        :return: 是否成功
        """
        if not self.is_connected or not self.driver:
            print("驱动未连接")
            return False
        
        try:
            # 先设置模式
            self.set_mode(motor_id, mode)
            
            # 根据模式选择控制方法
            if mode == RunMode.POSITION_CSP:
                success = self.driver.set_position_csp(motor_id, position)
            else:  # POSITION_PP
                success = self.driver.set_position_pp(motor_id, position)
            
            if success:
                print(f"电机 {motor_id} 移动到 {position:.3f} rad")
            else:
                print(f"❌ 设置位置失败")
            
            return success
            
        except Exception as e:
            print(f"❌ 设置位置异常: {e}")
            return False
    
    def set_speed(self, motor_id: int, speed: float) -> bool:
        """
        设置目标速度
        
        :param motor_id: 电机ID
        :param speed: 目标速度 (rad/s)
        :return: 是否成功
        """
        if not self.is_connected or not self.driver:
            print("驱动未连接")
            return False
        
        try:
            # 先设置为速度模式
            self.set_mode(motor_id, RunMode.VELOCITY)
            
            # 写入速度参数
            success = self.driver.write_parameter(motor_id, ParamIndex.SPD_REF, speed)
            
            if success:
                print(f"电机 {motor_id} 速度设置为 {speed:.3f} rad/s")
            else:
                print(f"❌ 设置速度失败")
            
            return success
            
        except Exception as e:
            print(f"❌ 设置速度异常: {e}")
            return False
    
    def move_to_position(self, motor_id: int, position: float, time_ms: int = 0) -> bool:
        """
        便捷方法：移动到指定位置（自动确保位置模式）
        
        :param motor_id: 电机ID
        :param position: 目标位置 (弧度)
        :param time_ms: 运动时间(毫秒, 暂未使用)
        :return: 是否成功
        """
        return self.set_position(motor_id, position, RunMode.POSITION_PP)
    
    def rotate_at_speed(self, motor_id: int, speed: float) -> bool:
        """
        便捷方法：以指定速度旋转（自动确保速度模式）
        
        :param motor_id: 电机ID
        :param speed: 速度 (rad/s)
        :return: 是否成功
        """
        return self.set_speed(motor_id, speed)
    
    # ==================== 功能4: 读取电机数据 ====================
    
    def get_observation(self, motor_id: int) -> Optional[Dict]:
        """
        读取电机状态
        
        :param motor_id: 电机ID
        :return: 状态字典 或 None
        """
        if not self.is_connected or not self.driver:
            print("驱动未连接")
            return None
        
        try:
            feedback = self.driver.get_feedback(motor_id)
            
            if feedback:
                observation = {
                    'motor_id': feedback.motor_id,
                    'position': feedback.position,      # rad
                    'velocity': feedback.velocity,      # rad/s
                    'torque': feedback.torque,          # Nm
                    'temperature': feedback.temperature, # °C
                }
                print(f"电机 {motor_id} 状态: pos={feedback.position:.3f}, "
                           f"vel={feedback.velocity:.3f}, tor={feedback.torque:.3f}")
                return observation
            else:
                print(f"未收到电机 {motor_id} 的反馈")
                return None
                
        except Exception as e:
            print(f"❌ 读取状态异常: {e}")
            return None
    
    def get_temperature(self, motor_id: int) -> Optional[float]:
        """读取温度"""
        obs = self.get_observation(motor_id)
        return obs['temperature'] if obs else None
    
    def get_position(self, motor_id: int) -> Optional[float]:
        """读取位置"""
        obs = self.get_observation(motor_id)
        return obs['position'] if obs else None
    
    def get_speed(self, motor_id: int) -> Optional[float]:
        """读取速度"""
        obs = self.get_observation(motor_id)
        return obs['velocity'] if obs else None
    
    def get_torque(self, motor_id: int) -> Optional[float]:
        """读取力矩"""
        obs = self.get_observation(motor_id)
        return obs['torque'] if obs else None
    
    # ==================== 辅助功能 ====================
    
    def enable_motor(self, motor_id: int) -> bool:
        """使能电机"""
        if not self.is_connected or not self.driver:
            return False
        return self.driver.enable_motor(motor_id)
    
    def disable_motor(self, motor_id: int, clear_fault: bool = False) -> bool:
        """失能电机"""
        if not self.is_connected or not self.driver:
            return False
        return self.driver.disable_motor(motor_id, clear_fault)
    
    def set_zero_position(self, motor_id: int) -> bool:
        """设置当前位置为零位"""
        if not self.is_connected or not self.driver:
            return False
        return self.driver.set_zero_position(motor_id)
    
    def ping(self, motor_id: int) -> bool:
        """测试电机是否在线（通过检查是否有反馈）"""
        if not self.is_connected or not self.driver:
            return False
        
        feedback = self.driver.get_feedback(motor_id)
        return feedback is not None
    
    def query_version(self, motor_id: int) -> Optional[str]:
        """查询固件版本"""
        if not self.is_connected or not self.driver:
            return None
        
        try:
            version = self.driver.query_firmware_version(motor_id)
            if version:
                return f"{version.major}.{version.minor}.{version.patch}"
            return None
        except Exception as e:
            print(f"查询版本失败: {e}")
            return None
