"""
飞特 ST3215 舵机驱动 - 直接基于 scservo_sdk
"""
from typing import Optional, Dict
import numpy as np

from .scservo_sdk import PortHandler, sms_sts
from .scservo_sdk.scservo_def import COMM_SUCCESS

# ── 用到的寄存器地址 (与 sms_sts.py 一致) ──
ADDR_TORQUE_ENABLE = 40
ADDR_MODE = 33
ADDR_ID = 5
ADDR_LOCK = 55

# 速度模式常量
MODE_POSITION = 0
MODE_VELOCITY = 1


class ST3215Driver:
    """ST3215 舵机驱动"""

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 1000000,
                 servo_config: Optional[Dict] = None):
        self.port = port
        self.baudrate = baudrate
        self.is_connected = False
        self._ph: Optional[PortHandler] = None
        self._servo: Optional[sms_sts] = None
        self.id_to_offset = self._build_id_offset_map(servo_config)

    # ── 生命周期 ──

    def connect(self) -> bool:
        """打开串口并初始化通信。

        Returns:
            bool: 连接成功返回 True
        """
        try:
            self._ph = PortHandler(self.port)
            self._servo = sms_sts(self._ph)
            if not self._ph.openPort():
                return False
            if not self._ph.setBaudRate(self.baudrate):
                return False
            if hasattr(self._ph, 'ser') and self._ph.ser:
                self._ph.ser.timeout = 0.1
                self._ph.ser.write_timeout = 0.1
            self.is_connected = True
            print(f"✅ 串口连接成功: {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"❌ servo 连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """关闭串口连接。"""
        if self._ph:
            self._ph.closePort()
        self.is_connected = False
        self._servo = None
        self._ph = None

    def ping(self, servo_id: int) -> bool:
        """检测舵机是否在线。

        Args:
            servo_id: 舵机 ID

        Returns:
            bool: 在线返回 True
        """
        if not self.is_connected or self._servo is None:
            return False
        _, result, _ = self._servo.ping(servo_id)
        return result == COMM_SUCCESS

    # ── 位置 / 角度 ──

    def get_position(self, servo_id: int) -> Optional[int]:
        """读取舵机当前步进位置值 (0-4095)。

        Returns:
            int: 步进值，读取失败返回 None
        """
        if not self.is_connected or self._servo is None:
            return None
        pos, result, _ = self._servo.ReadPos(servo_id)
        return pos if result == COMM_SUCCESS else None

    # ST3215 速度范围 0-3400，0=EEPROM 默认值（出厂很慢），3400≈712 RPM
    _DEFAULT_SERVO_SPEED: int = 2000
    # 加速度范围 0-255，0=EEPROM 默认值（出厂极慢），值越大加速越快
    _DEFAULT_SERVO_ACC: int = 150

    def set_position(
        self, servo_id: int, position: int,
        speed: int = _DEFAULT_SERVO_SPEED, time_ms: int = 0,
    ) -> bool:
        """设置舵机目标步进位置 (0-4095, 对应 0°-360°)。

        Args:
            servo_id: 舵机 ID
            position: 步进值 (自动 clamp 到 0-4095)
            speed:    运动速度 (0-3400)，0 则使用 EEPROM 默认值
            time_ms:  保留参数（此 SDK 封包 time 固定为 0，不生效）
        """
        if not self.is_connected or self._servo is None:
            print(f"⚠️ [ST3215] 未连接，跳过发送 - ID={servo_id}, Position={position}")
            return False
        # 速度/加速度为 0 会回退到 EEPROM 出厂默认（通常极慢），保护防止误传
        actual_speed = speed if speed > 0 else self._DEFAULT_SERVO_SPEED
        position = max(0, min(4095, position))
        result, _ = self._servo.WritePosEx(servo_id, position, actual_speed, self._DEFAULT_SERVO_ACC)
        return result == COMM_SUCCESS

    # ── 角度便捷方法 ──

    def move_to_angle(
        self, servo_id: int, angle: float,
        speed: int = _DEFAULT_SERVO_SPEED, time_ms: int = 0,
    ) -> bool:
        """角度制便捷接口：将角度 (°) 转为步进值后发送。"""
        offset = self.id_to_offset.get(servo_id, 0.0)
        angle_with_offset = angle + offset
        normalized = angle_with_offset + 180
        position = int((normalized / 360.0) * 4095)
        position = max(0, min(4095, position))
        return self.set_position(servo_id, position, speed, time_ms)

    def set_angle(
        self, servo_id: int, angle_deg: float,
        speed: int = _DEFAULT_SERVO_SPEED, time_ms: int = 0,
    ) -> bool:
        """move_to_angle 的别名。"""
        return self.move_to_angle(servo_id, angle_deg, speed, time_ms)

    # ── 扭矩 ──

    def set_torque(self, servo_id: int, enable: bool) -> bool:
        """使能/失能舵机扭矩（上电/掉电）。

        Args:
            servo_id: 舵机 ID
            enable: True=使能, False=失能

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            return False
        result, _ = self._servo.write1ByteTxRx(servo_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)
        return result == COMM_SUCCESS

    # ── 模式切换 ──

    def set_velocity_mode(self, servo_id: int) -> bool:
        """切换舵机到速度模式（写入 MODE_VELOCITY 寄存器）。

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            return False
        try:
            result, _ = self._servo.write1ByteTxRx(servo_id, ADDR_MODE, MODE_VELOCITY)
            if result == COMM_SUCCESS:
                print(f"✅ 舵机 {servo_id} 已切换到速度模式")
            return result == COMM_SUCCESS
        except Exception as e:
            print(f"❌ 切换模式失败: {e}")
            return False

    def set_position_mode(self, servo_id: int) -> bool:
        """切换舵机到位置控制模式（写入 MODE_POSITION 寄存器）。

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            return False
        try:
            result, _ = self._servo.write1ByteTxRx(servo_id, ADDR_MODE, MODE_POSITION)
            if result == COMM_SUCCESS:
                print(f"✅ 舵机 {servo_id} 已切换到位置模式")
            return result == COMM_SUCCESS
        except Exception as e:
            print(f"❌ 切换模式失败: {e}")
            return False

    # ── 速度 (速度模式下) ──

    def set_speed(self, servo_id: int, speed: int) -> bool:
        """设置舵机目标速度（速度模式）。

        Args:
            servo_id: 舵机 ID
            speed: 速度值 (0~1023，对应 0%~100% 转速)

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            print(f"⚠️ [ST3215] 未连接,跳过发送 - ID={servo_id}, Speed={speed}")
            return False
        try:
            speed = int(speed)  # 防御: 确保是 int，防止 float 传入导致 SDK 位运算报错
            result, _ = self._servo.WriteSpec(servo_id, speed, 0)
            return result == COMM_SUCCESS
        except Exception as e:
            print(f"❌ 设置速度失败: {e}")
            return False

    def stop(self, servo_id: int) -> bool:
        """停止舵机（设速度为 0）。"""
        return self.set_speed(servo_id, 0)

    def sync_write_velocity(self, targets: dict) -> bool:
        """批量写入速度值（逐个发送）。

        Args:
            targets: {servo_id: speed}

        Returns:
            bool: 是否成功
        """
        if not self.is_connected:
            print(f"⚠️ [ST3215] 未连接，跳过同步速度发送")
            return False
        try:
            for servo_id, speed in targets.items():
                self.set_speed(servo_id, speed)
            return True
        except Exception as e:
            print(f"❌ 同步写入速度失败: {e}")
            return False

    def sync_write_spec_batch(self, targets: dict, acc: int = 0) -> bool:
        """批量写入速度（一次串口事务，不阻塞等待回复）。

        使用 SDK 的 groupSyncWrite + SyncWritePosEx(position=0) 实现。
        轮子模式下 position 无意义，等价于 WriteSpec 的批量版。
        将 N 次 request-response 串口往返压缩为 1 次广播发送。

        Args:
            targets: {servo_id: speed}
            acc: 加速度值，默认 0

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            print(f"⚠️ [ST3215] 未连接，跳过批量速度发送")
            return False
        try:
            self._servo.groupSyncWrite.clearParam()
            for servo_id, speed in targets.items():
                # 利用 SDK 已有的 SyncWritePosEx: position=0 时包体与 WriteSpec 一致
                self._servo.SyncWritePosEx(servo_id, 0, int(speed), acc)
            self._servo.groupSyncWrite.txPacket()
            return True
        except Exception as e:
            print(f"❌ 批量写入速度失败: {e}")
            return False

    sync_write_speeds = sync_write_velocity  # 别名

    def sync_write_positions(self, targets: Dict[int, float], time_ms: int = 50) -> bool:
        """批量写入目标角度（逐个发送，非真正的 SYNC_WRITE）。

        Args:
            targets: {servo_id: angle}
            time_ms: 运动时间（保留参数）

        Returns:
            bool: 是否成功
        """
        try:
            for servo_id, angle in targets.items():
                self.move_to_angle(servo_id, angle, time_ms=0)
            return True
        except Exception as e:
            print(f"❌ 同步写入位置失败: {e}")
            return False

    def write_position(self, servo_id: int, position: int) -> bool:
        """直接写入位置寄存器（无速度和加速度控制）。

        Args:
            servo_id: 舵机 ID
            position: 16 位步进值

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            print(f"⚠️ [ST3215] 未连接，跳过发送 - ID={servo_id}, Position={position}")
            return False
        try:
            result, _ = self._servo.write2ByteTxRx(
                servo_id, sms_sts.SMS_STS_GOAL_POSITION_L, position
            )
            return result == COMM_SUCCESS
        except Exception as e:
            print(f"❌ 写入位置失败: {e}")
            return False

    # ── ID 管理 ──

    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改舵机 ID（解锁 EEPROM → 写 ID → 锁定）。

        执行前校验: old_id 在线、new_id 不冲突。

        Args:
            old_id: 当前 ID
            new_id: 新 ID (1-253)

        Returns:
            bool: 是否成功
        """
        if not self.is_connected or self._servo is None:
            return False
        if new_id < 1 or new_id > 253 or old_id == new_id:
            return False
        if not self.ping(old_id):
            return False
        if self.ping(new_id):
            return False
        try:
            # 解锁 → 写 ID → 锁定 → 验证
            self._servo.unLockEprom(old_id)
            self.set_torque(old_id, False)
            result, _ = self._servo.write1ByteTxRx(old_id, ADDR_ID, new_id)
            self._servo.LockEprom(new_id)
            return result == COMM_SUCCESS
        except Exception:
            return False

    # ── 状态 ──

    def read_status(self, servo_id: int) -> Optional[Dict]:
        """读取舵机完整状态（位置、速度、负载、电压、温度、电流）。

        Returns:
            dict: {servo_id, port, position, angle, voltage, temperature, current, speed, load, mode, torque_enabled}
            读取失败返回 None
        """
        if not self.is_connected or self._servo is None:
            return None
        try:
            pos, res, _ = self._servo.ReadPos(servo_id)
            if res != COMM_SUCCESS:
                return None
            angle = (pos / 4095.0) * 360.0 - 180.0
            speed, _, _ = self._servo.ReadSpeed(servo_id)
            load, _, _ = self._servo.read2ByteTxRx(servo_id, 60)  # PRESENT_LOAD
            volt, _, _ = self._servo.read1ByteTxRx(servo_id, 62)  # PRESENT_VOLTAGE
            temp, _, _ = self._servo.read1ByteTxRx(servo_id, 63)  # PRESENT_TEMPERATURE
            cur, _, _ = self._servo.read2ByteTxRx(servo_id, 69)   # PRESENT_CURRENT
            return {
                'servo_id': servo_id, 'port': self.port,
                'position': pos, 'angle': round(angle, 2),
                'voltage': volt / 10.0 if volt else 0,
                'temperature': temp or 0,
                'current': cur or 0,
                'speed': speed or 0,
                'load': load or 0,
                'mode': 'position', 'torque_enabled': True,
            }
        except Exception:
            return None

    def get_status(self, servo_id: int) -> Optional[Dict]:
        """read_status 的别名。"""
        return self.read_status(servo_id)

    # ── 内部 ──

    def _build_id_offset_map(self, servo_config: Optional[Dict]) -> Dict[int, float]:
        """从配置中提取舵机 ID → 零位偏移的映射表。

        遍历配置树结构 (bus → part → joint)，提取每个关节的 id 和 zero_offset。

        Args:
            servo_config: 舵机配置字典

        Returns:
            Dict[int, float]: {servo_id: zero_offset}
        """
        id_to_offset = {}
        if not servo_config:
            return id_to_offset
        try:
            for bus_config in servo_config.values():
                if not isinstance(bus_config, dict):
                    continue
                for part_config in bus_config.values():
                    if not isinstance(part_config, dict):
                        continue
                    for joint_info in part_config.values():
                        if isinstance(joint_info, dict) and 'id' in joint_info:
                            id_to_offset[joint_info['id']] = joint_info.get('zero_offset', 0)
        except Exception as e:
            print(f"⚠️ 构建 ID-offset 映射失败: {e}")
        return id_to_offset
