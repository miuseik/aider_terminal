"""
RobStride 官方驱动适配器 — 实现 JointActuatorInterface 契约

将底层 RobstrideCanDriver 包装为与 Feetech ST3215Driver 统一接口，
使 ActuatorController 可以无差别控制舵机和电机。
"""

import time
import logging
from typing import Dict, List, Optional
from dataclasses import dataclass, field

from drivers.actuator.robStride.robstride_dynamics.can_driver import RobstrideCanDriver, _busy_wait_us
from drivers.actuator.robStride.robstride_dynamics.slcan_can_driver import SlcanCanDriver
from drivers.actuator.robStride.robstride_dynamics.protocol import (
    CommType, MotorType, RunMode, ParamIndex, MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP,
)
from drivers.actuator.robStride.robstride_dynamics.data_types import MotorFeedback
from drivers.actuator.robStride.robstride_dynamics.utils import rad_to_deg, deg_to_rad

logger = logging.getLogger("robstride.drive")


@dataclass
class RobStrideMotor:
    """RobStride 电机注册信息"""
    motor_id: int
    motor_type: MotorType = MotorType.RS00
    can_interface: str = "can0"
    enabled: bool = False
    online: bool = False

    @property
    def brand(self) -> str:
        return "robstride"

    @property
    def model(self) -> str:
        return self.motor_type.name

    def __repr__(self):
        return f"RobStrideMotor(id={self.motor_id}, type={self.motor_type.name}, online={self.online})"


class RobStrideOfficialDriver:
    """
    RobStride 电机官方驱动 — 适配 JointActuatorInterface

    将底层 RobstrideCanDriver 包装为与 Feetech ST3215Driver 统一的接口，
    使 ActuatorController 可以无差别控制舵机和电机。

    用法:
        driver = RobStrideOfficialDriver(can_interface="can0")
        driver.connect()
        ids = driver.scan_motors(1, 6)
        for mid in ids:
            driver.add_motor(mid)
        driver.set_torque(1, True)          # 使能
        driver.set_position(1, 0.5, 500)    # 设位置
        status = driver.get_status(1)       # 读状态
        driver.set_torque(1, False)         # 失能
    """

    def __init__(self, can_interface: str = "can0",
                 host_can_id: int = 0xFD,
                 motor_type_map: Optional[Dict[int, MotorType]] = None):
        self._can = RobstrideCanDriver(
            can_name=can_interface,
            host_can_id=host_can_id,
            motor_type_map=motor_type_map,
        )
        self._can_name = can_interface
        self._motors: Dict[int, RobStrideMotor] = {}   # motor_id → motor info
        self._kp: float = 10.0   # openArmX 默认值，平滑不抖动
        self._kd: float = 1.0
        self._recv_started: bool = False
        self._initialized: set = set()  # 已使能+切MIT位置模式的电机ID
        self._csp_initialized: set = set()  # 已使能+切CSP模式的电机ID
        self._vel_initialized: set = set()  # 已使能+切速度模式的电机ID
        self._csp_speed_limit: float = 1.0  # CSP 最大速度 (rad/s)，openArmX 默认 1 rad/s ≈ 57°/s

    # ── 生命周期 ──

    def connect(self) -> bool:
        """连接 CAN 接口并启动接收线程"""
        if not self._can.connect():
            return False
        self._can.start_receive_thread()
        self._recv_started = True
        logger.info("RobStride driver connected on %s", self._can_name)
        return True

    def disconnect(self) -> None:
        """断开连接"""
        self._can.disconnect()
        self._recv_started = False
        self._motors.clear()
        self._initialized.clear()
        self._csp_initialized.clear()
        self._vel_initialized.clear()

    @property
    def is_connected(self) -> bool:
        return self._can.is_connected

    # ── Ping ──

    def ping(self, device_id: int, timeout: float = 0.2) -> bool:
        """检测电机是否在线（读 VBUS 参数）"""
        result = self._can.read_parameter(device_id, ParamIndex.VBUS, timeout=timeout)
        return result is not None and result.success

    # ── 位置控制 ──

    def _ensure_ready(self, device_id: int) -> bool:
        """确保电机已使能并处于 MIT 运控模式"""
        if device_id in self._initialized:
            return True

        # 如果之前在速度模式或 CSP 模式，先失能再切换
        if device_id in self._vel_initialized:
            self._can.disable_motor(device_id)
            time.sleep(0.01)
            self._vel_initialized.discard(device_id)
        if device_id in self._csp_initialized:
            self._can.disable_motor(device_id)
            time.sleep(0.01)
            self._csp_initialized.discard(device_id)

        # 切换到 MIT 模式
        if not self._can.set_run_mode(device_id, RunMode.MOTION_CONTROL):
            logger.warning("Motor %d: failed to set MIT mode", device_id)
            return False
        time.sleep(0.02)

        # 使能电机
        if not self._can.enable_motor(device_id):
            logger.warning("Motor %d: failed to enable", device_id)
            return False
        time.sleep(0.01)

        self._initialized.add(device_id)
        if device_id in self._motors:
            self._motors[device_id].enabled = True
        logger.info("Motor %d initialized (MIT mode + enabled)", device_id)
        return True

    def _ensure_csp_ready(self, device_id: int) -> bool:
        """确保电机已使能并处于 CSP 连续位置模式（电机自己做速度规划，平滑运动）"""
        if device_id in self._csp_initialized:
            return True

        # 如果之前在其他模式，先失能再切换
        if device_id in self._vel_initialized:
            self._can.disable_motor(device_id)
            time.sleep(0.01)
            self._vel_initialized.discard(device_id)
        if device_id in self._initialized:
            self._can.disable_motor(device_id)
            time.sleep(0.01)
            self._initialized.discard(device_id)

        # 1) 切 CSP 模式 (run_mode = 5)
        if not self._can.set_run_mode(device_id, RunMode.POSITION_CSP):
            logger.warning("Motor %d: failed to set CSP mode", device_id)
            return False
        time.sleep(0.02)

        # 2) 使能
        if not self._can.enable_motor(device_id):
            logger.warning("Motor %d: failed to enable in CSP mode", device_id)
            return False
        time.sleep(0.01)

        # 3) 设置 CSP 速度上限 (limit_spd)
        if not self._can.set_velocity_limit(device_id, self._csp_speed_limit):
            logger.warning("Motor %d: failed to set CSP speed limit", device_id)
            return False
        time.sleep(0.01)

        self._csp_initialized.add(device_id)
        if device_id in self._motors:
            self._motors[device_id].enabled = True
        logger.info("Motor %d initialized (CSP mode, speed_limit=%.1f rad/s)", device_id, self._csp_speed_limit)
        return True

    def set_position(self, device_id: int, position: float, time_ms: int = 500,
                     velocity: float = 0.0, use_csp: bool = False) -> bool:
        """
        设置目标角度 (°)

        - MIT 模式 (use_csp=False, 默认)：PD 控制，每个 set_position 调用都发一帧
          运控指令。修复型号映射后 Kp/Kd 不会错位，不会猛冲。
        - CSP 模式 (use_csp=True, 实验)：电机内部做速度规划，平滑运动。
          注意：CSP 需要电机先 disable→切模式→enable，且有 CAN 超时看门狗
          可能自动失能。

        Args:
            device_id: 电机 ID
            position: 目标角度 (°)
            time_ms: 运动时间 (ms)，MIT 模式忽略（PD 闭环）
            velocity: 速度前馈 (rad/s)，仅 MIT 模式有效
            use_csp: True=CSP 平滑模式（实验）, False=MIT PD 模式（推荐）
        """
        pos_rad = deg_to_rad(position)

        if use_csp:
            if not self._ensure_csp_ready(device_id):
                return False
            return self._can.set_position_csp(device_id, pos_rad)
        else:
            if not self._ensure_ready(device_id):
                return False
            return self._can.send_motion_control(
                motor_id=device_id,
                position=pos_rad,
                velocity=velocity,
                kp=self._kp,
                kd=self._kd,
                torque=0.0,
            )

    def get_position(self, device_id: int) -> float:
        """读取当前位置 (°) — 返回角度制供前端显示"""
        fb = self._can.get_feedback(device_id)
        if fb and fb.is_valid:
            return rad_to_deg(fb.position)
        return 0.0

    # ── 速度控制 ──

    def _ensure_velocity_mode(self, device_id: int) -> bool:
        """
        确保电机已切换到速度模式并使能。

        官方流程:
        1. run_mode = 2 (VELOCITY)
        2. enable
        3. limit_cur (=27.0)  ← 最大电流，默认 0 会导致电机不转！
        4. acc_rad  (=20.0)   ← 加速度，默认 20 rad/s²
        """
        if device_id in self._vel_initialized:
            return True

        # 先停止当前 MIT 控制，再切速度模式
        self._can.disable_motor(device_id)
        time.sleep(0.01)

        if not self._can.set_run_mode(device_id, RunMode.VELOCITY):
            logger.warning("Motor %d: failed to set velocity mode", device_id)
            return False
        time.sleep(0.02)

        if not self._can.enable_motor(device_id):
            logger.warning("Motor %d: failed to enable in velocity mode", device_id)
            return False
        time.sleep(0.01)

        # 设置最大电流限制（必须！默认 0 → 电机不转）
        if not self._can.write_parameter(device_id, ParamIndex.LIMIT_CUR, 27.0):
            logger.warning("Motor %d: failed to set limit_cur", device_id)
            return False
        time.sleep(0.01)

        # 设置加速度
        if not self._can.write_parameter(device_id, ParamIndex.ACC_RAD, 20.0):
            logger.warning("Motor %d: failed to set acc_rad", device_id)
            return False

        # 退出 MIT 初始化集合，加入速度集合（两种模式互斥）
        self._initialized.discard(device_id)
        self._vel_initialized.add(device_id)
        logger.info("Motor %d switched to velocity mode (limit_cur=27A, acc=20rad/s²) + enabled", device_id)
        return True

    def set_velocity_mode(self, device_id: int) -> bool:
        """切换到速度模式（供 ActuatorController 通过 hasattr 调用）"""
        return self._ensure_velocity_mode(device_id)

    def set_position_mode(self, device_id: int) -> bool:
        """切换回 MIT 位置控制模式（失能→切 MOTION_CONTROL→使能）。

        与速度模式/CSP 模式互斥，会从 _vel_initialized / _csp_initialized 移除该电机。
        """
        self._can.disable_motor(device_id)
        time.sleep(0.01)

        if not self._can.set_run_mode(device_id, RunMode.MOTION_CONTROL):
            logger.warning("Motor %d: failed to set MIT mode", device_id)
            return False
        time.sleep(0.02)

        if not self._can.enable_motor(device_id):
            logger.warning("Motor %d: failed to enable in MIT mode", device_id)
            return False

        self._vel_initialized.discard(device_id)
        self._csp_initialized.discard(device_id)
        self._initialized.add(device_id)
        logger.info("Motor %d switched back to MIT position mode", device_id)
        return True

    def set_csp_speed(self, device_id: int, speed_rads: float) -> bool:
        """动态调整 CSP 模式最大速度 (rad/s)。

        Args:
            device_id: 电机 ID
            speed_rads: 最大速度 (rad/s)，RS06 电机建议 0.5~20 rad/s
        """
        self._csp_speed_limit = float(speed_rads)
        if device_id in self._csp_initialized:
            return self._can.set_velocity_limit(device_id, self._csp_speed_limit)
        return True

    def set_speed(self, device_id: int, velocity: float) -> bool:
        """
        设置速度 — 速度模式

        前端发送 Feetech 原始速度值（绝对值 0~1023 对应 0~100% 转速），
        按比例映射到 RobStride 电机最大速度 (rad/s)。
        """
        if not self._ensure_velocity_mode(device_id):
            return False

        # 获取电机最大速度
        motor_type = self._can.motor_type_map.get(device_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]
        max_speed = params.v_max  # rad/s

        # 前端 Feetech 原始值 → 按比例映射到 RobStride rad/s
        raw = max(-1023.0, min(1023.0, float(velocity)))
        speed_rads = (raw / 1023.0) * max_speed

        return self._can.write_parameter(device_id, ParamIndex.SPD_REF, speed_rads)

    # ── 力矩 / 使能 ──

    def set_torque(self, device_id: int, enabled_or_torque) -> bool:
        """
        使能/失能 或 设置力矩

        - True/1 → 使能电机
        - False/0 → 失能电机
        - float → 设置力矩 (Nm)，电流模式
        """
        if isinstance(enabled_or_torque, bool):
            if enabled_or_torque:
                ok = self._can.enable_motor(device_id)
                if ok and device_id in self._motors:
                    self._motors[device_id].enabled = True
                return ok
            else:
                ok = self._can.disable_motor(device_id)
                if ok and device_id in self._motors:
                    self._motors[device_id].enabled = False
                return ok
        else:
            # 力矩模式
            torque = float(enabled_or_torque)
            return self._can.write_parameter(device_id, ParamIndex.IQ_REF, torque)

    def enable(self, device_id: int) -> bool:
        return self.set_torque(device_id, True)

    def disable(self, device_id: int) -> bool:
        return self.set_torque(device_id, False)

    # ── 零位校准 ──

    def set_zero_position(self, device_id: int) -> bool:
        """设置当前位置为零位并保存到 Flash"""
        ok = self._can.set_zero_position(device_id)
        if ok:
            time.sleep(0.05)
            self._can.save_parameters(device_id)
        return ok

    # ── 状态读取 ──

    def get_status(self, device_id: int) -> dict:
        """返回 {position(°), velocity(°/s), torque(Nm), temperature(°C), voltage(V), ...}"""
        fb = self._can.get_feedback(device_id)
        voltage = None
        v_result = self._can.read_parameter(device_id, ParamIndex.VBUS, timeout=0.1)
        if v_result and v_result.success:
            voltage = v_result.value

        if not fb or not fb.is_valid:
            return {
                "position": 0.0, "velocity": 0.0, "torque": 0.0,
                "temperature": 0.0, "voltage": voltage or 0.0,
                "motor_id": device_id, "online": False,
            }

        return {
            "position": rad_to_deg(fb.position),
            "velocity": rad_to_deg(fb.velocity),
            "torque": fb.torque,
            "temperature": fb.temperature,
            "voltage": voltage or 0.0,
            "motor_id": device_id,
            "online": True,
        }

    # ── ID 管理 ──

    def set_id(self, old_id: int, new_id: int) -> bool:
        """修改电机 CAN ID (写入 Flash)

        支持未注册电机直接改 ID（通过 CAN 帧直发，不依赖本地注册状态）。
        """
        # 失能（纯 CAN 帧操作，无需注册）
        self._can.disable_motor(old_id)
        time.sleep(0.1)

        # 发送正确的 SET_CAN_ID 帧（新ID在bits 23-16，host在bits 15-8）
        ok = self._can.set_can_id(old_id, new_id)
        if not ok:
            return False
        time.sleep(0.2)

        # 保存到 EEPROM
        self._can.save_parameters(old_id)
        time.sleep(0.2)

        # 更新本地注册（如果已注册的话）
        motor = self._motors.pop(old_id, None)
        if motor:
            motor.motor_id = new_id
            self._motors[new_id] = motor
            # 更新底层类型映射
            self._can.motor_type_map[new_id] = self._can.motor_type_map.pop(old_id, MotorType.RS00)

        logger.info("Motor ID changed: %d → %d (saved to EEPROM)", old_id, new_id)
        return True

    # ── 扫描 ──

    def scan_motors(self, start_id: int = 1, end_id: int = 16) -> List[int]:
        """
        扫描 CAN 总线上的电机 ID

        通过逐个 ping (读 VBUS) 发现在线电机。
        范围越广 ping 超时越短，保证总耗时可控。
        """
        found: List[int] = []
        id_count = max(1, end_id - start_id + 1)
        # 动态超时: 范围 ≤16 用 200ms, 更大范围缩短到最低 15ms
        per_ping_timeout = max(0.015, min(0.2, 3.0 / id_count))
        for mid in range(start_id, end_id + 1):
            if self.ping(mid, timeout=per_ping_timeout):
                found.append(mid)
                logger.info("Found RobStride motor ID=%d on %s", mid, self._can_name)
        return found

    def add_motor(self, motor_id: int, motor_type: Optional[MotorType] = None) -> None:
        """注册电机到驱动管理"""
        if motor_type is None:
            motor_type = DEFAULT_MOTOR_TYPE_MAP.get(motor_id, MotorType.RS00)
        self._motors[motor_id] = RobStrideMotor(
            motor_id=motor_id,
            motor_type=motor_type,
            can_interface=self._can_name,
            online=True,
        )
        # 更新底层驱动的类型映射
        self._can.motor_type_map[motor_id] = motor_type
        logger.info("Registered motor %d (type=%s) on %s", motor_id, motor_type.name, self._can_name)

    # ── 批量同步 ──

    def sync_write_positions(self, targets: Dict[int, float], time_ms: int = 500) -> bool:
        """批量写入位置 (°) — 前端传角度制，转为弧度发给电机"""
        ok = 0
        for mid, pos in targets.items():
            pos_rad = deg_to_rad(pos)
            if self._can.send_motion_control(
                motor_id=mid, position=pos_rad, velocity=0.0,
                kp=self._kp, kd=self._kd, torque=0.0,
            ):
                ok += 1
            _busy_wait_us(150)  # CAN 帧间间隔，防止总线缓冲区溢出丢帧
        return ok > 0

    def sync_write_spec_batch(self, targets: Dict[int, int], acc: int = 0) -> bool:
        """批量写入速度（与 Feetech ST3215Driver 接口对齐）。

        Args:
            targets: {motor_id: speed_raw}，speed_raw 为 Feetech 原始速度值 (0~1023)
            acc: 加速度（RobStride 暂不使用）

        Returns:
            bool: 是否全部成功
        """
        ok = 0
        for motor_id, speed in targets.items():
            if self.set_speed(motor_id, speed):
                ok += 1
            _busy_wait_us(150)
        return ok > 0

    # ── 批量操作（对标 openArmX Robot API）──

    def enable_all(self) -> bool:
        """使能所有已注册电机"""
        ok = 0
        for mid in self._motors:
            if self._can.enable_motor(mid):
                self._motors[mid].enabled = True
                ok += 1
            _busy_wait_us(300)
        logger.info("enable_all: %d/%d motors", ok, len(self._motors))
        return ok > 0

    def disable_all(self) -> bool:
        """失能所有已注册电机（安全停机）"""
        ok = 0
        for mid in self._motors:
            if self._can.disable_motor(mid):
                self._motors[mid].enabled = False
                ok += 1
            _busy_wait_us(300)
        self._initialized.clear()
        self._csp_initialized.clear()
        self._vel_initialized.clear()
        logger.info("disable_all: %d/%d motors", ok, len(self._motors))
        return ok > 0

    def set_mode_all(self, mode: str = 'mit') -> bool:
        """设置所有电机运行模式 ('mit' | 'csp' | 'velocity')"""
        mode_map = {
            'mit': RunMode.MOTION_CONTROL,
            'csp': RunMode.POSITION_CSP,
            'velocity': RunMode.VELOCITY,
        }
        run_mode = mode_map.get(mode)
        if run_mode is None:
            logger.error("Unknown mode: %s (choose mit/csp/velocity)", mode)
            return False
        ok = 0
        for mid in self._motors:
            if self._can.set_run_mode(mid, run_mode):
                ok += 1
            _busy_wait_us(300)
        logger.info("set_mode_all('%s'): %d/%d motors", mode, ok, len(self._motors))
        return ok > 0

    def set_csp_limits_all(self, speed_limit: float = 1.0) -> bool:
        """设置所有电机 CSP 模式速度上限 (rad/s)"""
        self._csp_speed_limit = speed_limit
        ok = 0
        for mid in self._motors:
            if self._can.set_velocity_limit(mid, speed_limit):
                ok += 1
            _busy_wait_us(300)
        logger.info("set_csp_limits_all(%.1f rad/s): %d/%d", speed_limit, ok, len(self._motors))
        return ok > 0

    def move_all_to_zero(self, kp: float = None, kd: float = None) -> bool:
        """所有电机回到零位 (MIT PD 控制)"""
        _kp = kp if kp is not None else self._kp
        _kd = kd if kd is not None else self._kd
        ok = 0
        for mid in sorted(self._motors.keys()):
            # 如果之前在 CSP/速度模式，无条件切回 MIT
            if mid in self._csp_initialized or mid in self._vel_initialized:
                self._can.disable_motor(mid)
                time.sleep(0.01)
                self._csp_initialized.discard(mid)
                self._vel_initialized.discard(mid)
                self._can.set_run_mode(mid, RunMode.MOTION_CONTROL)
                time.sleep(0.01)
                self._can.enable_motor(mid)
                self._initialized.add(mid)
            if mid not in self._initialized:
                self._ensure_ready(mid)
            if self._can.send_motion_control(mid, 0.0, 0.0, _kp, _kd, 0.0):
                ok += 1
            _busy_wait_us(300)
        logger.info("move_all_to_zero(kp=%.1f, kd=%.1f): %d/%d", _kp, _kd, ok, len(self._motors))
        return ok > 0

    def move_one_joint_mit(self, motor_id: int, position: float,
                           kp: float = None, kd: float = None) -> bool:
        """单电机 MIT 运动 (position=弧度)"""
        _kp = kp if kp is not None else self._kp
        _kd = kd if kd is not None else self._kd
        if motor_id not in self._initialized:
            if not self._ensure_ready(motor_id):
                return False
        return self._can.send_motion_control(motor_id, position, 0.0, _kp, _kd, 0.0)

    def move_one_joint_csp(self, motor_id: int, position: float) -> bool:
        """单电机 CSP 运动 (position=弧度)"""
        if motor_id not in self._csp_initialized:
            if not self._ensure_csp_ready(motor_id):
                return False
        return self._can.set_position_csp(motor_id, position)

    def set_zero_all(self) -> bool:
        """设置所有电机当前位置为零位并写入 Flash（需先失能）"""
        ok = 0
        for mid in self._motors:
            if self._can.set_zero_position(mid):
                ok += 1
            time.sleep(0.05)
            self._can.save_parameters(mid)
            time.sleep(0.05)
        logger.info("set_zero_all: %d/%d motors", ok, len(self._motors))
        return ok > 0

    def show_all_status(self) -> None:
        """打印所有电机状态表格 (openArmX 风格)"""
        header = (
            f"{'ID':>4} {'Type':>6} {'On':>3} {'Mode':>8} "
            f"{'Pos(°)':>10} {'Vel(°/s)':>10} {'Torque(Nm)':>11} "
            f"{'Temp(°C)':>9} {'Volt(V)':>8} {'Fault':>6} {'FPS':>6}"
        )
        sep = "=" * len(header)
        print(sep)
        print("Motor Status")
        print(sep)
        print(header)
        print("-" * len(header))

        for mid in sorted(self._motors.keys()):
            motor = self._motors[mid]
            fb = self._can.get_feedback(mid)
            if fb and fb.is_valid:
                pos = rad_to_deg(fb.position)
                vel = rad_to_deg(fb.velocity)
                tor = fb.torque
                temp = fb.temperature
                mode = fb.mode_state
                fault = fb.fault_code
            else:
                pos = vel = tor = temp = 0.0
                mode = fault = -1

            v_result = self._can.read_parameter(mid, ParamIndex.VBUS, timeout=0.05)
            voltage = v_result.value if (v_result and v_result.success) else 0.0

            mode_str = {0: "RESET", 1: "CALIB", 2: "MOTOR"}.get(mode, f"?{mode}")
            online = "✓" if (fb and fb.is_valid) else "✗"
            fault_str = "OK" if fault == 0 else f"E{fault}"

            print(
                f"{mid:>4} {motor.motor_type.name:>6} {online:>3} {mode_str:>8} "
                f"{pos:>10.2f} {vel:>10.1f} {tor:>11.2f} {temp:>9.1f} "
                f"{voltage:>8.1f} {fault_str:>6} {self._can.get_can_fps():>6.0f}"
            )

        print(sep)
        total = len(self._motors)
        online_count = sum(
            1 for m in self._motors.values()
            if self._can.get_feedback(m.motor_id) and self._can.get_feedback(m.motor_id).is_valid
        )
        error_count = sum(
            1 for m in self._motors.values()
            if (fb := self._can.get_feedback(m.motor_id)) and fb.fault_code != 0
        )
        print(f"Total: {total} | Online: {online_count} | Errors: {error_count}")
        print(sep)

    # ── 上下文管理器 ──

    def __enter__(self):
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.disable_all()
        except Exception:
            pass
        self.disconnect()
        return False

    # ── PD 参数 ──

    def set_pd(self, kp: float, kd: float) -> None:
        """设置 MIT 运控模式的 PD 参数。

        Args:
            kp: 比例增益
            kd: 微分增益
        """
        self._kp = kp
        self._kd = kd

    # ── 模式切换 ──

    def set_mode(self, device_id: int, mode: RunMode) -> bool:
        """切换电机运行模式（MIT/位置/速度/电流）。

        Args:
            device_id: 电机 ID
            mode: 运行模式枚举值

        Returns:
            bool: 是否成功
        """
        return self._can.set_run_mode(device_id, mode)

    # ── 属性 ──

    @property
    def motors(self) -> Dict[int, RobStrideMotor]:
        """返回已注册的电机字典 {motor_id: RobStrideMotor}。"""
        return self._motors

    @property
    def can_fps(self) -> float:
        """返回 CAN 总线当前帧率 (FPS)。"""
        return self._can.get_can_fps()
