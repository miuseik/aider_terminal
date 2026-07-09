"""
RobStride 官方驱动适配器 — 实现 JointActuatorInterface 契约

将底层 RobstrideCanDriver 包装为与 Feetech ST3215Driver 统一接口，
使 ActuatorController 可以无差别控制舵机和电机。
"""

import time
import logging
from typing import Dict, List, Optional
from dataclasses import dataclass, field

from aiderminal.drivers.actuator.robStride.robstride_dynamics.can_driver import RobstrideCanDriver, _busy_wait_us
from aiderminal.drivers.actuator.robStride.robstride_dynamics.slcan_can_driver import SlcanCanDriver
from aiderminal.drivers.actuator.robStride.robstride_dynamics.protocol import (
    CommType, MotorType, RunMode, ParamIndex, MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP,
)
from aiderminal.drivers.actuator.robStride.robstride_dynamics.data_types import MotorFeedback
from aiderminal.drivers.actuator.robStride.robstride_dynamics.utils import rad_to_deg, deg_to_rad

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
                 motor_type_map: Optional[Dict[int, MotorType]] = None,
                 directions: Optional[Dict[int, float]] = None,
                 offsets_rad: Optional[Dict[int, float]] = None):
        self._can = RobstrideCanDriver(
            can_name=can_interface,
            host_can_id=host_can_id,
            motor_type_map=motor_type_map,
        )
        self._can_name = can_interface
        self._motors: Dict[int, RobStrideMotor] = {}   # motor_id → motor info
        self._kp: float = 30.0   # KP=30 确保足够力矩克服静摩擦（实测 KP=15 电机不转）
        self._kd: float = 2.0
        self._recv_started: bool = False
        self._initialized: set = set()  # 已使能+切MIT位置模式的电机ID
        self._csp_initialized: set = set()  # 已使能+切CSP模式的电机ID
        self._vel_initialized: set = set()  # 已使能+切速度模式的电机ID
        self._csp_speed_limit: float = 1.0  # CSP 最大速度 (rad/s)，openArmX 默认 1 rad/s ≈ 57°/s
        self._last_stale_warn: Dict[int, float] = {}  # motor_id → 上次 stale 日志时间戳（防刷屏）
        self._last_reenable: Dict[int, float] = {}    # motor_id → 上次强制重使能时间戳（限流，防总线风暴）

        # 关节方向/偏移校正：处理电机反向安装和机械零点不重合
        # direction = +1.0 表示正方向一致，-1.0 表示反向安装
        # offset 为弧度制偏移量，logical = direction * motor + offset
        self._directions: Dict[int, float] = dict(directions) if directions else {}
        self._offsets_rad: Dict[int, float] = dict(offsets_rad) if offsets_rad else {}

    # ── 生命周期 ──

    def _warn_stale(self, device_id: int, age: float, context: str = "get_position") -> None:
        """限流 stale 告警：同一电机最多每 10 秒打一次."""
        now = time.time()
        last = self._last_stale_warn.get(device_id, 0)
        if now - last < 10.0:
            return
        self._last_stale_warn[device_id] = now
        logger.warning(
            "[%s] motor %d feedback stale (age=%.1fs), fallback to %s",
            self._can_name, device_id, age, context,
        )

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

    def force_reinitialize(self) -> None:
        """强制清空所有电机的初始化标记，使下次 set_position/set_velocity 时自动重新使能。

        用例：电机因过流/看门狗等物理失能后，选择姿态时自动恢复，无需断电重启。
        """
        self._initialized.clear()
        self._csp_initialized.clear()
        self._vel_initialized.clear()
        logger.info("RobStride %s: all motor init flags cleared — will re‑engage on next command", self._can_name)

    @property
    def is_connected(self) -> bool:
        return self._can.is_connected

    # ── Ping ──

    def ping(self, device_id: int, timeout: float = 0.2) -> bool:
        """检测电机是否在线（读 VBUS 参数）"""
        result = self._can.read_parameter(device_id, ParamIndex.VBUS, timeout=timeout)
        return result is not None and result.success

    # ── 位置控制 ──

    # 总线空闲超过该秒数（或 can_state 非 ERROR-ACTIVE/UNKNOWN）→ 判定总线掉线，
    # 跳过重使能，避免往死总线狂发帧 / 误 disable 刚恢复的电机。
    _BUS_DOWN_IDLE = 2.0
    # 单电机 stale 后重使能的最小间隔（秒），避免每个控制周期都重发 → 总线风暴。
    _REENABLE_THROTTLE = 1.0

    def _bus_allows_enable(self) -> bool:
        """总线当前是否处于可以安全发送使能帧的状态。

        RobStride 控制帧只发不确认、总线无重传。若整条总线已掉线
        (can_state=STOPPED/BUS-OFF，或长时间无反馈帧)，此时对某电机重使能毫无意义，
        只会往死总线灌帧，还可能因为 _enable_sequence 里的 disable 把刚恢复的电机
        再次失能 → 表现为"抽风"。因此总线掉线时一律跳过重使能，交给
        actuator_controller 的 CAN 健康检查重连去恢复。

        判定：can_state 健康(ERROR-ACTIVE/UNKNOWN) 且 最近收到帧的空闲时间 < _BUS_DOWN_IDLE。
        从未收到帧(idle=-1)时退化为仅看 can_state —— 重连后首帧到达前的窗口允许尝试使能。
        """
        can = self._can
        if not getattr(can, 'is_bus_healthy', True):
            return False
        idle = can.idle_seconds
        if idle < 0:
            return True  # 从未收帧，靠上面 can_state 判断；重连后首帧前允许尝试
        return idle <= self._BUS_DOWN_IDLE

    def _ensure_ready(self, device_id: int) -> bool:
        """确保电机已使能并处于 MIT 运控模式，同时禁用 CAN 超时看门狗.

        ⚠️ 关键修复（14 号松/无力矩根因）：RobStride 的使能/切模式/失能帧都是
        **只发不确认**（_send_frame 返回的是 TX 是否成功，不等电机 ACK，且总线
        无重传）。一旦某帧被丢掉（can2 总线偶发卡顿 / 该关节线头接触不良），
        电机会停在失能态 → 松、无力矩，但上层仍照常发 MIT 位置帧（未使能电机
        忽略 MIT 帧），表现就是"命令在发、电机不动"。

        原实现把"帧发出去"误当"使能成功"，且一旦标记 _initialized 就再也不重发
        使能 → 丢帧的电机永久松着。本实现：
          1) 发完使能序列后**读回 RUN_MODE 确认**确实进入 MIT 模式（set_run_mode
             帧也可能丢），未确认则整段重试（每次重试都会重发 enable，提高命中率）；
          2) 运行中若某电机**反馈变 stale（大概率已掉使能：CAN 超时/故障/供电）**，
             用"轻量重使能"(不复先 disable，避免抖动) 恢复，并**仅在总线健康时**才尝试
             —— 总线整体掉线(Network is down)时所有电机都会 stale，此时若逐台重使能会
             制造总线风暴/电机反复失能("抽风")，应直接跳过，交给 actuator_controller 重连。
        """
        # 已初始化：反馈新鲜说明仍在线且处于 MIT 跟踪态，直接复用，不再重发使能
        if device_id in self._initialized:
            fb = self._can.get_feedback(device_id)
            now = time.time()
            if fb and fb.is_valid and (now - fb.timestamp) < 0.5:
                return True
            # ── 反馈陈旧 ──
            # 总线整体掉线：所有电机都会 stale，此时重使能无意义且有害 → 跳过。
            if not self._bus_allows_enable():
                return True
            # 总线健康、唯独本电机无反馈 → 大概率本电机掉使能，轻量重使能(不复先 disable)。
            last = self._last_reenable.get(device_id, 0)
            if now - last < self._REENABLE_THROTTLE:
                return True  # 节流期内维持当前指令，不重发使能帧
            self._last_reenable[device_id] = now
            logger.warning("[%s] motor %d feedback stale (bus healthy) → light re-enable",
                           self._can_name, device_id)
            if self._light_enable(device_id):
                return True
            # 轻量重使能失败 → 降级为完整序列(含 clear_fault)，仍失败则报松。
            return self._full_enable(device_id)

        # 尚未初始化 → 完整使能序列（含 disable+clear_fault）
        return self._full_enable(device_id)

    def _light_enable(self, device_id: int) -> bool:
        """运行中轻量重使能：仅重发 MIT 模式 + enable + CAN_TIMEOUT=0，**不先 disable**。

        用于运行中偶发掉使能的恢复。不复先 disable 是为了避免 disable→enable 抖动
        （这正是"抽风"的根因：每个控制周期 stale 就 disable 一个还在跟踪的电机）。
        仅做读回确认 + 重试；真正的故障恢复交给 _full_enable 的 clear_fault。
        """
        for attempt in range(3):
            if self._enable_sequence(device_id, clear_fault=False) and self._is_mit_mode(device_id):
                if device_id in self._motors:
                    self._motors[device_id].enabled = True
                logger.info("[%s] motor %d re-enabled (MIT mode, light)", self._can_name, device_id)
                return True
            time.sleep(0.05)
        return False

    def _full_enable(self, device_id: int) -> bool:
        """完整使能：disable+clear_fault → MIT 模式 → enable → CAN_TIMEOUT=0，带读回确认与重试。

        ⚠️ 注意：首次使能是"让电机上线"的动作，不能用 idle_seconds 判定总线健康
        （使能前电机本就不发反馈帧，idle 天然很高）——否则 engage 时反而使能不到电机。
        这里只判断 CAN 硬件接口是否真的 down(STOPPED/BUS-OFF)：若硬件掉线，发使能帧
        毫无意义且会灌死接口，直接跳过，交给 actuator_controller 重连恢复。
        """
        if not getattr(self._can, 'is_bus_healthy', True):
            return True  # 硬件掉线：本次先不使能，交重连恢复
        for attempt in range(3):
            if self._enable_sequence(device_id, clear_fault=True) and self._is_mit_mode(device_id):
                self._initialized.add(device_id)
                if device_id in self._motors:
                    self._motors[device_id].enabled = True
                logger.info("[%s] motor %d enabled (MIT mode + CAN_TIMEOUT=0)",
                            self._can_name, device_id)
                return True
            logger.warning("[%s] motor %d enable attempt %d/3 not confirmed",
                           self._can_name, device_id, attempt + 1)
            time.sleep(0.05)
        logger.error("[%s] motor %d ENABLE FAILED after 3 retries — will be LOOSE (no torque). "
                     "Check CAN wiring/connector/power for this motor.",
                     self._can_name, device_id)
        return False

    def _enable_sequence(self, device_id: int, clear_fault: bool = True) -> bool:
        """发送使能序列（MIT 模式 → enable → CAN_TIMEOUT=0），可选先 disable+clear_fault。

        clear_fault=True（首次/完整使能）：先 disable 并清除电机内部故障锁存，
            保证干净初始状态。仅 enable_motor 无法退出硬件故障态，必须带故障清除。
        clear_fault=False（运行中轻量重使能）：不再先 disable —— 避免运行中偶发
            stale 就对电机做 disable→enable 抖动（这正是"抽风"的根因之一）。

        返回该序列所有帧是否都成功 TX（注意：只是 TX 成功，不等 ACK；
        是否真正使能由 _is_mit_mode 读回确认）。
        """
        # 1.（可选）清除故障 + 失能（保证干净初始状态）
        if clear_fault:
            # clear_fault=True 发送 Type 4 帧 data[0]=1，清除电机内部故障锁存。
            # 这是 key：仅 enable_motor 无法退出硬件故障态，必须带故障清除的 disable。
            self._can.disable_motor(device_id, clear_fault=True)
            time.sleep(0.01)

        # 清理之前的模式跟踪
        self._vel_initialized.discard(device_id)
        self._csp_initialized.discard(device_id)

        # 2. 切换到 MIT 运控模式
        if not self._can.set_run_mode(device_id, RunMode.MOTION_CONTROL):
            return False
        time.sleep(0.02)

        # 3. 使能电机
        if not self._can.enable_motor(device_id):
            logger.warning("[%s] motor %d: enable frame failed to TX", self._can_name, device_id)
            return False
        time.sleep(0.01)

        # 4. 禁用 CAN 超时看门狗
        # 默认值可能很小（如 20~100ms），导致 set_position 发一帧后电机
        # 在超时窗口内停止追踪，永远到不了目标角度
        if not self._can.write_parameter_int(device_id, ParamIndex.CAN_TIMEOUT, 0):
            logger.warning("[%s] motor %d: CAN_TIMEOUT=0 frame failed to TX",
                           self._can_name, device_id)
        return True

    def _is_mit_mode(self, device_id: int) -> bool:
        """读回 RUN_MODE 确认电机已进入 MIT 运控模式（控制帧只发不确认，用读确认）。

        RunMode.MOTION_CONTROL == 0。读回失败/非 MIT 说明 set_run_mode 帧被丢，
        调用方应重试整段使能序列。
        """
        res = self._can.read_parameter(device_id, ParamIndex.RUN_MODE, timeout=0.3)
        if res and res.success:
            return int(round(res.value)) == int(RunMode.MOTION_CONTROL)
        return False

    def _ensure_csp_ready(self, device_id: int) -> bool:
        """确保电机已使能并处于 CSP 连续位置模式（电机自己做速度规划，平滑运动）"""
        if device_id in self._csp_initialized:
            return True

        # ── 强制失能 + 清除故障 ──
        self._can.disable_motor(device_id, clear_fault=True)
        time.sleep(0.01)

        # 清理之前的模式跟踪
        self._vel_initialized.discard(device_id)
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
        # 逻辑角度 → 电机角度 → 弧度
        motor_deg = self._logical_deg_to_motor_deg(device_id, position)
        pos_rad = deg_to_rad(motor_deg)

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

    # ── 方向/偏移变换 ──

    def _motor_deg_to_logical_deg(self, device_id: int, motor_deg: float) -> float:
        """电机原始角度 → 逻辑角度：logical = direction * motor + offset_deg"""
        d = self._directions.get(device_id, 1.0)
        off_deg = rad_to_deg(self._offsets_rad.get(device_id, 0.0))
        return round(d * motor_deg + off_deg, 2)

    def _logical_deg_to_motor_deg(self, device_id: int, logical_deg: float) -> float:
        """逻辑角度 → 电机角度：motor = direction * (logical - offset_deg)"""
        d = self._directions.get(device_id, 1.0)
        off_deg = rad_to_deg(self._offsets_rad.get(device_id, 0.0))
        return round(d * (logical_deg - off_deg), 2)

    def _motor_rad_to_logical_rad(self, device_id: int, motor_rad: float) -> float:
        """电机原始弧度 → 逻辑弧度"""
        d = self._directions.get(device_id, 1.0)
        off = self._offsets_rad.get(device_id, 0.0)
        return d * motor_rad + off

    def _logical_rad_to_motor_rad(self, device_id: int, logical_rad: float) -> float:
        """逻辑弧度 → 电机弧度"""
        d = self._directions.get(device_id, 1.0)
        off = self._offsets_rad.get(device_id, 0.0)
        return d * (logical_rad - off)

    # ── 位置读写 ──

    def get_position(self, device_id: int) -> Optional[float]:
        """读取当前逻辑角度 (°) — 自动应用 direction/offset 变换；读取失败返回 None。

        注意: 与 Feetech 驱动的 get_position (返回步进值 0~4095) 语义不同，
        本方法直接返回角度制。换算由调用方按 brand 区分处理。

        返回的是多圈位置（反馈帧 position），不是单圈机械角 MECH_POS。
        反馈帧里的 position 是电机 MIT 控制器追踪的多圈真值，
        MECH_POS (0x7019) 只是单圈机械角（0~360° 循环），不适用于多圈场景。
        
        读取后自动解绕 + direction/offset 变换。
        """
        fb = self._can.get_feedback(device_id)
        now = time.time()

        raw_deg = None
        # 反馈帧存在且新鲜（1 秒内）→ 直接取多圈位置（电机原始坐标系）
        if fb and fb.is_valid and (now - fb.timestamp) < 1.0:
            raw_deg = rad_to_deg(fb.position)
        else:
            # 反馈帧不存在或过期 → 主动读硬件 MECH_POS（耗时，最多 3 次重试）
            stale_age = now - fb.timestamp if fb else float('inf')
            self._warn_stale(device_id, stale_age, context="MECH_POS read")
            for attempt in range(3):
                result = self._can.read_parameter(device_id, ParamIndex.MECH_POS, timeout=0.3)
                if result and result.success:
                    raw_deg = rad_to_deg(result.value)
                    break
                logger.warning("[%s] motor %d MECH_POS read attempt %d/3 failed", self._can_name, device_id, attempt + 1)
            if raw_deg is None:
                logger.error("[%s] motor %d MECH_POS FAILED after 3 retries", self._can_name, device_id)
                return None

        # 电机坐标系 → 逻辑坐标系
        logical_deg = self._motor_deg_to_logical_deg(device_id, raw_deg)
        return logical_deg

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

        # 强制失能 + 清除故障，再切速度模式
        self._can.disable_motor(device_id, clear_fault=True)
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
        self._can.disable_motor(device_id, clear_fault=True)
        time.sleep(0.01)

        if not self._can.set_run_mode(device_id, RunMode.MOTION_CONTROL):
            return False
        time.sleep(0.02)

        if not self._can.enable_motor(device_id):
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
        """设置当前位置为零位并保存到 Flash。

        标零前会自动失能电机，标零后重新使能。
        断电后重新上电位置即正确，无需软件解绕。
        """
        # 1. 从所有模式集合中移除（标零前必须失能）
        was_mit = device_id in self._initialized
        was_csp = device_id in self._csp_initialized
        was_vel = device_id in self._vel_initialized
        self._initialized.discard(device_id)
        self._csp_initialized.discard(device_id)
        self._vel_initialized.discard(device_id)

        # 2. 失能电机（标零必须在失能状态下执行）
        self._can.disable_motor(device_id)
        time.sleep(0.1)

        # 3. 设零位 + 保存到 Flash
        ok = self._can.set_zero_position(device_id)
        if ok:
            time.sleep(0.05)
            self._can.save_parameters(device_id)
            time.sleep(0.05)
            logger.info("Motor %d: zero position set and saved to Flash", device_id)
        else:
            logger.error("Motor %d: failed to set zero position", device_id)
            return False

        # 4. 如果原来处于使能状态，重新使能
        if was_mit:
            self._ensure_ready(device_id)
        elif was_csp:
            self._ensure_csp_ready(device_id)
        elif was_vel:
            self._ensure_velocity_mode(device_id)

        return ok

    # ── 状态读取 ──

    def get_status(self, device_id: int) -> dict:
        """返回 {position(°), velocity(°/s), torque(Nm), temperature(°C), voltage(V), ...}

        位置优先取反馈帧的多圈 position（电机 MIT 控制器追踪的真值）。
        只在反馈帧过期时才退去主动读 MECH_POS 单圈机械角（慢且有循环问题）。
        读取后自动解绕 + direction/offset 变换为逻辑坐标系。
        """
        fb = self._can.get_feedback(device_id)
        now = time.time()
        fb_fresh = fb and fb.is_valid and (now - fb.timestamp) < 1.0

        raw_deg = None
        if fb_fresh:
            # 反馈帧新鲜 → 直接用多圈位置（电机原始坐标系）
            raw_deg = rad_to_deg(fb.position)
        else:
            # 反馈帧过期或不存在 → 退去 MECH_POS（单圈机械角，带重试）
            stale_age = now - fb.timestamp if (fb and fb.timestamp) else float('inf')
            self._warn_stale(device_id, stale_age, context="MECH_POS")
            for attempt in range(3):
                pos_result = self._can.read_parameter(device_id, ParamIndex.MECH_POS, timeout=0.3)
                if pos_result and pos_result.success:
                    raw_deg = rad_to_deg(pos_result.value)
                    break
                logger.warning("[%s] motor %d MECH_POS read attempt %d/3 failed",
                               self._can_name, device_id, attempt + 1)
            if raw_deg is None:
                logger.error("[%s] motor %d MECH_POS FAILED after 3 retries — returning 0",
                             self._can_name, device_id)

        # 电机坐标系 → 逻辑坐标系
        logical_deg = self._motor_deg_to_logical_deg(device_id, raw_deg if raw_deg is not None else 0.0)

        # 电压：只要电机有反馈就主动读（反馈帧不带电压）
        voltage = None
        if fb and fb.is_valid:
            v_result = self._can.read_parameter(device_id, ParamIndex.VBUS, timeout=0.2)
            if v_result and v_result.success:
                voltage = v_result.value

        # 速度也需应用方向变换（direction=-1 时翻转符号）
        motor_vel_dps = rad_to_deg(fb.velocity) if (fb and fb.is_valid) else 0.0
        d = self._directions.get(device_id, 1.0)
        logical_vel_dps = round(motor_vel_dps * d, 2)

        if not fb or not fb.is_valid:
            # MECH_POS 读取成功（raw_deg 非 None）说明 CAN 通信双向正常，
            # 只是电机不在主动上报反馈帧 → 应视为"在线"
            online = raw_deg is not None
            return {
                "position": logical_deg or 0.0, "angle": logical_deg or 0.0,
                "velocity": 0.0, "torque": 0.0,
                "temperature": 0.0, "voltage": round(voltage, 2) if voltage else 0.0,
                "motor_id": device_id, "online": online,
            }

        return {
            "position": logical_deg or 0.0,
            "angle": logical_deg or 0.0,
            "velocity": logical_vel_dps,
            "torque": round(fb.torque, 2),
            "temperature": round(fb.temperature, 2),
            "voltage": round(voltage, 2) if voltage else 0.0,
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
        """批量写入逻辑角度 (°) — 自动应用 direction/offset 变换后发往电机。
        
        首次调用会自动初始化所有电机（MIT模式+使能），后续调用复用已初始化状态。
        """
        ok = 0
        fail_reasons = []
        for mid, logical_deg in targets.items():
            ready = self._ensure_ready(mid)
            if not ready:
                fail_reasons.append(f"id={mid}")
                continue
            motor_deg = self._logical_deg_to_motor_deg(mid, logical_deg)
            pos_rad = deg_to_rad(motor_deg)
            sent = self._can.send_motion_control(
                motor_id=mid, position=pos_rad, velocity=0.0,
                kp=self._kp, kd=self._kd, torque=0.0,
            )
            if sent:
                ok += 1
            else:
                fail_reasons.append(f"id={mid}")
            _busy_wait_us(150)  # CAN 帧间间隔，防止总线缓冲区溢出丢帧

        if fail_reasons and ok > 0:  # 全失败=can 没接，无需日志
            logger.warning("[Motor] sync_write: ok=%d/%d fail=%s",
                          ok, len(targets), ",".join(fail_reasons))
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
        """所有电机回到逻辑零位 (MIT PD 控制) — 自动应用 direction/offset"""
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
            # 逻辑 0° → 电机角度
            motor_zero_rad = self._logical_rad_to_motor_rad(mid, 0.0)
            if self._can.send_motion_control(mid, motor_zero_rad, 0.0, _kp, _kd, 0.0):
                ok += 1
            _busy_wait_us(300)
        logger.info("move_all_to_zero(kp=%.1f, kd=%.1f): %d/%d", _kp, _kd, ok, len(self._motors))
        return ok > 0

    def move_one_joint_mit(self, motor_id: int, position: float,
                           kp: float = None, kd: float = None) -> bool:
        """单电机 MIT 运动 (position=逻辑弧度) — 自动应用 direction/offset"""
        _kp = kp if kp is not None else self._kp
        _kd = kd if kd is not None else self._kd
        if motor_id not in self._initialized:
            if not self._ensure_ready(motor_id):
                return False
        motor_rad = self._logical_rad_to_motor_rad(motor_id, position)
        return self._can.send_motion_control(motor_id, motor_rad, 0.0, _kp, _kd, 0.0)

    def move_one_joint_csp(self, motor_id: int, position: float) -> bool:
        """单电机 CSP 运动 (position=逻辑弧度) — 自动应用 direction/offset"""
        if motor_id not in self._csp_initialized:
            if not self._ensure_csp_ready(motor_id):
                return False
        motor_rad = self._logical_rad_to_motor_rad(motor_id, position)
        return self._can.set_position_csp(motor_id, motor_rad)

    def set_zero_all(self) -> bool:
        """设置所有电机当前位置为零位并写入 Flash。

        标零前自动失能所有电机，标零后不自动重新使能。
        断电后重新上电位置即正确，无需软件解绕。
        """
        # 1. 记录当前状态并失能所有电机
        was_enabled: Dict[int, str] = {}  # motor_id → mode ('mit'|'csp'|'vel')
        for mid in self._motors:
            if mid in self._vel_initialized:
                was_enabled[mid] = 'vel'
            elif mid in self._csp_initialized:
                was_enabled[mid] = 'csp'
            elif mid in self._initialized:
                was_enabled[mid] = 'mit'
        self._initialized.clear()
        self._csp_initialized.clear()
        self._vel_initialized.clear()

        # 2. 失能所有电机
        for mid in self._motors:
            self._can.disable_motor(mid)
            time.sleep(0.02)

        time.sleep(0.3)  # 等待全部失能确认

        # 3. 设零位 + 保存
        ok = 0
        for mid in sorted(self._motors.keys()):
            if self._can.set_zero_position(mid):
                ok += 1
            time.sleep(0.05)
            self._can.save_parameters(mid)
            time.sleep(0.05)

        logger.info("set_zero_all: %d/%d motors zeroed and saved to Flash", ok, len(self._motors))

        # 4. 重新使能之前使能的电机
        for mid, mode in was_enabled.items():
            if mode == 'mit':
                self._ensure_ready(mid)
            elif mode == 'csp':
                self._ensure_csp_ready(mid)
            elif mode == 'vel':
                self._ensure_velocity_mode(mid)
            time.sleep(0.03)

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
                motor_pos = rad_to_deg(fb.position)
                motor_vel = rad_to_deg(fb.velocity)
                pos = self._motor_deg_to_logical_deg(mid, motor_pos)
                d = self._directions.get(mid, 1.0)
                vel = motor_vel * d
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
