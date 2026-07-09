"""
执行器总管理器 — 品牌无关, 只依赖 JointActuatorInterface 契约。
所有驱动方法为同步调用 (串口/CAN 操作在主线程阻塞执行)。
支持 async 方法和同步包装（兼容同步调用方）。
"""
import asyncio
import time
import logging
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, asdict
from concurrent.futures import ThreadPoolExecutor

logger = logging.getLogger(__name__)


@dataclass
class ActuatorInfo:
    """执行器注册信息."""
    port: str
    device_id: int
    kind: str                      # "joint"
    brand: str                     # "feetech" | "robstride"
    model: str = ""
    joint_name: str = ""
    is_online: bool = True


class ActuatorController:
    """执行器总管理器.

    Feetech 舵机和 RobStride 关节电机共用本控制器.
    驱动方法为同步调用（串口/CAN I/O 在 asyncio 线程内直接执行）。
    """

    def __init__(self, motor_type_overrides: Optional[Dict[int, int]] = None,
                 direction_map: Optional[Dict[int, float]] = None,
                 offset_map: Optional[Dict[int, float]] = None) -> None:
        self._joint_drivers: Dict[str, object] = {}   # port → driver
        self._registry: Dict[Tuple[str, int], ActuatorInfo] = {}
        self._joint_map: Dict[str, Tuple[str, int]] = {}
        self._last_targets: Dict[Tuple[str, int], Tuple[float, int]] = {}
        self._running = False
        self._motor_type_overrides: Dict[int, int] = motor_type_overrides or {}
        self._direction_map: Dict[int, float] = direction_map or {}
        self._offset_map: Dict[int, float] = offset_map or {}
        self._pipeline = None  # MotionPipeline (扫描后自动同步)
        # 共享线程池：避免每次同步包装都创建/销毁 ThreadPoolExecutor
        self._executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="actuator")
        # CAN 重连冷却：防止硬件故障时无限重连
        self._last_reconnect_attempt: Dict[str, float] = {}     # port → timestamp
        self._reconnect_backoff: Dict[str, float] = {}           # port → cooldown_seconds
        self._reconnect_skip_count: Dict[str, int] = {}          # port → 已跳过次数

    def set_pipeline(self, pipeline) -> None:
        """注入运动管线：扫描完成后自动同步关节映射."""
        self._pipeline = pipeline

    def _can_reconnect_cooldown(self, port: str) -> float:
        """检查 CAN 重连冷却期，返回还需等待秒数（0=允许重连）."""
        now = time.time()
        last = self._last_reconnect_attempt.get(port, 0)
        cooldown = self._reconnect_backoff.get(port, 10.0)  # 默认 10s
        elapsed = now - last
        if elapsed < cooldown:
            return cooldown - elapsed
        return 0.0

    def _record_reconnect(self, port: str) -> None:
        """记录一次重连尝试，指数退避."""
        now = time.time()
        self._last_reconnect_attempt[port] = now
        prev = self._reconnect_backoff.get(port, 10.0)
        self._reconnect_backoff[port] = min(prev * 2, 120.0)  # 最大 120s

    def _reset_reconnect_backoff(self, port: str) -> None:
        """反馈恢复时重置退避."""
        if port in self._last_reconnect_attempt:
            del self._last_reconnect_attempt[port]
        if port in self._reconnect_backoff:
            del self._reconnect_backoff[port]
        if port in self._reconnect_skip_count:
            del self._reconnect_skip_count[port]

    def _get_or_create_driver(self, port: str):
        """获取或按需创建驱动实例（连接池模式，自动重连，CAN 热插拔容错）."""
        # 已缓存且连接正常 → 直接返回
        if port in self._joint_drivers:
            driver = self._joint_drivers[port]
            if hasattr(driver, "is_connected") and not driver.is_connected:
                if self._can_retry_setup(port):
                    if driver.connect():
                        logger.info("Reconnected driver for %s", port)
                        return driver
                logger.warning("Failed to reconnect driver for %s", port)
                return None
            # CAN 健康检查：电机断电再上电后，is_connected 仍为 True（socket 还在），
            # 但 _last_frame_time 会停滞 → idle>5 触发重连。更隐蔽的是接口本身在 OS 层
            # DOWN（'Network is down'/Errno 100）：此时 idle_seconds=-1，idle>5 永不成立，
            # 重连永不触发、电机永久掉线。故同时检测 is_interface_up，接口下线也立即重连。
            if "can" in port.lower() and hasattr(driver, "_can"):
                can = driver._can
                idle = can.idle_seconds
                bus_stuck = can.check_bus_health() in ("BUS-OFF", "STOPPED")
                if idle > 5.0 or not can.is_interface_up or bus_stuck:
                    cooldown_left = self._can_reconnect_cooldown(port)
                    if idle > 5.0:
                        reason = f"已 {idle:.1f}s 无反馈帧"
                    elif not can.is_interface_up:
                        reason = "接口已 DOWN (Network is down)"
                    else:
                        reason = f"总线异常 ({can.check_bus_health()})"
                    if cooldown_left > 0:
                        # 冷却期内，跳过重连（避免硬件故障时无限重连 spam）
                        cnt = self._reconnect_skip_count.get(port, 0) + 1
                        self._reconnect_skip_count[port] = cnt
                        # 每 30 次跳过才打一次 WARNING，避免刷屏
                        if cnt % 30 == 1:
                            logger.warning(
                                "CAN %s: %s（已跳过 %d 次重连，冷却剩余 %.0fs）",
                                port, reason, cnt, cooldown_left,
                            )
                    else:
                        logger.warning(
                            "CAN %s: %s，触发重连...",
                            port, reason,
                        )
                        try:
                            driver.disconnect()
                        except Exception:
                            pass
                        del self._joint_drivers[port]
                        self._record_reconnect(port)
                        return self._get_or_create_driver(port)
                else:
                    # 反馈帧恢复 → 重置退避计数器
                    self._reset_reconnect_backoff(port)
            return driver

        # 拒绝无效端口名（如前端下拉的 "all"）
        if port.lower() == "all":
            logger.warning("Invalid port 'all' — refusing to create driver")
            return None

        # 按端口类型创建驱动
        if "can" in port.lower():
            # CAN 热插拔：先尝试 setup_can，再连接
            self._can_retry_setup(port)
            from aiderminal.drivers.actuator.robStride import RobStrideOfficialDriver
            driver = RobStrideOfficialDriver(
                can_interface=port,
                directions=self._direction_map,
                offsets_rad=self._offset_map,
            )
        else:
            from aiderminal.drivers.actuator.feetech.feetech_driver import ST3215Driver
            driver = ST3215Driver(port)

        if driver.connect():
            self._joint_drivers[port] = driver
            logger.info("Created and cached driver for %s", port)
            return driver

        return None

    @staticmethod
    def _can_retry_setup(port: str) -> bool:
        """CAN 热插拔恢复：尝试重新初始化 can0 接口（最多一次）."""
        try:
            from aiderminal.utils.can_setup import setup_can
            ok = setup_can(port)
            if ok:
                logger.info("CAN setup retry succeeded for %s", port)
            return ok
        except Exception as e:
            logger.debug("CAN setup retry failed: %s", e)
            return False

    # ── 驱动注入 ───────────────────────────────────────

    def bind_joint_driver(self, port: str, driver: object) -> None:
        """注入关节执行器驱动 (ST3215Driver / RobStrideOfficialDriver / ...)."""
        self._joint_drivers[port] = driver
        logger.info("ActuatorController bound joint driver on %s", port)

    # ── 注册 ───────────────────────────────────────────

    async def register_actuator(
        self, port: str, device_id: int, kind: str, brand: str,
        model: str = "", joint_name: str = "",
    ) -> None:
        key = (port, device_id)
        info = ActuatorInfo(
            port=port, device_id=device_id,
            kind=kind, brand=brand,
            model=model, joint_name=joint_name,
        )
        self._registry[key] = info
        if joint_name:
            self._joint_map[joint_name] = key
        logger.info("Registered: %s %s @ %s ID=%d joint=%s", brand, kind, port, device_id, joint_name)

    # ── 生命周期 ────────────────────────────────────────

    async def start(self) -> None:
        if not self._joint_drivers:
            return
        self._running = True
        # 位置保持已迁移到 Dispatcher._on_tick
        logger.info("ActuatorController started (%d drivers)", len(self._joint_drivers))

    async def stop(self) -> None:
        self._running = False
        await self.cleanup()

    # ── 位置控制 ───────────────────────────────────────

    async def set_position(
        self, port: str, device_id: int, position: float, time_ms: int = 500,
    ) -> bool:
        """设置单个执行器目标位置。

        Args:
            port: 物理端口（串口设备名或 CAN 接口）
            device_id: 执行器 ID
            position: 目标角度 (度)
            time_ms: 运动持续时间 (毫秒)

        Returns:
            bool: 是否成功
        """
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        try:
            if hasattr(driver, 'move_to_angle'):
                ok = driver.move_to_angle(device_id, position, time_ms=0)
            else:
                ok = driver.set_position(device_id, position, time_ms)
            if ok:
                self._last_targets[(port, device_id)] = (position, time_ms)
            return ok
        except Exception as e:
            logger.warning("set_position %d→%.2f failed: %s", device_id, position, e)
            return False

    async def set_positions(
        self, port: str, targets: Dict[int, float], time_ms: int = 500,
    ) -> bool:
        """批量设置执行器目标位置。

        Args:
            port: 物理端口
            targets: {device_id: angle} 映射
            time_ms: 运动持续时间

        Returns:
            bool: 至少一个设置成功返回 True
        """
        ok = 0
        for did, pos in targets.items():
            if await self.set_position(port, did, pos, time_ms):
                ok += 1
        return ok > 0

    # ── 速度控制 ───────────────────────────────────────

    async def set_velocity(self, port: str, device_id: int, velocity: float) -> bool:
        """设置执行器目标速度（速度模式）。

        Args:
            port: 物理端口
            device_id: 执行器 ID
            velocity: Feetech 原始速度值 (0~1023)，或 RobStride 速度 (rad/s)

        Returns:
            bool: 是否成功
        """
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        try:
            return driver.set_speed(device_id, velocity)  # Feetech 用 set_speed
        except Exception as e:
            logger.warning("set_velocity %d failed: %s", device_id, e)
            return False

    async def stop_actuator(self, port: str, device_id: int) -> bool:
        """停止单个执行器（设速度为 0）。"""
        return await self.set_velocity(port, device_id, 0)

    # ── 模式切换 (Feetech 专有 — hasattr) ───────────────

    async def set_velocity_mode(self, port: str, device_id: int) -> bool:
        """将执行器切换到速度模式（Feetech 专有）。"""
        driver = self._get_or_create_driver(port)
        if driver is None or not hasattr(driver, 'set_velocity_mode'):
            return False
        return driver.set_velocity_mode(device_id)

    async def set_position_mode(self, port: str, device_id: int) -> bool:
        """将执行器切换到位置控制模式（Feetech 专有）。"""
        driver = self._get_or_create_driver(port)
        if driver is None or not hasattr(driver, 'set_position_mode'):
            return False
        return driver.set_position_mode(device_id)

    # ── ID 管理 ────────────────────────────────────────

    async def change_id(self, port: str, old_id: int, new_id: int) -> bool:
        """修改执行器 ID（写入 EEPROM/Flash）。

        Args:
            port: 物理端口
            old_id: 当前 ID
            new_id: 新 ID

        Returns:
            bool: 是否成功
        """
        driver = self._get_or_create_driver(port)
        if driver is None or not hasattr(driver, 'set_id'):
            return False
        try:
            ok = driver.set_id(old_id, new_id)
            if ok:
                old_key = (port, old_id)
                new_key = (port, new_id)
                self._move_registry_key(old_key, new_key)
            return ok
        except Exception as e:
            logger.warning("change_id %d→%d failed: %s", old_id, new_id, e)
            return False

    def _move_registry_key(
        self, old_key: Tuple[str, int], new_key: Tuple[str, int],
    ) -> None:
        """ID 变更时迁移注册表、最后目标缓存和关节映射中的键。

        Args:
            old_key: 旧的 (port, device_id) 键
            new_key: 新的 (port, device_id) 键
        """
        if old_key in self._registry:
            info = self._registry.pop(old_key)
            info.device_id = new_key[1]
            self._registry[new_key] = info
        if old_key in self._last_targets:
            self._last_targets[new_key] = self._last_targets.pop(old_key)
        for jname, key in list(self._joint_map.items()):
            if key == old_key:
                self._joint_map[jname] = new_key

    # ── 使能/失能 ──────────────────────────────────────

    async def enable(self, port: str, device_id: int) -> bool:
        """使能执行器扭矩（上电）。"""
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        try:
            driver.set_torque(device_id, True)
            return True
        except Exception as e:
            logger.warning("enable %d failed: %s", device_id, e)
            return False

    async def disable(self, port: str, device_id: int) -> bool:
        """失能执行器扭矩（掉电）。"""
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        try:
            driver.set_torque(device_id, False)
            return True
        except Exception as e:
            logger.warning("disable %d failed: %s", device_id, e)
            return False

    async def disable_port(self, port: str) -> bool:
        """失能指定端口上的所有执行器。"""
        for (p, sid) in self._registry:
            if p == port:
                await self.disable(port, sid)
        return True

    # ── 零位校准 ───────────────────────────────────────

    async def set_zero_position(self, port: str, device_id: int) -> bool:
        """设置电机零位（将当前位置设为0），保存到Flash."""
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        try:
            if hasattr(driver, 'set_zero_position'):
                return driver.set_zero_position(device_id)
            logger.warning("Driver for %s does not support set_zero_position", port)
            return False
        except Exception as e:
            logger.warning("set_zero_position %d failed: %s", device_id, e)
            return False

    async def calibrate_servo_offsets(self, port: str, servo_ids: Optional[List[int]] = None) -> Dict[int, float]:
        """批量校准舵机零位偏移量。

        将舵机物理上转到期望零位后调用，读取当前编码器位置并反算 zero_offset。
        偏移量同时写入驱动内存 (id_to_offset)，即时生效。

        Args:
            port: 物理端口
            servo_ids: 要校准的舵机 ID 列表，为 None 则校准该端口所有在线 Feetech 舵机

        Returns:
            Dict[int, float]: {servo_id: zero_offset}，只包含成功校准的舵机
        """
        driver = self._get_or_create_driver(port)
        if driver is None:
            return {}
        if not hasattr(driver, 'batch_calibrate_offsets'):
            logger.warning("Driver for %s does not support batch_calibrate_offsets", port)
            return {}

        if servo_ids is None:
            # 收集该端口上所有 Feetech 舵机
            servo_ids = [sid for (p, sid) in self._registry if p == port]

        if not servo_ids:
            logger.warning("No servos to calibrate on %s", port)
            return {}

        logger.info("Calibrating %d servos on %s ...", len(servo_ids), port)
        results = driver.batch_calibrate_offsets(servo_ids)
        logger.info("Calibration done: %d/%d servos", len(results), len(servo_ids))
        return results

    # ── 状态读取 ───────────────────────────────────────

    async def get_actuator_info(self, port: str, device_id: int) -> Optional[Dict]:
        """读取执行器完整状态信息。

        Returns:
            dict: {device_id, port, position, velocity, torque, temperature, voltage, brand, joint_name} 或 None
        """
        driver = self._get_or_create_driver(port)
        if driver is None:
            return None
        try:
            state = driver.get_status(device_id)
            info: Dict = {"device_id": device_id, "port": port, **state}
            key = (port, device_id)
            if key in self._registry:
                reg = self._registry[key]
                info['brand'] = reg.brand
                info['joint_name'] = reg.joint_name
            return info
        except Exception as e:
            logger.warning("get_actuator_info %d failed: %s", device_id, e)
            return None

    async def read_positions(self, port: str, ids: List[int]) -> Dict[int, float]:
        """批量读取执行器当前位置。

        Args:
            port: 物理端口
            ids: 执行器 ID 列表

        Returns:
            Dict[int, float]: {device_id: position}
        """
        driver = self._get_or_create_driver(port)
        if driver is None:
            return {}
        results = {}
        for sid in ids:
            try:
                pos = driver.get_position(sid)
                if pos is not None:
                    results[sid] = pos
            except Exception:
                pass
        return results

    async def ping_actuator(self, port: str, device_id: int) -> bool:
        """检测执行器是否在线。

        Returns:
            bool: 在线返回 True
        """
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        try:
            return driver.ping(device_id)
        except Exception:
            return False

    # ── 端口扫描与发现 ──────────────────────────────────

    async def scan_available_ports(self) -> List[str]:
        """扫描系统中可用的串口和 CAN 接口。

        自动加载 CAN 内核驱动 (gs_usb)，确保刚插入的 USB-CAN 适配器能被发现。
        无需用户手动执行任何命令。

        Returns:
            List[str]: 端口列表，CAN 接口排在前面
        """
        ports: List[str] = []

        # ── 0) 自动加载 CAN 内核模块（幂等操作，已加载则跳过）──
        try:
            import subprocess
            import os
            for mod in ("can", "can_raw", "gs_usb"):
                try:
                    cmd = ["sudo", "-n", "modprobe", mod] if os.geteuid() != 0 else ["modprobe", mod]
                    subprocess.run(cmd, capture_output=True, timeout=3)
                except Exception:
                    pass  # 模块已加载或不可用，静默跳过
        except Exception:
            pass

        # ── 1) 扫描 CAN 接口 ──
        try:
            import glob as glob_mod
            can_ifaces = sorted(glob_mod.glob("/sys/class/net/can*"))
            can_matches = [os.path.basename(p) for p in can_ifaces]
            if can_matches:
                logger.info("Found CAN interfaces: %s", can_matches)
            # CAN 接口排在列表最前面
            ports = can_matches + ports
        except Exception:
            # 回退：用 ip link show 解析
            try:
                import subprocess
                import re
                result = subprocess.run(
                    ["ip", "-o", "link", "show"],
                    capture_output=True, text=True, timeout=2,
                )
                can_matches: List[str] = []
                for line in result.stdout.split("\n"):
                    m = re.search(r"\bcan\d+\b", line)
                    if m:
                        name = m.group(0)
                        if name not in can_matches:
                            can_matches.append(name)
                ports = can_matches + ports
            except Exception:
                pass

        # ── 2) 扫描 USB 串口 ──
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                if any(x in p.device for x in ("USB", "ACM", "ttyUSB")):
                    ports.append(p.device)
        except ImportError:
            logger.debug("pyserial not available, skip USB scan")

        logger.info("Found %d ports: %s", len(ports), ports)
        return ports

    @staticmethod
    def _parse_motor_type_from_brand(brand: str):
        """从 brand 字符串解析 RobStride 电机型号。

        例: robstride_04 → MotorType.RS04 (value=4)

        Args:
            brand: 格式为 "品牌_型号编号"，如 "robstride_04"

        Returns:
            Optional[MotorType]: 解析成功返回 MotorType 枚举值，否则返回 None
        """
        if not brand or "_" not in brand:
            return None
        try:
            model_num = int(brand.rsplit("_", 1)[-1])
        except (ValueError, IndexError):
            return None
        try:
            from aiderminal.drivers.actuator.robStride.robstride_dynamics.protocol import MotorType
            return MotorType(model_num)
        except ValueError:
            return None

    async def scan_servos(
        self, port: str, start_id: int = 1, end_id: int = 253,
    ) -> List[ActuatorInfo]:
        """扫描端口上在线执行器，自动注册并分配关节名。

        CAN 端口使用 scan_motors() 批量发现，USB 端口逐个 ping。
        扫描完成后自动同步关节映射到 dispatcher。

        Args:
            port: 物理端口
            start_id: 起始 ID
            end_id: 结束 ID

        Returns:
            List[ActuatorInfo]: 在线执行器列表
        """
        found: List[ActuatorInfo] = []
        brand = "robstride" if "can" in port.lower() else "feetech"

        # 连接池模式: 获取或创建 driver，扫描完成后不释放
        driver = self._get_or_create_driver(port)
        if driver is None:
            logger.warning("Failed to get/create driver for %s, scan aborted", port)
            return found

        try:
            if "can" in port.lower():
                # CAN / RobStride: 使用专用 scan_motors
                found_ids = driver.scan_motors(start_id, end_id)
                for mid in found_ids:
                    # 电机型号: manual override → DEFAULT_MOTOR_TYPE_MAP
                    motor_type = None
                    if mid in self._motor_type_overrides:
                        from aiderminal.drivers.actuator.robStride.robstride_dynamics.protocol import MotorType
                        try:
                            motor_type = MotorType(self._motor_type_overrides[mid])
                            logger.info("Motor ID=%d type override -> %s", mid, motor_type.name)
                        except ValueError:
                            logger.warning("Motor ID=%d invalid override type: %s", mid, self._motor_type_overrides[mid])
                    driver.add_motor(mid, motor_type=motor_type)
                    info = ActuatorInfo(
                        port=port, device_id=mid,
                        kind="joint", brand=brand,
                        is_online=True,
                    )
                    found.append(info)
                    self._registry[(port, mid)] = info
                    logger.info("Found RobStride motor ID=%d on %s", mid, port)
            else:
                # USB 串口 / Feetech: ping 逐个扫描
                for device_id in range(start_id, end_id + 1):
                    try:
                        if driver.ping(device_id):
                            info = ActuatorInfo(
                                port=port, device_id=device_id,
                                kind="joint", brand=brand,
                                is_online=True,
                            )
                            found.append(info)
                            self._registry[(port, device_id)] = info
                    except Exception:
                        continue

            # 关节名由 actuator_router 根据 servo_ids.yaml 统一分配，
            # 不再使用协议层硬编码的 DEFAULT_JOINT_ACTUATOR_MAP。

            logger.info("Scan %s complete: %d actuators, driver kept in pool", port, len(found))
        except Exception as e:
            logger.warning("Scan %s error: %s", port, e)

        return found

    # ── 批量同步控制 ────────────────────────────────────

    async def sync_write_positions(
        self, port: str, targets: Dict[int, float], time_ms: int = 500,
    ) -> bool:
        """批量同步写入位置（优先使用驱动层 SYNC_WRITE 指令）。

        Args:
            port: 物理端口
            targets: {device_id: position} 映射
            time_ms: 运动时间

        Returns:
            bool: 是否成功
        """
        if not targets:
            return True
        driver = self._get_or_create_driver(port)
        if driver is None:
            return False
        if hasattr(driver, 'sync_write_positions'):
            ok = driver.sync_write_positions(targets, time_ms)
            if ok:
                for did, pos in targets.items():
                    self._last_targets[(port, did)] = (pos, time_ms)
            return ok
        return await self.set_positions(port, targets, time_ms)

    # ── 批量运维操作 ────────────────────────────────────

    def _scan_and_register_motors(self, port: str):
        """扫描并注册总线上所有电机（内部复用）."""
        driver = self._get_or_create_driver(port)
        if driver is None:
            logger.warning("_scan_and_register_motors: driver for %s not available", port)
            return None
        found = driver.scan_motors(1, 16)
        if not found:
            logger.warning("_scan_and_register_motors: no motors found on %s", port)
            return None
        from aiderminal.drivers.actuator.robStride.robstride_dynamics.protocol import DEFAULT_MOTOR_TYPE_MAP
        for mid in found:
            driver.add_motor(mid, DEFAULT_MOTOR_TYPE_MAP.get(mid))
        return driver

    def enable_all_motors(self, port: str = "can0") -> bool:
        """使能总线上所有电机（自动扫描 + 注册 + 使能）."""
        driver = self._scan_and_register_motors(port)
        return driver is not None and driver.enable_all()

    def disable_all_motors(self, port: str = "can0") -> bool:
        """失能总线上所有电机（安全停机）."""
        driver = self._scan_and_register_motors(port)
        return driver is not None and driver.disable_all()

    def show_motor_status(self, port: str = "can0") -> None:
        """打印总线上所有电机状态."""
        driver = self._scan_and_register_motors(port)
        if driver is not None:
            driver.show_all_status()

    def gohome_all_motors(self, port: str = "can0", kp: float = 10.0, kd: float = 1.0) -> bool:
        """所有电机回到零位（MIT PD 模式）."""
        driver = self._scan_and_register_motors(port)
        if driver is None:
            return False
        driver.enable_all()
        return driver.move_all_to_zero(kp=kp, kd=kd)

    def set_zero_all_motors(self, port: str = "can0") -> bool:
        """将所有电机当前位置设为零位并写入 Flash（永久操作）."""
        driver = self._scan_and_register_motors(port)
        if driver is None:
            return False
        driver.disable_all()
        return driver.set_zero_all()

    # ── 全局急停 ───────────────────────────────────────

    async def emergency_stop_all(self) -> None:
        for port in list(self._joint_drivers.keys()):
            await self.disable_port(port)
        logger.info("ActuatorController emergency stop all")

    # ── 属性 & 生命周期 ─────────────────────────────────

    @property
    def registry(self) -> Dict[Tuple[str, int], ActuatorInfo]:
        return self._registry

    @property
    def joint_drivers(self) -> Dict[str, object]:
        return self._joint_drivers

    # ═══════════════════════════════════════════════════════════════════
    # motor_controller 独有方法（同步，兼容旧调用方）
    # ═══════════════════════════════════════════════════════════════════

    def discover_ports_by_ids(self, target_ids: set) -> Dict[int, str]:
        """
        根据舵机 ID 自动发现所在的端口（同步版）。

        Args:
            target_ids: 需要查找的舵机 ID 集合

        Returns:
            Dict[int, str]: {servo_id: port} 映射
        """
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        if loop.is_running():
            future = self._executor.submit(
                asyncio.run,
                self._async_discover_ports_by_ids(target_ids)
            )
            return future.result(timeout=30)
        return loop.run_until_complete(self._async_discover_ports_by_ids(target_ids))

    async def _async_discover_ports_by_ids(self, target_ids: set) -> Dict[int, str]:
        """discover_ports_by_ids 的异步实现。

        遍历所有可用端口，在每个端口上全量扫描舵机，再与目标 ID 匹配。
        - CAN: 用目标 ID 范围（scan_motors 批量扫描效率高）
        - USB: 用宽范围 (1~253) 扫描，防止漏掉不在目标范围内的舵机
        """
        ports = await self.scan_available_ports()
        if not ports:
            return {}
        id_to_port: Dict[int, str] = {}
        min_id_cfg = min(target_ids) if target_ids else 1
        max_id_cfg = max(target_ids) if target_ids else 253

        for port in ports:
            # CAN 用目标范围；USB/ACM 从 1 扫到配置最大 ID
            if "can" in port.lower():
                scan_start, scan_end = min_id_cfg, max_id_cfg
            else:
                scan_start = 1
                scan_end = max_id_cfg
            print(f"🔍 探测端口: {port} (ID {scan_start}~{scan_end}) ...")
            try:
                found = await self.scan_servos(port, start_id=scan_start, end_id=scan_end)
                found_ids = {info.device_id for info in found}
                matched = [info for info in found if info.device_id in target_ids]
                for info in matched:
                    id_to_port[info.device_id] = port
                if matched:
                    extra = sorted(found_ids - target_ids)
                    msg = f"  ✅ {port}: 找到 {len(matched)} 个目标舵机 {sorted([i.device_id for i in matched])}"
                    if extra:
                        msg += f"（另有 {len(extra)} 个不在配置中: {extra}）"
                    print(msg)
                elif found:
                    print(f"  ⚠️ {port}: 找到 {len(found)} 个舵机 {sorted(found_ids)}，"
                          f"但都未在配置目标 {sorted(target_ids)[:10]}{'...' if len(target_ids)>10 else ''} 中")
                else:
                    print(f"  ⚠️ {port}: 驱动已连接，但 ping 1~{scan_end} 全部未响应（无舵机在线）")
            except Exception as e:
                print(f"  ❌ {port}: 扫描异常: {e}")
                continue
        return id_to_port

    def scan_all_servos(self) -> Dict[str, List['ActuatorInfo']]:
        """全局扫描所有串口上的舵机（同步版）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        if loop.is_running():
            future = self._executor.submit(asyncio.run, self._async_scan_all_servos())
            return future.result(timeout=30)
        return loop.run_until_complete(self._async_scan_all_servos())

    async def _async_scan_all_servos(self) -> Dict[str, List['ActuatorInfo']]:
        """scan_all_servos 的异步实现。"""
        ports = await self.scan_available_ports()
        result: Dict[str, List['ActuatorInfo']] = {}
        for port in ports:
            found = await self.scan_servos(port)
            if found:
                result[port] = found
        return result

    def set_joint_angle(self, joint_name: str, angle_deg: float, time_ms: int = 500) -> bool:
        """按关节名称控制（同步包装）。"""
        if joint_name not in self._joint_map:
            logger.warning("未知关节: %s", joint_name)
            return False
        port, device_id = self._joint_map[joint_name]
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(
                asyncio.run,
                self.set_position(port, device_id, angle_deg, time_ms)
            ).result(timeout=10)
        return loop.run_until_complete(self.set_position(port, device_id, angle_deg, time_ms))

    # ── 同步包装（兼容 motor_controller 调用方）───────────────

    def register_servo(self, port: str, device_id: int, brand: str, joint_name: str = "") -> None:
        """注册舵机（同步包装 register_actuator）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            self._executor.submit(asyncio.run, self.register_actuator(port, device_id, 254, brand, "", joint_name)).result(timeout=10)
            return
        loop.run_until_complete(self.register_actuator(port, device_id, 254, brand, "", joint_name))

    def set_servo_angle(self, port: str, device_id: int, angle_deg: float, time_ms: int = 500) -> bool:
        """设置舵机角度（同步包装 set_position）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.set_position(port, device_id, angle_deg, time_ms)).result(timeout=10)
        return loop.run_until_complete(self.set_position(port, device_id, angle_deg, time_ms))

    def set_servos_angles(self, port: str, targets: Dict[int, float], time_ms: int = 500) -> bool:
        """批量设置舵机角度（同步包装 set_positions）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.set_positions(port, targets, time_ms)).result(timeout=10)
        return loop.run_until_complete(self.set_positions(port, targets, time_ms))

    def change_servo_id(self, port: str, old_id: int, new_id: int) -> bool:
        """修改舵机ID（同步包装 change_id）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.change_id(port, old_id, new_id)).result(timeout=10)
        return loop.run_until_complete(self.change_id(port, old_id, new_id))

    def set_servo_velocity_mode(self, port: str, device_id: int) -> bool:
        """切换舵机到速度模式（同步包装 set_velocity_mode）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.set_velocity_mode(port, device_id)).result(timeout=10)
        return loop.run_until_complete(self.set_velocity_mode(port, device_id))

    def write_positions_sync(self, port: str, targets: Dict[int, float], time_ms: int = 500) -> bool:
        """批量同步写入位置（同步包装 sync_write_positions）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.sync_write_positions(port, targets, time_ms)).result(timeout=5)
        return loop.run_until_complete(self.sync_write_positions(port, targets, time_ms))

    def set_servo_speed(self, port: str, device_id: int, speed: int) -> bool:
        """设置舵机速度（同步包装 set_velocity）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.set_velocity(port, device_id, float(speed))).result(timeout=5)
        return loop.run_until_complete(self.set_velocity(port, device_id, float(speed)))

    def scan_servos_on_port(self, port: str, start_id: int = 1, end_id: int = 253) -> List:
        """扫描端口舵机（同步包装 scan_servos，返回 ServoInfo 格式）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.scan_servos(port, start_id, end_id)).result(timeout=30)
        return loop.run_until_complete(self.scan_servos(port, start_id, end_id))

    def get_servo_info(self, port: str, device_id: int) -> Optional[Dict]:
        """获取舵机信息（同步包装 get_actuator_info）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.get_actuator_info(port, device_id)).result(timeout=5)
        return loop.run_until_complete(self.get_actuator_info(port, device_id))

    def set_torque(self, port: str, device_id: int, enable: bool) -> bool:
        """设置扭矩（同步包装 enable/disable）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(
                asyncio.run,
                self.enable(port, device_id) if enable else self.disable(port, device_id)
            ).result(timeout=5)
        return loop.run_until_complete(
            self.enable(port, device_id) if enable else self.disable(port, device_id)
        )
    def disable_all_torques(self, port: str) -> bool:
        """全部禁用扭矩（同步包装 disable_port）。"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        if loop.is_running():
            return self._executor.submit(asyncio.run, self.disable_port(port)).result(timeout=5)
        return loop.run_until_complete(self.disable_port(port))

    def force_reinitialize_all_robstride(self) -> None:
        """强制清空所有 RobStride 驱动的初始化标记。

        下次 set_position 时电机会自动重新使能，用于恢复因过流/看门狗
        等物理失能的电机，无需断电重启。
        """
        for port, driver in list(self._joint_drivers.items()):
            if hasattr(driver, "force_reinitialize"):
                driver.force_reinitialize()
                logger.info("Reinit marker cleared for driver on %s", port)

    async def cleanup(self) -> None:
        for port, driver in list(self._joint_drivers.items()):
            try:
                driver.disconnect()
            except Exception:
                pass
        self._joint_drivers.clear()
        self._registry.clear()
        self._joint_map.clear()
        self._last_targets.clear()
        self._executor.shutdown(wait=False, cancel_futures=True)
        logger.info("ActuatorController cleaned up")
