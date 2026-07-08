"""
执行器命令路由 — 统一调度所有执行器类型。

消息格式:
  1. actuator_command (实时控制):
     {"type":"actuator_command","servos":{id:pos},"motors":{"vx":...}}

  2. actuator_stop (急停):
     {"type":"actuator_stop"}

  3. api_command category=motor (管理 CRUD):
     {"type":"api_command","category":"motor","action":"scan_servos",...}
"""
import logging
from typing import Any, Dict, List

logger = logging.getLogger(__name__)


class ActuatorRouter:
    """执行器命令路由 — 自注册到 CommandRouter.
    
    负责：
      - 实时控制 (actuator_command / actuator_stop)
      - 真机连接/断开 (robot_connect / robot_disconnect)
      - 管理 CRUD (scan / ping / set_id / calibrate / ...)
    """

    # 本路由处理的命令类型
    _ROUTES = ("actuator_command", "actuator_stop")

    def __init__(self, control_loop=None) -> None:
        self._actuator_ctrl = None
        self._servo_cfg = None
        self._pipeline = None
        self._control_loop = control_loop

    def _get_actuator_ctrl(self):
        """动态获取 ActuatorController（优先绑定的，否则从 control_loop）。"""
        if self._actuator_ctrl is not None:
            return self._actuator_ctrl
        if self._control_loop and hasattr(self._control_loop, 'motor_controller'):
            return self._control_loop.motor_controller
        return None

    def bind_actuator_controller(self, ctrl) -> None:
        """绑定 ActuatorController."""
        self._actuator_ctrl = ctrl

    @property
    def _ctrl(self):
        """获取 controller（优先绑定，否则从 control_loop 兜底）。"""
        if self._actuator_ctrl is not None:
            return self._actuator_ctrl
        if self._control_loop and hasattr(self._control_loop, 'motor_controller'):
            return self._control_loop.motor_controller
        return None

    def bind_servo_config(self, servo_cfg) -> None:
        """绑定 ServoConfigManager（连接时读取 brand / joint_name）。"""
        self._servo_cfg = servo_cfg

    def bind_pipeline(self, pipeline) -> None:
        """绑定 MotionPipeline（连接时同步驱动池）。"""
        self._pipeline = pipeline

    def register_with(self, cmd_router: "CommandRouter") -> None:
        """向 CommandRouter 注册本路由的所有处理器."""
        cmd_router.register("actuator_command", self.handle_command)
        cmd_router.register("actuator_stop", lambda _: self.handle_stop())

    # ================================================================
    # 消息入口: actuator_command (直接控制)
    # ================================================================

    async def handle_command(self, payload: Dict) -> None:
        """
        处理 actuator_command.

        格式:
          {"servos": {id: position}, "time_ms": 500}
          {"motors": {"vx": 0.5, "vy": 0, "omega": 0}}
        """
        ctrl = self._ctrl
        if ctrl is None:
            logger.warning("ActuatorController not bound")
            return

        # 关节执行器控制 — 遍历所有已绑定驱动的端口
        servos = payload.get("servos", {})
        if servos:
            time_ms = payload.get("time_ms", 500)
            for port in ctrl.joint_drivers:
                await ctrl.set_positions(port, servos, time_ms)

        # 底盘控制 — 暂未实现

    # ================================================================
    # 消息入口: actuator_stop
    # ================================================================

    async def handle_stop(self) -> None:
        """急停所有执行器."""
        ctrl = self._ctrl
        if ctrl is not None:
            await ctrl.emergency_stop_all()

    # ================================================================
    # 真机连接管理
    # ================================================================

    async def handle_robot_connection(self, cmd: Dict) -> None:
        """处理 robot_connect / robot_disconnect."""
        action = cmd.get("action", "")
        if action == "robot_connect":
            logger.info("Received robot_connect command")
            print("\n🔌 正在连接真机...")
            try:
                await self._connect_actuators()
                print("✅ 真机连接成功")
            except Exception as e:
                logger.error("Failed to connect robot: %s", e)
                print(f"❌ 真机连接失败: {e}")
        elif action == "robot_disconnect":
            logger.info("Received robot_disconnect command")
            print("\n🔌 正在断开真机...")
            if self._ctrl:
                await self._ctrl.stop()
                print("✅ 真机已断开")

    async def _connect_actuators(self) -> None:
        """按配置 brand 自动选择驱动，扫描并注册执行器，启动位置保持。

        品牌路由:
          - robstride_* → can0 (RobStrideOfficialDriver / CAN)
          - feetech_*   → /dev/ttyACM* / /dev/ttyUSB* (ST3215Driver / 串口)
        """
        from aiderminal.controller.actuator_controller import ActuatorController

        servo_cfg = self._servo_cfg
        pipeline = self._pipeline

        if servo_cfg is None:
            raise RuntimeError("ServoConfigManager 未绑定")
        if pipeline is None:
            raise RuntimeError("MotionPipeline 未绑定")

        total = servo_cfg.get_servo_count()
        if total == 0:
            print("  ℹ️  执行器控制未启用（无配置）")
            return

        # 创建 ActuatorController（连接时才创建，每次重新连覆盖旧的）
        motor_type_overrides = servo_cfg.build_motor_type_overrides()
        self._actuator_ctrl = ActuatorController(
            motor_type_overrides=motor_type_overrides,
        )
        self._actuator_ctrl.set_pipeline(pipeline)
        logger.info("Pipeline ↔ ActuatorController linked")

        # ── 按品牌分组 ────────────────────────────────────
        robstride_ids: List[int] = []
        feetech_ids: List[int] = []
        for sid in servo_cfg.get_all_servo_ids():
            brand = servo_cfg.get_brand(sid)
            if brand.startswith("robstride"):
                robstride_ids.append(sid)
            elif brand.startswith("feetech"):
                feetech_ids.append(sid)
            else:
                feetech_ids.append(sid)

        logger.info(
            "Servo group: robstride=%s (n=%d), feetech=%s (n=%d)",
            robstride_ids, len(robstride_ids),
            feetech_ids, len(feetech_ids),
        )

        available_ports = await self._actuator_ctrl.scan_available_ports()
        logger.info("Available ports: %s", available_ports)

        ctrl = self._actuator_ctrl

        # ── 1. RobStride CAN 电机 ──────────────────────────
        if robstride_ids:
            can_port = next(
                (p for p in available_ports if "can" in p.lower()), None,
            )
            if can_port:
                try:
                    found = await ctrl.scan_servos(
                        can_port,
                        start_id=min(robstride_ids),
                        end_id=max(robstride_ids),
                    )
                    for info in found:
                        sid = info.device_id
                        info.brand = servo_cfg.get_brand(sid)
                        joint_name = servo_cfg.get_joint_name(sid)
                        if joint_name:
                            info.joint_name = joint_name
                            ctrl._joint_map[joint_name] = (can_port, sid)
                    online = len(found)
                    expected = len(robstride_ids)
                    print(
                        f"  🦾 RobStride {can_port}: "
                        f"{online}/{expected} 个电机 {[i.device_id for i in found]}"
                    )
                except Exception as e:
                    logger.warning("RobStride CAN init failed: %s", e)
                    print(f"  ⚠️  {can_port} RobStride 初始化失败: {e}")
            else:
                print(
                    f"  ⚠️  配置了 {len(robstride_ids)} 个 RobStride 电机，"
                    f"但未发现 CAN 接口"
                )

        # ── 2. Feetech 串口舵机 ─────────────────────────────
        if feetech_ids:
            serial_ports = [
                p for p in available_ports if "can" not in p.lower()
            ]
            if not serial_ports:
                print(
                    f"  ⚠️  配置了 {len(feetech_ids)} 个 Feetech 舵机，"
                    f"但未发现串口"
                )
            for port in serial_ports:
                try:
                    found = await ctrl.scan_servos(
                        port,
                        start_id=min(feetech_ids),
                        end_id=max(feetech_ids),
                    )
                    matched = [
                        info for info in found
                        if info.device_id in feetech_ids
                    ]
                    for info in matched:
                        sid = info.device_id
                        info.brand = servo_cfg.get_brand(sid)
                        joint_name = servo_cfg.get_joint_name(sid)
                        if joint_name:
                            info.joint_name = joint_name
                            ctrl._joint_map[joint_name] = (port, sid)
                    if matched:
                        print(
                            f"  🦾 Feetech {port}: "
                            f"{len(matched)} 个舵机 "
                            f"{[i.device_id for i in matched]}"
                        )
                except Exception as e:
                    logger.warning(
                        "Feetech serial init on %s failed: %s", port, e,
                    )
                    print(f"  ⚠️  {port} Feetech 初始化失败: {e}")

        # ── 3. 同步驱动池到 Pipeline（扫描完成后 driver 才存在）──
        pipeline.sync_drivers(self._actuator_ctrl)
        logger.info("Pipeline synced for dispatch (%d drivers)", len(self._actuator_ctrl._joint_drivers))

        # ── 4. 启动位置保持 ─────────────────────────────────
        await ctrl.start()

        # ── 5. 底盘控制状态 ────────────────────────────────
        base_ids = [
            sid for sid in robstride_ids
            if servo_cfg.get_group_name(sid) == "base"
        ]
        if base_ids:
            base_online = any(
                (p, sid) in ctrl.registry
                for p in available_ports for sid in base_ids
            )
            if base_online:
                print("  ✅ 底盘控制已启用")
            else:
                print("  ⚠️  底盘电机未在线")
        else:
            print("  ℹ️  底盘控制未启用（待配置）")

    # ================================================================
    # motor 路由 + 响应发送
    # ================================================================

    async def route_and_respond(self, cmd: Dict, ws) -> None:
        """路由 motor 命令并发送响应。"""
        import json
        result = await self.route(cmd)
        action = cmd.get("action", "")
        
        logger.info("motor route %s: success=%s data=%s",
                     action, result.get("success"), result.get("count", result.get("data")))
        
        # 根据 action 决定响应类型
        if action == "scan_servos":
            response = {"type": "scan_servos_response", "result": result.get("data", [])}
        elif action == "list_ports":
            response = {"type": "list_ports_response", "ports": result.get("data", [])}
        elif action == "get_servo_info":
            servo_id = cmd.get("servo_id")
            info_data = result.get("data", {})
            response = {"type": "servo_info_response", "servo_id": servo_id, **info_data}
        elif action == "get_network_info":
            net_data = result.get("data", {})
            response = {"type": "network_info_response", **net_data}
        else:
            response = {"type": "api_response", "category": "motor", "action": action, **result}
        
        try:
            await ws.send_raw(json.dumps(response, default=str))
        except Exception as e:
            logger.warning("Failed to send motor response: %s", e)

    # ================================================================
    # api_command (category=motor) 路由 — 管理 CRUD
    # ================================================================

    async def route(self, cmd: Dict[str, Any]) -> Dict:
        """
        路由 api_command (category=motor).

        Returns:
            Dict: {success, data?, message?}
        """
        action = cmd.get("action", "")
        if not action:
            return {"success": False, "message": "Missing action"}

        ctrl = self._ctrl
        if ctrl is None:
            return {"success": False, "message": "ActuatorController not initialized"}

        route_map = {
            "set_servo_angle":   self._route_set_position,
            "set_servo_speed":   self._route_set_velocity,
            "set_servo_id":      self._route_change_id,
            "set_speed_mode":    self._route_speed_mode,
            "set_position_mode": self._route_position_mode,
            "scan_servos":       self._route_scan,
            "list_ports":        self._route_list_ports,
            "get_network_info":  self._route_network_info,
            "ping_servo":        self._route_ping,
            "get_servo_info":    self._route_get_info,
            "set_torque":        self._route_set_torque,
            "disable_torques":   self._route_disable_torques,
            "sync_positions":    self._route_sync_positions,
            "calibrate_servo_zero": self._route_set_zero,
            "calibrate_all_servo_offsets": self._route_calibrate_all_offsets,
            "calibrate_single_servo_offset": self._route_calibrate_single_offset,
            "goto_pose":         self._route_goto_pose,
            "list_poses":        self._route_list_poses,
        }

        handler = route_map.get(action)
        if handler:
            return await handler(ctrl, cmd)
        else:
            logger.debug("Unhandled actuator action: %s", action)
            return {"success": False, "message": f"Unknown action: {action}"}

    # ── 路由处理器 ───────────────────────────────────

    async def _route_set_position(self, ctrl, cmd: Dict) -> Dict:
        """设置执行器目标角度。

        cmd 参数: port, servo_id, angle, time_ms
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        position = cmd.get("angle")  # 兼容旧字段名
        time_ms = cmd.get("time_ms", 500)
        if device_id is None or position is None:
            return {"success": False, "message": "Missing servo_id or angle"}
        ok = await ctrl.set_position(port, device_id, position, time_ms)
        return {"success": ok, "message": f"actuator {device_id} → {position}" if ok else "Failed"}

    async def _route_set_velocity(self, ctrl, cmd: Dict) -> Dict:
        """设置执行器速度（速度模式）。

        cmd 参数: port, servo_id, speed
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        speed = cmd.get("speed")
        if device_id is None or speed is None:
            return {"success": False, "message": "Missing servo_id or speed"}
        speed = int(speed)  # 防御: JSON 反序列化可能产生 float，确保传给 SDK 的是 int
        ok = await ctrl.set_velocity(port, device_id, speed)
        return {"success": ok, "message": f"actuator {device_id} speed={speed}" if ok else "Failed"}

    async def _route_change_id(self, ctrl, cmd: Dict) -> Dict:
        """修改执行器 ID。

        cmd 参数: port, old_id, new_id
        """
        port = cmd.get("port", "/dev/ttyACM0")
        old_id = cmd.get("old_id")
        new_id = cmd.get("new_id")
        if old_id is None or new_id is None:
            return {"success": False, "message": "Missing old_id or new_id"}
        ok = await ctrl.change_id(port, old_id, new_id)
        return {"success": ok, "message": f"ID {old_id} → {new_id}" if ok else "Failed"}

    async def _route_speed_mode(self, ctrl, cmd: Dict) -> Dict:
        """切换执行器到速度模式。

        cmd 参数: port, servo_id
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        if device_id is None:
            return {"success": False, "message": "Missing servo_id"}
        ok = await ctrl.set_velocity_mode(port, device_id)
        return {"success": ok, "message": f"actuator {device_id} → speed mode" if ok else "Failed"}

    async def _route_position_mode(self, ctrl, cmd: Dict) -> Dict:
        """切换执行器到位置控制模式。

        cmd 参数: port, servo_id
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        if device_id is None:
            return {"success": False, "message": "Missing servo_id"}
        ok = await ctrl.set_position_mode(port, device_id)
        return {"success": ok, "message": f"actuator {device_id} → position mode" if ok else "Failed"}

    async def _route_scan(self, ctrl, cmd: Dict) -> Dict:
        """扫描端口上的在线执行器。

        cmd 参数: port, start_id, end_id
        返回: {success, data: [{id, port, brand, online}], count}
        """
        port = cmd.get("port", "/dev/ttyACM0")
        start_id = cmd.get("start_id", 1)
        end_id = cmd.get("end_id", 253)
        found = await ctrl.scan_servos(port, start_id, end_id)
        result = [
            {"id": s.device_id, "port": s.port, "brand": s.brand, "online": s.is_online}
            for s in found
        ]
        return {"success": True, "data": result, "count": len(result)}

    async def _route_list_ports(self, ctrl, cmd: Dict) -> Dict:
        """获取系统可用端口列表（串口 + CAN）。"""
        ports = await ctrl.scan_available_ports()
        return {"success": True, "data": ports}

    async def _route_network_info(self, ctrl, cmd: Dict) -> Dict:
        """获取系统网络信息。"""
        try:
            from aiderminal.utils.network_utils import get_network_info
            return {"success": True, "data": get_network_info()}
        except ImportError:
            return {"success": False, "message": "network_utils not available"}

    async def _route_ping(self, ctrl, cmd: Dict) -> Dict:
        """检测执行器是否在线。

        cmd 参数: port, servo_id
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        if device_id is None:
            return {"success": False, "message": "Missing servo_id"}
        online = await ctrl.ping_actuator(port, device_id)
        return {"success": online, "data": {"servo_id": device_id, "online": online}}

    async def _route_get_info(self, ctrl, cmd: Dict) -> Dict:
        """获取执行器状态信息。不传 servo_id 则返回所有注册的执行器。

        cmd 参数: port, servo_id (可选)
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        if device_id is None:
            all_info: Dict[int, Dict] = {}
            for (p, sid) in ctrl.registry:
                info = await ctrl.get_actuator_info(p, sid)
                if info:
                    all_info[sid] = info
            return {"success": True, "data": all_info}
        info = await ctrl.get_actuator_info(port, device_id)
        if info:
            return {"success": True, "data": info}
        return {"success": False, "message": f"Actuator {device_id} not found"}

    async def _route_set_torque(self, ctrl, cmd: Dict) -> Dict:
        """使能/失能执行器扭矩。

        cmd 参数: port, servo_id, enable (bool)
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        enable = cmd.get("enable", True)
        if device_id is None:
            return {"success": False, "message": "Missing servo_id"}
        if enable:
            ok = await ctrl.enable(port, device_id)
        else:
            ok = await ctrl.disable(port, device_id)
        state = "enabled" if enable else "disabled"
        return {"success": ok, "message": f"Torque {state}" if ok else "Failed"}

    async def _route_disable_torques(self, ctrl, cmd: Dict) -> Dict:
        """失断所有执行器扭矩（整端口）。

        cmd 参数: port
        """
        port = cmd.get("port", "/dev/ttyACM0")
        await ctrl.disable_port(port)
        return {"success": True, "message": "All torques disabled"}

    async def _route_sync_positions(self, ctrl, cmd: Dict) -> Dict:
        """批量同步写入执行器位置。

        cmd 参数: port, targets (dict or JSON string), time_ms
        """
        port = cmd.get("port", "/dev/ttyACM0")
        targets = cmd.get("targets", {})
        time_ms = cmd.get("time_ms", 500)
        if not targets:
            return {"success": False, "message": "Missing targets"}
        if isinstance(targets, str):
            import json
            targets = json.loads(targets)
        targets = {int(k): float(v) for k, v in targets.items()}
        ok = await ctrl.sync_write_positions(port, targets, time_ms)
        return {"success": ok, "message": "Sync positions" if ok else "Failed"}

    async def _route_set_zero(self, ctrl, cmd: Dict) -> Dict:
        """设置执行器当前角度为零位并保存到 Flash。

        cmd 参数: port, servo_id
        """
        port = cmd.get("port", "/dev/ttyACM0")
        device_id = cmd.get("servo_id")
        if device_id is None:
            return {"success": False, "message": "Missing servo_id"}
        ok = await ctrl.set_zero_position(port, device_id)
        return {
            "success": ok,
            "message": f"Zero set for servo {device_id}" if ok else "Failed to set zero",
        }

    async def _route_calibrate_all_offsets(self, ctrl, cmd: Dict) -> Dict:
        """批量校准所有 Feetech 舵机的零位偏移量并写回配置 YAML。

        流程:
        1. 遍历所有 Feetech 端口，读取每个舵机当前位置
        2. 反算 zero_offset = (raw_pos / 4095 * 360) - 180
        3. 更新驱动内存中的 id_to_offset（即时生效）
        4. 通过 Server API 写回 servo_ids.yaml

        cmd 参数: port (可选，不传则校准所有 Feetech 端口)
        """
        # 收集所有需要校准的舵机（Feetech 品牌）
        calibrate_ports = []
        specified_port = cmd.get("port")
        if specified_port:
            if specified_port in ctrl.joint_drivers:
                calibrate_ports = [specified_port]
            else:
                return {"success": False, "message": f"Port {specified_port} not connected"}
        else:
            calibrate_ports = list(ctrl.joint_drivers.keys())

        all_offsets: Dict[int, float] = {}
        for port in calibrate_ports:
            try:
                port_offsets = await ctrl.calibrate_servo_offsets(port)
                all_offsets.update(port_offsets)
            except Exception as e:
                logger.warning("Calibrate %s failed: %s", port, e)

        if not all_offsets:
            return {"success": False, "message": "No servos calibrated"}

        # 写回配置到 Server
        try:
            from aiderminal.comm.api.client import ServerAPIClient
            api = ServerAPIClient()
            ok = api.batch_calibrate_servo_zeros(all_offsets)
            if ok:
                return {
                    "success": True,
                    "message": f"Calibrated {len(all_offsets)} servos, config saved",
                    "data": all_offsets,
                }
            else:
                return {
                    "success": True,
                    "message": f"Calibrated {len(all_offsets)} servos but failed to save config",
                    "data": all_offsets,
                }
        except Exception as e:
            logger.warning("Failed to save calibration to server: %s", e)
            return {
                "success": True,
                "message": f"Calibrated {len(all_offsets)} servos, but config save failed: {e}",
                "data": all_offsets,
            }

    async def _route_calibrate_single_offset(self, ctrl, cmd: Dict) -> Dict:
        """校准单个舵机的零位偏移量并写回配置 YAML。

        cmd 参数: port, servo_id
        """
        port = cmd.get("port", "/dev/ttyACM0")
        servo_id = cmd.get("servo_id")
        if servo_id is None:
            return {"success": False, "message": "Missing servo_id"}

        # 复用批量校准接口，只校准一个舵机
        all_offsets = await ctrl.calibrate_servo_offsets(port, [servo_id])
        if not all_offsets or servo_id not in all_offsets:
            return {"success": False, "message": f"Servo {servo_id} calibration failed"}

        # 写回配置到 Server
        try:
            from aiderminal.comm.api.client import ServerAPIClient
            api = ServerAPIClient()
            ok = api.batch_calibrate_servo_zeros(all_offsets)
            if ok:
                return {
                    "success": True,
                    "message": f"Servo {servo_id} offset calibrated: {all_offsets[servo_id]}°",
                    "data": all_offsets,
                }
            else:
                return {
                    "success": True,
                    "message": f"Offset computed but config save failed",
                    "data": all_offsets,
                }
        except Exception as e:
            logger.warning("Failed to save single offset: %s", e)
            return {
                "success": True,
                "message": f"Offset computed ({all_offsets[servo_id]}°) but save failed: {e}",
                "data": all_offsets,
            }

    async def _route_goto_pose(self, ctrl, cmd: Dict) -> Dict:
        """将机器人移动到指定姿态（安全/默认等）。

        cmd 参数: arm (left/right/both), pose_name (safe/default/...)
        """
        arm = cmd.get("arm", "both")
        pose_name = cmd.get("pose_name", "safe")
        cl = self._control_loop
        if cl and cl.robot_interface:
            return await cl.robot_interface.goto_pose(arm, pose_name)
        return {"success": False, "message": "Robot interface not available"}

    async def _route_list_poses(self, ctrl, cmd: Dict) -> Dict:
        """返回当前机器人类型可用的姿态预设列表。

        cmd 参数: 无
        """
        cl = self._control_loop
        if cl and cl.robot_interface:
            poses = cl.robot_interface.list_poses()
            return {"success": True, "data": poses}
        return {"success": False, "message": "Robot interface not available"}
