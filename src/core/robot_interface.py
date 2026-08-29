"""
机器人接口模块。
提供带安全检查的机器人设备封装和便捷方法。

架构: 计算逻辑委托给 robots/ 下的适配器（AiderAdapter 或 AlohaAdapter），
本模块负责硬件通信和流程编排。
"""

import numpy as np
import time
import logging
import os
import sys
import yaml
import asyncio

from src.inputs.base import is_any_input_active
from pathlib import Path
from typing import Optional, Dict, Tuple

from src.config.settings import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE,
    WRIST_FLEX_INDEX, URDF_TO_INTERNAL_NAME_MAP,
    get_robot_type,
)


class RobotInterface:
    """机器人控制高级接口。硬件通信在此，计算逻辑委托给机器人适配器。"""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.robot_type = config.robot_type
        self.left_robot = None
        self.right_robot = None
        self.base_robot = None
        self.is_connected = False
        self.is_engaged = False
        self._servo_limits_fetched = False
        
        # 底层电机控制器
        from src.controller.actuator_controller import ActuatorController
        self.motor_controller = ActuatorController()

        # 各机械臂连接状态
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # 舵机 ID 配置（扁平结构：{left_arm: {joint: {id, ...}}, right_arm: ..., base: ..., ...}）
        self.servo_ids = {}
        
        # 端口到部位的映射（运行时自动发现：{part_name: port}）
        self.servo_ports = {}

        # 在线舵机 ID → 端口映射（连接时自动发现）
        self.online_servos = {}  # {id: port}
        
        # 底盘和升降轴状态
        self.base_motors = []
        self.lift_motor = None

        # ---- 机器人适配器 (根据类型动态选择) ----
        self._load_adapter()

        # 关节限位 (由 adapter 管理，此处为向后兼容)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)

        # 控制时序
        self.last_send_time = 0
        self._hw_lock = asyncio.Lock()  # 硬件写入锁，防止串口并发写入
        self._hw_version = 0  # 硬件写入版本号，旧任务拿锁后跳过

        # 错误跟踪
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3

        # 多圈丢失检测：连接后检测到的掉圈数电机列表
        self.lost_multiturn_motors = []  # [{id, joint_name, part, raw_angle, corrected_angle}]
        self.max_general_errors = 8

        # 安全关机初始位置 (由适配器类型决定)
        from src.config.settings import get_robot_initial_arm
        self.initial_left_arm = np.array(get_robot_initial_arm("left"))
        self.initial_right_arm = np.array(get_robot_initial_arm("right"))

        # 底盘状态 (由 control_loop 更新)
        self.base_connected = False
        self.base_velocity_target = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # 升降轴状态 (由 control_loop 更新)
        self.lift_connected = False
        self.lift_height_mm = 0
        self.lift_velocity = 0

        # 仿真相关状态 (由 control_loop 更新)
        self.vr_raw_data = {}
        self.left_arm_state = None
        self.right_arm_state = None
        self.visualizer = None

    def _load_adapter(self):
        """根据 robot_type 动态加载对应的适配器。"""
        if self.robot_type == "aloha":
            from src.robots.aloha import AlohaAdapter
            self.adapter = AlohaAdapter()
            print("[RobotInterface] 使用 AlohaAdapter (6-DOF, SO100)")
        else:
            from src.robots.aider import AiderAdapter
            self.adapter = AiderAdapter()
            print("[RobotInterface] 使用 AiderAdapter (8-DOF)")

    # ---- 属性：向后兼容（委托给 adapter） ----

    @property
    def left_arm_angles(self):
        return self.adapter.left_angles

    @left_arm_angles.setter
    def left_arm_angles(self, value):
        self.adapter.left_angles = value

    @property
    def right_arm_angles(self):
        return self.adapter.right_angles

    @right_arm_angles.setter
    def right_arm_angles(self, value):
        self.adapter.right_angles = value

    def set_arm_angles(self, arm: str, angles, clamp: bool = True):
        """写入臂关节角度。clamp=True（默认）时经软限位钳制，覆盖姿态预设等命令路径。

        注意：硬件反馈读回请用 left_arm_angles/right_arm_angles 属性直写（不钳制，保留真实读数）。
        """
        arr = np.asarray(angles, dtype=float)
        if clamp and self.adapter is not None:
            arr = self.adapter._clamp_arm_angles(arm, arr)
        if arm == "left":
            self.adapter.left_angles = arr
        else:
            self.adapter.right_angles = arr

    # 姿态语义名 → URDF 关节名 映射（供 set_body_joint 使用）
    _BODY_JOINT_MAP = {
        "waist":      "waist_Link",
        "head_yaw":   "head_Link",
        "head_pitch": "head_Link2",
    }

    def set_body_joint(self, name: str, angle_rad: float):
        """写入身体关节（腰/头）角度，经软限位钳制。命令路径统一入口。

        注意：硬件反馈读回请直写 ri.adapter.waist_angle 等属性（不钳制，保留真实读数）。
        """
        if self.adapter is None:
            return
        urdf_name = self._BODY_JOINT_MAP.get(name, name)
        self.adapter.set_body_joint_absolute(urdf_name, float(angle_rad))

    def set_servo_ids_config(self, config: dict):
        """设置舵机 ID 配置（从 Server 获取，扁平结构，无 bus 包装）"""
        if not config:
            print("⚠️ 收到空的舵机配置")
            return False
        
        self.servo_ids = config
        
        # 更新底盘和升降轴引用（扁平结构直接读 base / lift_axis）
        try:
            base_config = config.get('base', {})
            lift_config = config.get('lift_axis', {})
            
            if base_config:
                self.base_motors = list(base_config.keys())
                print(f"✅ 底盘舵机配置: {self.base_motors}")
            
            if lift_config:
                lift_key = next(iter(lift_config)) if lift_config else None
                if lift_key:
                    self.lift_motor = lift_config[lift_key].get('id')
            
            print(f"✅ 舵机配置已更新: {len(config)} 个部位配置")

            # 用 yaml 限位覆盖 JOINT_LIMIT_OVERRIDES（(0,0) 兜底占位 → 启动拉 yaml 后覆盖为真实限位，重启生效）
            from src.robots.aider.settings import apply_servo_limits_from_yaml
            apply_servo_limits_from_yaml(config)

            return True
        except Exception as e:
            print(f"❌ 解析舵机配置失败: {e}")
            return False

    def fetch_servo_limits(self) -> bool:
        """轻量：仅从 Server 拉 servo_ids.yaml 并覆盖 JOINT_LIMIT_OVERRIDES（不连真机硬件）。

        用于启动早期在 setup_kinematics 之前同步关节限位，避免在 --env-dev/无硬件环境下卡住。
        返回是否成功拉到配置。
        """
        if self._servo_limits_fetched:
            return True
        try:
            from src.comm.api.client import ServerAPIClient
            api_client = ServerAPIClient()
            servo_config = api_client.get_servo_ids_config()
            if not servo_config:
                print("❌ 未能从 Server 获取舵机配置（fetch_servo_limits）")
                return False
            self.set_servo_ids_config(servo_config)
            self._servo_limits_fetched = True
            print("✅ 舵机限位已从 Server 同步（未连接硬件）")
            return True
        except Exception as e:
            print(f"❌ fetch_servo_limits 失败: {e}")
            return False

    def connect(self, force_scan: bool = False) -> bool:
        print(f"开始连接机器人...：{self.is_connected} (force_scan={force_scan})")
        if self.is_connected:
            print("机器人接口已连接")
            return True

        try:
            print("正在连接机器人...")

            # ✅ 第一步：从 Server 获取舵机配置（扁平结构，无 bus/port）
            from src.comm.api.client import ServerAPIClient
            api_client = ServerAPIClient()
            servo_config = api_client.get_servo_ids_config()

            if not servo_config:
                print("❌ 未能从 Server 获取舵机配置")
                return False

            # 保存配置
            self.set_servo_ids_config(servo_config)
            print("✅ 舵机配置已从 Server 同步")

            # 构建 ServoConfigManager（供 ActuatorRouter 连接时使用 brand/motor_type）
            from src.config.servo_config_manager import ServoConfigManager
            self.servo_config_manager = ServoConfigManager(servo_config)
            
            # 注入 direction_map、offset_map、motor_type_overrides 到 motor_controller。
            # ⚠️ 关键：motor_controller 是用无参 ActuatorController() 创建的，_motor_type_overrides
            # 默认为空 {}。若不在此注入，连接真机时 _get_or_create_driver 建 RobStride 驱动
            # 拿不到 brand 型号 → 全部回退写死的 DEFAULT_MOTOR_TYPE_MAP（id2/3=RS06...），
            # 与 servo_ids.yaml 的 robstride_04 等真实型号冲突 → MOTOR_PARAMS 错位（Kp/Kd/
            # 力矩/速度上限全错）→ 猛冲/超限掉使能 → 总线错误 → BUS-OFF → Network is down。
            # 这正是"电机型号不同参数不同"导致掉线的根因，必须在此把 brand 解析出的 override 灌进去。
            self.motor_controller._direction_map = self.servo_config_manager.build_direction_map()
            self.motor_controller._offset_map = self.servo_config_manager.build_offset_map()
            self.motor_controller._motor_type_overrides = self.servo_config_manager.build_motor_type_overrides()
            
            # ✅ 第二步：从扁平配置中收集所有舵机 ID
            all_ids = set()
            part_names = ['left_arm', 'right_arm', 'base', 'lift_axis', 'neck', 'waist']  # Server 会将 body_joints 重命名为 neck
            for part_name in part_names:
                part_config = servo_config.get(part_name, {})
                if isinstance(part_config, dict):
                    for joint_info in part_config.values():
                        if isinstance(joint_info, dict) and 'id' in joint_info:
                            all_ids.add(joint_info['id'])
            print(f"🔍 配置中共 {len(all_ids)} 个舵机 ID: {sorted(all_ids)}")
            
            # ✅ 第三步：自动扫描端口发现各 ID 所在的物理端口
            id_to_port = self.motor_controller.discover_ports_by_ids(all_ids)
            if not id_to_port:
                print("❌ 端口自动发现失败——未在任何端口找到配置的舵机")
                return False
            print(f"🔍 端口发现结果: {len(id_to_port)}/{len(all_ids)} 个舵机在线, {len(set(id_to_port.values()))} 个端口")
            print(f"🟢 在线舵机 ID: {sorted(id_to_port.keys())}")
            self.online_servos = dict(id_to_port)  # 保存在线舵机列表供状态推送
            
            # ✅ 第四步：按部位分组端口
            self.servo_ports = {}
            for part_name in part_names:
                part_config = servo_config.get(part_name, {})
                if not isinstance(part_config, dict):
                    continue
                for joint_info in part_config.values():
                    if not isinstance(joint_info, dict):
                        continue
                    sid = joint_info.get('id')
                    if sid and sid in id_to_port:
                        self.servo_ports[part_name] = id_to_port[sid]
                        print(f"  📌 {part_name} → {id_to_port[sid]}")
                        break  # 找到该部位第一个在线舵机即可确定端口
            
            # 收集所有需要连接的独立端口
            unique_ports = set(self.servo_ports.values())
            print(f"🔌 共需连接 {len(unique_ports)} 个端口: {unique_ports}")
            
            # ✅ 第五步：复用 ActuatorController 已创建的驱动（_async_discover_ports_by_ids 已内置端口类型判断）
            port_drivers = {}  # port → driver
            
            for port in unique_ports:
                driver = self.motor_controller._get_or_create_driver(port)
                if driver:
                    port_drivers[port] = driver
                    print(f"✅ 端口驱动就绪: {port}")
                else:
                    print(f"❌ 端口驱动创建失败: {port}")
            
            if not port_drivers:
                print("❌ 无任何端口连接成功")
                return False
            
            # ✅ 第六步：按部位分配驱动
            left_port = self.servo_ports.get('left_arm')
            right_port = self.servo_ports.get('right_arm')
            base_port = self.servo_ports.get('base') or self.servo_ports.get('lift_axis') or self.servo_ports.get('neck')
            
            if left_port and left_port in port_drivers:
                self.left_robot = port_drivers[left_port]
                self.left_arm_connected = True
                print(f"✅ 左臂已分配: {left_port}")
            else:
                print("❌ 左臂端口未发现或连接失败")
                self.left_arm_connected = False
            
            if right_port and right_port in port_drivers:
                self.right_robot = port_drivers[right_port]
                self.right_arm_connected = True
                print(f"✅ 右臂已分配: {right_port}")
            else:
                print("❌ 右臂端口未发现或连接失败")
                self.right_arm_connected = False
            
            if base_port and base_port in port_drivers:
                self.base_robot = port_drivers[base_port]
                self.base_connected = True
                self.lift_connected = True  # 底盘、升降轴、脖子共用同一端口
                print(f"✅ 底盘/升降轴/身体关节已分配: {base_port}")
                
                # 读取初始升降轴高度
                lift_config = servo_config.get('lift_axis', {})
                for lift_info in lift_config.values():
                    if isinstance(lift_info, dict) and 'id' in lift_info:
                        lift_servo_id = lift_info['id']
                        position = self.base_robot.get_position(lift_servo_id)
                        if position is not None:
                            self.lift_height_mm = int((position / 4095.0) * 1000)
                            print(f"✅ 升降轴初始高度: {self.lift_height_mm}mm")
                        break
            else:
                print("⚠️ 底盘/升降轴端口未发现，跳过底盘连接")
                self.base_connected = False
                self.lift_connected = False

            # 至少一个组件连接成功即标记为已连接
            self.is_connected = (
                self.left_arm_connected or 
                self.right_arm_connected or 
                self.base_connected
            )

            if self.is_connected:
                # ✅ 第七步：初始化底层驱动（ActuatorController）
                # 将所有已连接的端口注册到 motor_controller
                for port, driver in port_drivers.items():
                    self.motor_controller.bind_joint_driver(port, driver)
                print(f"✅ 底层驱动已初始化: {list(port_drivers.keys())}")

                # ✅ 第八步：读取电机当前实际位置作为 IK 起点
                self._read_initial_state()

                # ✅ 第九步：检测多圈丢失（断电可能丢圈数）
                self.detect_multiturn_loss(port_drivers)

                print(f"🤖 机器人接口已连接: 左臂={self.left_arm_connected}, 右臂={self.right_arm_connected}, 底盘={self.base_connected}")
            else:
                print("❌ 无法连接任何机械臂")

            return self.is_connected

        except Exception as e:
            print(f"❌ 机器人连接异常: {e}")
            import traceback
            traceback.print_exc()
            self.is_connected = False
            return False

    def _servo_angle(self, driver, servo_id, brand):
        """读取单个舵机角度 (°)。

        - Feetech: get_position 返回步进值(0~4095)，需换算为角度(-180~180)
        - RobStride: get_position 直接返回角度制
        读取失败 (返回 None) 时返回 None，由调用方回退到安全位。
        """
        pos = driver.get_position(servo_id)
        if pos is None:
            return None
        if 'robstride' in (brand or '').lower():
            return float(pos)
        # feetech: 步进值(0~4095) → 逻辑角度(°)，应用 direction/offset 对称变换（与 move_to_angle 一致）
        if hasattr(driver, 'step_to_angle'):
            return driver.step_to_angle(servo_id, pos)
        return (pos / 4095.0) * 360.0 - 180.0

    def _read_joint_angle(self, part_label, joint_name, joint_info, fallback_driver):
        """读取单个关节角度，按该舵机实际所在端口选驱动。

        手臂是混合总线：肩/肘等大关节走 CAN(robstride)，手腕偏航/爪机走串口(feetech)。
        不能用同一个手臂驱动去读所有关节——否则会在 CAN 总线上读飞特 ID，
        触发 MECH_POS 重试失败刷屏。这里按 online_servos({id:port}) 路由到正确驱动。

        返回值约定：
        - 正常读到 → 浮点角度(°)
        - 读不到 / 未在线 / 无驱动 → 返回 np.nan（表示“未知”）。调用方必须把 nan 当成
          “不要对该电机发令”的信号：电机保持原地，绝不能当成 0.0 去命令，否则电机会从
          真实位置猛地砸向 0°（乱动）。
        """
        servo_id = joint_info.get('id')
        if not servo_id:
            return np.nan
        brand = joint_info.get('brand') or ''
        # 该舵机发现时所在的物理端口（连接阶段 discover_ports_by_ids 的结果）
        port = self.online_servos.get(servo_id) if isinstance(self.online_servos, dict) else None
        if not port:
            # 未在线（如未接入的飞特关节）：未知，保持不动，不去错误总线上重试
            return np.nan
        driver = self.motor_controller._get_or_create_driver(port)
        if driver is None:
            return np.nan
        if not brand:
            brand = 'robstride' if 'can' in port.lower() else 'feetech'
        angle = self._servo_angle(driver, servo_id, brand)
        if angle is None:
            print(f"  ⚠️ {part_label} {joint_name}(ID={servo_id}) 读取失败，保持不动(不命令)")
            return np.nan
        return angle

    def _read_initial_state(self):
        """从机器人读取当前关节角度（按品牌换算）；读不到时保留原有值避免 NaN 破坏仿真。
        
        安全保证：离线舵机不在 online_servos 中，硬件发令时 _group_by_port 会过滤掉，
        此处保留的 0.0 不会变成电机命令目标。
        """
        try:
            # 左臂
            if self.left_robot and self.left_arm_connected:
                left_arm_config = self.servo_ids.get('left_arm', {})
                angles = []

                for idx, (joint_name, joint_info) in enumerate(left_arm_config.items()):
                    angles.append(self._read_joint_angle('左臂', joint_name, joint_info, self.left_robot))

                if len(angles) == NUM_JOINTS:
                    angles_arr = np.array(angles)
                    nan_mask = np.isnan(angles_arr)
                    if nan_mask.any():
                        existing = self.left_arm_angles.copy()
                        angles_arr[nan_mask] = existing[nan_mask]
                    self.left_arm_angles = angles_arr
                    print(f"📡 左臂当前角度: {self.left_arm_angles.round(1)}")
                else:
                    print(f"⚠️ 左臂舵机数量不匹配: {len(angles)} != {NUM_JOINTS}")

            # 右臂
            if self.right_robot and self.right_arm_connected:
                right_arm_config = self.servo_ids.get('right_arm', {})
                angles = []

                for idx, (joint_name, joint_info) in enumerate(right_arm_config.items()):
                    angles.append(self._read_joint_angle('右臂', joint_name, joint_info, self.right_robot))

                if len(angles) == NUM_JOINTS:
                    angles_arr = np.array(angles)
                    nan_mask = np.isnan(angles_arr)
                    if nan_mask.any():
                        existing = self.right_arm_angles.copy()
                        angles_arr[nan_mask] = existing[nan_mask]
                    self.right_arm_angles = angles_arr
                    print(f"📡 右臂当前角度: {self.right_arm_angles.round(1)}")
                else:
                    print(f"⚠️ 右臂舵机数量不匹配: {len(angles)} != {NUM_JOINTS}")

        except Exception as e:
            print(f"❌ 读取初始状态错误: {e}")
            import traceback
            traceback.print_exc()

    # ── 多圈丢失检测 & 自动标零 ──────────────────────────────────

    def detect_multiturn_loss(self, port_drivers: dict) -> list:
        """检测断电后多圈编码器丢失的电机。

        仅 RobStride 电机有此问题（Feetech 不用多圈编码器）。
        读取每个电机的逻辑角度，对比配置的 min_angle / max_angle。
        超出限位的尝试 ±360°×N 解绕，找到有效值的电机标记为掉圈。

        Args:
            port_drivers: {port_name: driver_instance} 端口→驱动映射

        Returns:
            list: [{id, joint_name, part, raw_angle, corrected_angle}]
        """
        lost = []
        if not self.servo_config_manager:
            return lost

        for sid, info in self.servo_config_manager._id_map.items():
            # 按电机自身配置决定是否跳过多圈检测（底盘/升降/身体等非关节电机标 skip_check）
            if info.get("skip_check"):
                continue
            min_a = info.get("min_angle")
            max_a = info.get("max_angle")
            if min_a is None or max_a is None:
                continue
            brand = (info.get("brand", "") or "").lower()
            if "feetech" in brand:
                continue  # Feetech 无多圈编码器

            # 找到该电机的驱动
            part = info.get("part", "")
            port = self.servo_ports.get(part)
            driver = port_drivers.get(port) if port else None
            if not driver:
                continue

            angle = self._servo_angle(driver, sid, brand)
            if angle is None:
                continue

            # 角度在限位内 → 未掉圈
            if min_a <= angle <= max_a:
                continue

            # 自动解绕：±5圈内找落入限位区间的最近候选值
            best = None
            best_dist = float("inf")
            mid = (min_a + max_a) / 2.0
            for n in range(-5, 6):
                candidate = angle + n * 360.0
                if min_a <= candidate <= max_a:
                    dist = abs(candidate - mid)
                    if dist < best_dist:
                        best = candidate
                        best_dist = dist

            if best is not None:
                # 把解绕后的真实角度写回当前角度数组，使软件认知与实际物理位置一致。
                # 否则后续发令会从“偏差 360°×N 的读数”插值，电机会整圈狂转（灵足典型乱动）。
                # 下标 arm_index 由 servo_config_manager 在构建 _id_map 时按 part 内遍历顺序记录，
                # 与 _read_initial_state 遍历 servo_ids[part] 的顺序一致，是唯一下标来源。
                arr = self.left_arm_angles if part == "left_arm" else (
                    self.right_arm_angles if part == "right_arm" else None)
                jidx = info.get("arm_index")
                if arr is not None and isinstance(jidx, int) and 0 <= jidx < len(arr):
                    arr[jidx] = best
                lost.append({
                    "id": sid,
                    "joint_name": info.get("joint_name", str(sid)),
                    "part": part,
                    "raw_angle": round(angle, 2),
                    "corrected_angle": round(best, 2),
                })
                print(f"  ⚠️ 掉圈电机 ID={sid} {info.get('joint_name', '')}: "
                      f"读数={angle:.1f}° → 实际≈{best:.1f}°（已修正当前角度）")

        self.lost_multiturn_motors = lost
        if lost:
            print(f"🔔 检测到 {len(lost)} 个电机多圈丢失，需重新标零")
        else:
            print("✅ 所有电机多圈编码器正常")
        return lost

    def recalibrate_lost_motors(self, port_drivers: dict) -> dict:
        """对掉圈电机自动移到最近零位并标零。

        流程:
        1. 使用 CSP 将电机移到最近基点 (0° 或 360°)
        2. 调用 set_zero_position 标零 + 保存 Flash
        3. 重新检测确认

        Returns:
            dict: {success: [id,...], failed: [{id, reason},...]}
        """
        import math
        results = {"success": [], "failed": []}

        if not self.lost_multiturn_motors:
            return results

        print(f"🔧 开始重新标零 {len(self.lost_multiturn_motors)} 个电机...")

        for motor in self.lost_multiturn_motors:
            sid = motor["id"]
            part = motor["part"]
            joint_name = motor["joint_name"]

            port = self.servo_ports.get(part)
            driver = port_drivers.get(port) if port else None
            if not driver:
                results["failed"].append({"id": sid, "reason": "no driver"})
                continue

            brand = self.servo_config_manager.get_brand(sid)
            raw = self._servo_angle(driver, sid, brand)
            if raw is None:
                results["failed"].append({"id": sid, "reason": "read failed"})
                continue

            # 计算到最近零位基点：在电机「连续多圈坐标」里取最近的 360 整数倍，
            # 而非归一化 0/360 窗口。否则掉圈电机读数 -340.63° 时发 goto 0° 会正转 +340°
            # 狂转一圈（终点虽对，路径错）。改为 raw - norm（norm<=180 向下取整到
            # 最近倍，norm>180 向上），保证走最短弧。
            norm = raw % 360.0
            target_deg = raw - norm if norm <= 180.0 else raw - norm + 360.0
            print(f"  📍 ID={sid} {joint_name}: 当前{raw:.1f}° → 目标{target_deg:.0f}°")

            # 移动到目标零位，并轮询确认电机「真的到达」目标后再标零，
            # 避免仅 sleep(0.5) 就设零导致电机未到位、零位偏差。
            move_fn = None
            if hasattr(driver, "move_one_joint_csp"):
                move_fn = lambda: driver.move_one_joint_csp(sid, math.radians(target_deg))
            elif hasattr(driver, "move_joint_csp"):
                move_fn = lambda: driver.move_joint_csp(sid, target_deg)

            reached = False
            cur = None
            deadline = time.time() + 12.0  # 最多等 12s
            while time.time() < deadline:
                # 重复下发目标（CSP 单次帧可能丢失），电机内部位置环持续跟踪 LOC_REF
                if move_fn:
                    try:
                        move_fn()
                    except Exception as e:
                        print(f"  ⚠️ ID={sid} 运动指令重发异常: {e}")
                cur = self._servo_angle(driver, sid, brand)
                if cur is not None and abs(cur - target_deg) <= 1.5:
                    reached = True
                    break
                time.sleep(0.1)

            if not reached:
                results["failed"].append({"id": sid, "reason": "timeout not reached target"})
                print(f"  ❌ ID={sid} {joint_name}: 超时未到达目标 {target_deg:.1f}°"
                      f"（当前 {cur if cur is not None else 'N/A'}°），跳过标零以免零位错误")
                continue

            print(f"  ✅ ID={sid} {joint_name}: 已到达目标 {target_deg:.1f}°"
                  f"（当前 {cur:.1f}°），准备标零")
            time.sleep(0.2)  # 到位后稍作稳定

            # 标零 + 保存 Flash
            if hasattr(driver, "set_zero_position"):
                ok = driver.set_zero_position(sid)
            else:
                ok = False

            if ok:
                results["success"].append(sid)
                print(f"  ✅ ID={sid} 标零成功")
            else:
                results["failed"].append({"id": sid, "reason": "set_zero failed"})
                print(f"  ❌ ID={sid} 标零失败")

        # 重新检测确认
        self.detect_multiturn_loss(port_drivers)
        print(f"🔧 标零完成: 成功={results['success']}, 失败={results['failed']}")
        return results

    async def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict,
                         end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray,
                         joint_limits_max_deg: np.ndarray):
        """设置运动学解算器。委托给对应的适配器。"""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()

        await self.adapter.setup(self.visualizer, self.config)
        print(f"[RobotInterface] 适配器已初始化 ({self.robot_type}, {NUM_JOINTS}-DOF)")

    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """获取指定机械臂的末端位置。委托给适配器。"""
        angles = self.get_arm_angles(arm)
        return self.adapter.compute_fk(arm, angles)

    def get_end_effector_pose(self, arm: str):
        """获取指定机械臂的 TCP 位姿 (位置, 旋转矩阵)。委托给适配器。"""
        angles = self.get_arm_angles(arm)
        return self.adapter.compute_fk_pose(arm, angles)

    def solve_ik_shoulder(self, arm: str, target_world: np.ndarray,
                          shoulder_pos: np.ndarray, shoulder_rot: np.ndarray) -> np.ndarray:
        """肩部系 IK（以肩安装座为基座的 8-DOF 模型）。委托给适配器。"""
        current_angles = self.get_arm_angles(arm)
        return self.adapter.solve_ik_shoulder(arm, target_world,
                                              shoulder_pos, shoulder_rot,
                                              current_angles)

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float, wrist_yaw: float = 0.0, override_wrist: bool = True):
        """更新关节角度（含限位钳制）。委托给适配器。"""
        self.adapter.update_arm_angles(arm, ik_angles, wrist_flex, wrist_roll, gripper, wrist_yaw, override_wrist)

    def engage(self) -> bool:
        """使能机器人电机(开始发送指令)。

        - 清空 RobStride 驱动的初始化标记，确保因过流/看门狗等原因物理失能的电机能重新使能。
        - 显式逐电机使能所有在线舵机（按连接时动态发现的真实端口，带重试），
          而非依赖逐帧懒使能：懒使能下某电机首次 enable 没拿到 ACK 会一直保持未初始化、
          每帧被跳过而"松"着，且无任何提示。
        """
        print("🔌 使能机器人电机(开始发送指令)")
        if not self.is_connected:
            print("无法使能机器人: 未连接")
            return False

        # 清空初始化标记 → 下方显式使能会重新初始化每个电机
        if self.motor_controller and hasattr(self.motor_controller, "force_reinitialize_all_robstride"):
            self.motor_controller.force_reinitialize_all_robstride()

        # 显式使能所有在线电机（按真实端口，带重试），并打印成败
        self._enable_all_online()

        self.is_engaged = True
        print("🔌 机器人电机已使能 - 将发送指令")
        return True

    def _enable_all_online(self) -> None:
        """按连接时动态发现的真实端口，显式使能所有在线舵机（RobStride 需要）。

        - 飞特(串口)上电即保持力矩，无需显式使能，其驱动无 _ensure_ready，跳过。
        - 使能带 3 次重试（CAN 偶发卡顿可恢复）。
        - 仍失败的电机明确打印，便于定位"松/无力矩"的电机（多为该电机 CAN 无反馈或硬件故障）。
        """
        if not self.online_servos or not self.motor_controller:
            return
        failed = []
        for sid, port in self.online_servos.items():
            driver = self.motor_controller._get_or_create_driver(port)
            if not driver or not hasattr(driver, "_ensure_ready"):
                continue  # 无需显式使能的驱动（如飞特）跳过
            ok = False
            for _ in range(3):
                if driver._ensure_ready(sid):
                    ok = True
                    break
                time.sleep(0.05)
            if not ok:
                failed.append(sid)
        if failed:
            print(f"⚠️ 以下电机使能失败（将无力矩/松，可能 CAN 无反馈或硬件故障）: {sorted(failed)}")
        else:
            print(f"✅ 全部 {len(self.online_servos)} 个在线电机使能成功")

    def list_poses(self) -> Dict:
        """返回可用的姿态预设列表，供前端动作列表使用。"""
        from src.config.settings import get_robot_poses
        poses = get_robot_poses()
        result = {}
        for name, data in poses.items():
            result[name] = {
                "left": data.get("left", []),
                "right": data.get("right", []),
                "body": data.get("body", {}),
            }
        return result

    async def disengage(self) -> bool:
        """回到初始安全位置后禁能电机力矩。"""
        if not self.is_connected:
            print("机器人已断开")
            return True

        try:
            from src.controller.pose_controller import return_to_initial_position
            await return_to_initial_position(ri=self)
            self.disable_torque()
            self.is_engaged = False
            print("✅ 机器人已回到安全位置并禁能")
            return True

        except Exception as e:
            print(f"断开过程错误: {e}")
            return False

    async def disconnect(self) -> None:
        """断开所有底层电机驱动并重置连接状态。

        供控制循环停止 / 系统软重启时释放 CAN/串口资源，
        避免旧 driver 仍持有端口导致重建后无法重新连接。

        底层委托给 ActuatorController.cleanup()（异步关闭所有 driver）。
        """
        try:
            if self.motor_controller and hasattr(self.motor_controller, "cleanup"):
                await self.motor_controller.cleanup()
        except Exception as e:
            print(f"清理电机控制器时出错: {e}")
        self.is_connected = False
        self.is_engaged = False


    async def send_command(self) -> bool:
        """使用字典格式向机器人发送当前关节角度，并更新仿真。
        
        关键优化：硬件写入以 fire-and-forget 方式提交到后台线程，
        不阻塞控制循环。仿真始终以全速（50Hz）运行。
        """
        current_time = time.time()

        # 检查时间间隔（真机和仿真共用）
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # 未到发送时间

        # ✅ 立即更新时间戳，保证控制循环不受硬件延迟影响
        self.last_send_time = current_time

        # 1. 发送到真机（如果连接且使能）—— fire-and-forget，不阻塞事件循环
        #    每次递增版本号，旧任务拿锁后发现版本过期直接跳过
        success = True
        if self.is_connected and self.is_engaged:
            try:
                self._hw_version += 1
                asyncio.ensure_future(self._send_to_hardware(self._hw_version))
            except Exception as e:
                print(f"发送机器人指令错误: {e}")
                self.general_errors += 1
                if self.general_errors > self.max_general_errors:
                    self.is_connected = False
                    print("❌ 机器人接口因重复错误而断开")
                success = False

        # 2. 更新仿真（无论真机是否连接，始终全速运行）
        if self.visualizer:
            self._update_simulation()

        return success

    def set_gripper(self, arm: str, closed: bool, trigger_value: Optional[float] = None):
        """设置指定机械臂的夹爪状态(键盘 C/. 或 VR 扳机)。

        委托 adapter 映射为 arm8 夹爪角度并钳制到软限位；
        trigger_value 为 None 时按开/关布尔映射为 0/1（与 VR 扳机行为一致）。
        """
        if self.adapter is None:
            return
        if trigger_value is not None:
            self.adapter.apply_gripper_from_trigger(arm, trigger_value)
        else:
            self.adapter.apply_gripper_from_trigger(arm, 1.0 if closed else 0.0)

    def get_arm_angles(self, arm: str) -> np.ndarray:
        """获取指定机械臂的当前关节角度。"""
        if arm == "left":
            angles = self.left_arm_angles.copy()
        elif arm == "right":
            angles = self.right_arm_angles.copy()
        else:
            raise ValueError(f"无效的机械臂: {arm}")

        return angles

    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """获取指定机械臂的当前关节角度，用于 PyBullet 可视化。"""
        # 返回原始角度，不进行任何修正以便正确诊断
        return self.get_arm_angles(arm)

    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """从机器人硬件获取实际关节角度(非指令角度)。"""
        try:
            if arm == "left" and self.left_robot and self.left_arm_connected:
                observation = self.left_robot.get_observation()
                if observation:
                    return np.array([
                        observation['shoulder_pan.pos'],
                        observation['shoulder_lift.pos'],
                        observation['elbow_flex.pos'],
                        observation['wrist_flex.pos'],
                        observation['wrist_roll.pos'],
                        observation['gripper.pos']
                    ])
            elif arm == "right" and self.right_robot and self.right_arm_connected:
                observation = self.right_robot.get_observation()
                if observation:
                    return np.array([
                        observation['shoulder_pan.pos'],
                        observation['shoulder_lift.pos'],
                        observation['elbow_flex.pos'],
                        observation['wrist_flex.pos'],
                        observation['wrist_roll.pos'],
                        observation['gripper.pos']
                    ])
        except Exception as e:
            print(f"读取 {arm} 实际关节角度错误: {e}")

        # 如果无法读取实际角度，回退到指令角度
        return self.get_arm_angles(arm)

    def disable_torque(self, arm: str = None):
        """禁能机器人关节力矩。

        Args:
            arm: 'left', 'right', 或 None 表示两个机械臂
        """
        if not self.is_connected:
            return

        try:
            # ST3215Driver 直接提供 set_torque 方法
            if arm is None or arm == "left":
                if self.left_robot and self.left_arm_connected:
                    print("正在禁能左臂力矩...")
                    left_arm_config = self.servo_ids.get('left_arm', {})
                    for joint_name, joint_info in left_arm_config.items():
                        servo_id = joint_info.get('id')
                        if servo_id:
                            self.left_robot.set_torque(servo_id, False)

            if arm is None or arm == "right":
                if self.right_robot and self.right_arm_connected:
                    print("正在禁能右臂力矩...")
                    right_arm_config = self.servo_ids.get('right_arm', {})
                    for joint_name, joint_info in right_arm_config.items():
                        servo_id = joint_info.get('id')
                        if servo_id:
                            self.right_robot.set_torque(servo_id, False)

        except Exception as e:
            print(f"禁能力矩错误: {e}")

    def build_robot_action(self) -> dict:
        """[兼容] 构建机器人动作字典。保留供外部调用，真机发送请用 _send_to_hardware。"""
        return self.adapter.build_action(
            vr_raw_data=self.vr_raw_data,
            base_vel=self.base_velocity_target,
            lift_vel=self.lift_velocity,
        )

    async def _send_to_hardware(self, version: int = 0):
        """发送指令到真机硬件（异步，串口 I/O 在独立线程执行，不阻塞事件循环）。
        
        用 asyncio.Lock 保证同一时刻只有一个任务在执行串口写入，
        版本号机制确保锁定释放后只有最新任务才真正写入硬件。
        """
        if not self.motor_controller:
            return
        
        # 硬件写入锁：只允许一个任务同时操作串口
        async with self._hw_lock:
            # 版本过期则跳过——说明有更新的任务在排队
            if version < self._hw_version:
                return
            # 先同步状态到 adapter（确保 keyboard 设置的 base_velocity_target 生效）
            self._sync_to_adapter()
            
            actions = self.adapter.build_hardware_actions(
                self.servo_ids, self.servo_ports, self.online_servos)

            pos_cmds = actions.get("position_commands", [])
            spd_cmds = actions.get("speed_commands", [])

            loop = asyncio.get_event_loop()
            executor = self.motor_controller._executor
            _HW_TIMEOUT = 1.0  # 单个串口操作超时（秒），超时则跳过该端口

            # 派发位置命令（双臂关节角度）—— 在独立线程中执行串口写入
            for cmd in pos_cmds:
                if not cmd.get("targets"):
                    continue
                port = cmd["port"]
                targets = cmd["targets"]
                # 用线程执行，避免 CAN 掉线时 setup_can 的阻塞 subprocess 卡死事件循环，
                # 导致 WebSocket 心跳超时掉线（服务端 WS_PING_TIMEOUT=10 会据此 close(1011)）
                driver = await asyncio.to_thread(self.motor_controller._get_or_create_driver, port)
                if driver and hasattr(driver, 'sync_write_positions'):
                    try:
                        await asyncio.wait_for(
                            loop.run_in_executor(
                                executor,
                                driver.sync_write_positions,
                                targets, 0
                            ),
                            timeout=_HW_TIMEOUT,
                        )
                    except asyncio.TimeoutError:
                        print(f"[HW] position port={port} 超时，跳过")
                    except Exception as e:
                        print(f"[HW] position port={port} 异常: {e}")

            # 派发速度命令（底盘轮子 + 升降轴）
            for cmd in spd_cmds:
                if not cmd.get("targets"):
                    continue
                port = cmd["port"]
                # 确保所有目标电机处于速度模式（一次性的 EEPROM 写）
                for servo_id in cmd["targets"]:
                    try:
                        await asyncio.wait_for(
                            self._ensure_speed_mode_async(port, servo_id),
                            timeout=_HW_TIMEOUT,
                        )
                    except asyncio.TimeoutError:
                        print(f"[HW] speed_mode port={port} id={servo_id} 超时，跳过")
                # 批量写入速度
                # 同上：避免 CAN 重连阻塞事件循环（事件循环卡死会导致 WS 心跳超时掉线）
                driver = await asyncio.to_thread(self.motor_controller._get_or_create_driver, port)
                if driver and hasattr(driver, 'sync_write_spec_batch'):
                    try:
                        await asyncio.wait_for(
                            loop.run_in_executor(
                                executor,
                                driver.sync_write_spec_batch,
                                cmd["targets"]
                            ),
                            timeout=_HW_TIMEOUT,
                        )
                    except asyncio.TimeoutError:
                        print(f"[HW] speed port={port} 超时，跳过")
                    except Exception as e:
                        print(f"[HW] speed port={port} 异常: {e}")

            # 升降轴：rad/s 直接下发，绕过 Feetech 0~1023 映射
            lift_rads = actions.get("lift_velocity_rads", {})
            if lift_rads:
                for lid, rads in lift_rads.items():
                    port = self.online_servos.get(lid)
                    if not port:
                        continue
                    # 同上：避免 CAN 重连阻塞事件循环
                    driver = await asyncio.to_thread(self.motor_controller._get_or_create_driver, port)
                    if driver and hasattr(driver, '_send_velocity_rad'):
                        try:
                            await asyncio.wait_for(
                                self._ensure_speed_mode_async(port, lid),
                                timeout=_HW_TIMEOUT,
                            )
                            await asyncio.wait_for(
                                loop.run_in_executor(
                                    executor,
                                    driver._send_velocity_rad,
                                    lid, rads
                                ),
                                timeout=_HW_TIMEOUT,
                            )
                        except asyncio.TimeoutError:
                            pass
                        except Exception:
                            pass

    def _ensure_speed_mode(self, port, servo_id):
        """确保指定舵机处于速度模式（同一 port+servo 只切换一次，同步版）。"""
        key = f"_speed_mode_{port}_{servo_id}"
        if not hasattr(self, key):
            self.motor_controller.set_servo_velocity_mode(port, servo_id)
            setattr(self, key, True)

    async def _ensure_speed_mode_async(self, port, servo_id):
        """确保指定舵机处于速度模式（异步版，模式切换在独立线程执行）。"""
        key = f"_speed_mode_{port}_{servo_id}"
        if not hasattr(self, key):
            loop = asyncio.get_event_loop()
            # 同上：避免 CAN 重连阻塞事件循环
            driver = await asyncio.to_thread(self.motor_controller._get_or_create_driver, port)
            if driver and hasattr(driver, 'set_velocity_mode'):
                await loop.run_in_executor(
                    self.motor_controller._executor,
                    driver.set_velocity_mode,
                    servo_id
                )
            else:
                self.motor_controller.set_servo_velocity_mode(port, servo_id)
            setattr(self, key, True)

    def _update_simulation(self):
        """更新仿真可视化。委托给适配器。"""
        if not self.visualizer:
            return

        # ---- 诊断: 首次进入打印 ----
        if not hasattr(self, '_sim_diag_printed'):
            self._sim_diag_printed = True
            print(f"[DIAG] _update_simulation 首次进入 | "
                  f"adapter.is_setup={self.adapter.is_setup} | "
                  f"left_angles={self.adapter.left_angles.round(1)} | "
                  f"right_angles={self.adapter.right_angles.round(1)} | "
                  f"base_vel=({self.base_velocity_target['x']:.3f}, "
                  f"{self.base_velocity_target['y']:.3f}, "
                  f"{self.base_velocity_target['theta']:.3f})")

        # 1. 同步状态到 adapter
        self._sync_to_adapter()

        # 2. 委托 adapter 更新可视化
        state = {
            "dt": self.config.send_interval,
            "vr_raw_data": self.vr_raw_data,
        }
        self.adapter.update_visualization(self.visualizer, state)

        # 3. 从 adapter 同步状态回去
        self._sync_from_adapter()

        # 4. 更新标记点
        self._update_markers()

        # 5. 推进仿真
        self.visualizer.step_simulation()

    def _sync_to_adapter(self):
        """同步 robot_interface 状态 → adapter。"""
        self.adapter.set_base_velocity(
            self.base_velocity_target["x"],
            self.base_velocity_target["y"],
            self.base_velocity_target["theta"],
        )
        self.adapter.set_lift_velocity(self.lift_velocity)

    def _sync_from_adapter(self):
        """同步 adapter 状态 → robot_interface。"""
        self.lift_height_mm = self.adapter.lift_height_mm

    def _update_markers(self):
        """更新目标/位姿标记点。

        与 VR / 键盘一致：仅在该臂处于 POSITION_CONTROL（握把/控制已激活）时绘制，
        否则隐藏。当前点（红/蓝）= 实时 FK；目标点（绿/黄）= arm_state.target_position。
        任何输入源（VR/键盘/AI 绝对 TCP）只要走 control_loop 的 _execute_goal 设了
        arm_state.target_position，marker 就会自动显示，无需各自特殊化。
        """
        if not self.visualizer:
            return

        from src.inputs.base import ControlMode

        for arm in ["left", "right"]:
            arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
            if arm_state and arm_state.mode == ControlMode.POSITION_CONTROL:
                if arm_state.target_position is not None:
                    current_pos = self.get_current_end_effector_position(arm)
                    self.visualizer.update_marker_position(f"{arm}_target", current_pos)
                    self.visualizer.update_coordinate_frame(f"{arm}_target_frame", current_pos)
                if arm_state.goal_position is not None:
                    self.visualizer.update_marker_position(f"{arm}_goal", arm_state.goal_position)
                    self.visualizer.update_coordinate_frame(f"{arm}_goal_frame", arm_state.goal_position)
            else:
                self.visualizer.hide_marker(f"{arm}_target")
                self.visualizer.hide_marker(f"{arm}_goal")
                self.visualizer.hide_frame(f"{arm}_target_frame")
                self.visualizer.hide_frame(f"{arm}_goal_frame")