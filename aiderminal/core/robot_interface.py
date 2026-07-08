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

from aiderminal.inputs.base import is_any_input_active
from pathlib import Path
from typing import Optional, Dict, Tuple

from aiderminal.config.settings import (
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
        
        # 底层电机控制器
        from aiderminal.controller.actuator_controller import ActuatorController
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
        self.max_general_errors = 8

        # 安全关机初始位置 (由适配器类型决定)
        from aiderminal.config.settings import get_robot_initial_arm
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
            from aiderminal.robots.aloha import AlohaAdapter
            self.adapter = AlohaAdapter()
            print("[RobotInterface] 使用 AlohaAdapter (6-DOF, SO100)")
        else:
            from aiderminal.robots.aider import AiderAdapter
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
            return True
        except Exception as e:
            print(f"❌ 解析舵机配置失败: {e}")
            return False

    def connect(self, force_scan: bool = False) -> bool:
        print(f"开始连接机器人...：{self.is_connected} (force_scan={force_scan})")
        if self.is_connected:
            print("机器人接口已连接")
            return True

        try:
            print("正在连接机器人...")

            # ✅ 第一步：从 Server 获取舵机配置（扁平结构，无 bus/port）
            from aiderminal.comm.api.client import ServerAPIClient
            api_client = ServerAPIClient()
            servo_config = api_client.get_servo_ids_config()
            
            if not servo_config:
                print("❌ 未能从 Server 获取舵机配置")
                return False
            
            # 保存配置
            self.set_servo_ids_config(servo_config)
            print("✅ 舵机配置已从 Server 同步")

            # 构建 ServoConfigManager（供 ActuatorRouter 连接时使用 brand/motor_type）
            from aiderminal.config.servo_config_manager import ServoConfigManager
            self.servo_config_manager = ServoConfigManager(servo_config)
            
            # ✅ 第二步：从扁平配置中收集所有舵机 ID
            all_ids = set()
            part_names = ['left_arm', 'right_arm', 'base', 'lift_axis', 'neck']  # Server 会将 body_joints 重命名为 neck
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

                # ✅ 第八步：连接完成（不自动读取状态，不自动移动到安全位，
                #     角度保持在适配器默认值 np.zeros。前端姿态列表由用户选择。）
                print(f"🤖 机器人接口已连接: 左臂={self.left_arm_connected}, 右臂={self.right_arm_connected}, 底盘={self.base_connected}")
                print("💡 机器人已连接但未使能，请在前端动作列表中选择姿态（安全/默认/...）")
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
        # feetech: 步进值(0~4095) → 角度(-180~180)
        return (pos / 4095.0) * 360.0 - 180.0

    def _read_initial_state(self):
        """从机器人读取当前关节角度（按品牌换算）；读不到时填 NaN，不自动回退到安全位。
        
        设计意图: 连接后不做任何自动移动。前端用户选择姿态后才发送位置指令。
        """
        nan = float('nan')
        try:
            # 左臂
            if self.left_robot and self.left_arm_connected:
                left_arm_config = self.servo_ids.get('left_arm', {})
                angles = []

                for idx, (joint_name, joint_info) in enumerate(left_arm_config.items()):
                    servo_id = joint_info.get('id')
                    brand = joint_info.get('brand') or ''
                    if not brand:
                        brand = 'robstride' if 'robstride' in type(self.left_robot).__module__.lower() else 'feetech'
                    if servo_id:
                        angle = self._servo_angle(self.left_robot, servo_id, brand)
                        if angle is None:
                            angle = nan
                            print(f"  ⚠️ 左臂 {joint_name}(ID={servo_id}) 读取失败，暂设为 NaN")
                        angles.append(angle)
                    else:
                        angles.append(nan)

                if len(angles) == NUM_JOINTS:
                    self.left_arm_angles = np.array(angles)
                    print(f"📡 左臂当前角度: {self.left_arm_angles.round(1)} (NaN=未读取)")
                else:
                    print(f"⚠️ 左臂舵机数量不匹配: {len(angles)} != {NUM_JOINTS}")

            # 右臂
            if self.right_robot and self.right_arm_connected:
                right_arm_config = self.servo_ids.get('right_arm', {})
                angles = []

                for idx, (joint_name, joint_info) in enumerate(right_arm_config.items()):
                    servo_id = joint_info.get('id')
                    brand = joint_info.get('brand') or ''
                    if not brand:
                        brand = 'robstride' if 'robstride' in type(self.right_robot).__module__.lower() else 'feetech'
                    if servo_id:
                        angle = self._servo_angle(self.right_robot, servo_id, brand)
                        if angle is None:
                            angle = nan
                            print(f"  ⚠️ 右臂 {joint_name}(ID={servo_id}) 读取失败，暂设为 NaN")
                        angles.append(angle)
                    else:
                        angles.append(nan)

                if len(angles) == NUM_JOINTS:
                    self.right_arm_angles = np.array(angles)
                    print(f"📡 右臂当前角度: {self.right_arm_angles.round(1)} (NaN=未读取)")
                else:
                    print(f"⚠️ 右臂舵机数量不匹配: {len(angles)} != {NUM_JOINTS}")

        except Exception as e:
            print(f"❌ 读取初始状态错误: {e}")
            import traceback
            traceback.print_exc()

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

    def solve_ik(self, arm: str, target_position: np.ndarray,
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """逆运动学求解。委托给适配器。"""
        current_angles = self.get_arm_angles(arm)
        return self.adapter.solve_ik(arm, target_position, current_angles)

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float, wrist_yaw: float = 0.0):
        """更新关节角度（含限位钳制）。委托给适配器。"""
        self.adapter.update_arm_angles(arm, ik_angles, wrist_flex, wrist_roll, gripper, wrist_yaw)

    def engage(self) -> bool:
        """使能机器人电机(开始发送指令)。"""
        print("🔌 使能机器人电机(开始发送指令)")
        if not self.is_connected:
            print("无法使能机器人: 未连接")
            return False

        self.is_engaged = True
        print("🔌 机器人电机已使能 - 将发送指令")
        return True

    def list_poses(self) -> Dict:
        """返回可用的姿态预设列表，供前端动作列表使用。"""
        from aiderminal.config.settings import get_robot_poses
        poses = get_robot_poses()
        result = {}
        for name, data in poses.items():
            result[name] = {
                "left": data.get("left", []),
                "right": data.get("right", []),
            }
        return result

    async def goto_pose(self, arm: str, pose_name: str) -> Dict:
        """将指定机械臂移动到命名姿态（安全/默认/...）。

        Args:
            arm: 'left', 'right', 或 'both'
            pose_name: 姿态名称，需在 POSES 字典中存在

        Returns:
            {"success": bool, "message": str}
        """
        if not self.is_connected:
            return {"success": False, "message": "机器人未连接"}

        poses = self.list_poses()
        if pose_name not in poses:
            return {"success": False, "message": f"未知姿态 '{pose_name}'，可选: {list(poses.keys())}"}

        pose = poses[pose_name]
        arms_to_move = ["left", "right"] if arm == "both" else [arm]

        for target_arm in arms_to_move:
            targets = pose.get(target_arm)
            if targets is None:
                print(f"  ⚠️ 姿态 '{pose_name}' 未定义 {target_arm} 臂角度")
                continue

            if target_arm == "left":
                self.left_arm_angles = np.array(targets, dtype=float)
                print(f"  🎯 左臂 → '{pose_name}': {self.left_arm_angles.round(1)}")
            else:
                self.right_arm_angles = np.array(targets, dtype=float)
                print(f"  🎯 右臂 → '{pose_name}': {self.right_arm_angles.round(1)}")

        # 使能并发送一次指令让舵机开始运动
        self.engage()
        # 重置 last_send_time 绕过频率限制，确保姿态指令立刻发送到硬件
        # （否则控制循环刚发完命令时 send_command 会因间隔检查跳过）
        self.last_send_time = 0
        await self.send_command()
        print(f"✅ 已发送 goto_pose 指令: arm={arm}, pose={pose_name}")
        return {"success": True, "message": f"已移动到 '{pose_name}' 姿态"}

    async def disengage(self) -> bool:
        """禁能机器人电机(停止发送指令)。"""
        if not self.is_connected:
            print("机器人已断开")
            return True

        try:
            # 禁能力矩（不回初始位置 — 部分舵机在线时硬编码初始位可能导致危险姿势）
            self.disable_torque()

            self.is_engaged = False
            print("🔌 机器人电机已禁能 - 指令停止")
            return True

        except Exception as e:
            print(f"禁能机器人错误: {e}")
            return False

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
        """设置指定机械臂的夹爪状态(仅存储 trigger_value,实际映射在 control_loop 中完成)。"""
        # 这个方法现在只是占位符,真正的线性映射在 _update_visualization() 中执行
        pass

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

    async def return_to_initial_position(self):
        """将两个机械臂返回到初始位置。"""
        print("⏪ 正在将机器人返回到初始位置...")

        try:
            # 设置初始位置 - 无方向映射
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()

            # 发送几次指令以确保移动
            for i in range(10):
                await self.send_command()
                await asyncio.sleep(0.1)

            print("✅ 机器人已返回到初始位置")
        except Exception as e:
            print(f"返回初始位置错误: {e}")

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
            
            actions = self.adapter.build_hardware_actions(self.servo_ids, self.servo_ports)

            pos_cmds = actions.get("position_commands", [])
            spd_cmds = actions.get("speed_commands", [])

            # 当 VR/键盘有输入时打印硬件命令（便于排查电机 ID 和角度）
            if is_any_input_active():
                for cmd in pos_cmds:
                    tgt_str = ",".join(f"{sid}:{ang:.1f}°" for sid, ang in cmd['targets'].items())
                    print(f"[HW] pos_cmd port={cmd['port']} targets=[{tgt_str}]")

            loop = asyncio.get_event_loop()
            executor = self.motor_controller._executor
            _HW_TIMEOUT = 1.0  # 单个串口操作超时（秒），超时则跳过该端口

            # 派发位置命令（双臂关节角度）—— 在独立线程中执行串口写入
            for cmd in pos_cmds:
                if not cmd.get("targets"):
                    continue
                port = cmd["port"]
                targets = cmd["targets"]
                driver = self.motor_controller._get_or_create_driver(port)
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
                driver = self.motor_controller._get_or_create_driver(port)
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
            driver = self.motor_controller._get_or_create_driver(port)
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
        """更新目标/位姿标记点。"""
        if not self.visualizer:
            return

        from aiderminal.inputs.base import ControlMode

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