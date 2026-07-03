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

        # 错误跟踪
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3
        self.max_general_errors = 8

        # 安全关机初始位置 (由适配器类型决定)
        from aiderminal.config.settings import _ROBOT_TYPE_CONFIGS
        rt_cfg = _ROBOT_TYPE_CONFIGS.get(self.robot_type, _ROBOT_TYPE_CONFIGS["aider"])
        self.initial_left_arm = np.array(rt_cfg["initial_left_arm"])
        self.initial_right_arm = np.array(rt_cfg["initial_right_arm"])

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



    def connect(self) -> bool:
        print(f"开始连接机器人...：{self.is_connected}")
        if self.is_connected:
            print("机器人接口已连接")
            return True

        if not self.config.enable_robot:
            print("⚠️ 配置中禁用了机器人接口，但仍可连接到仿真")
            print("💡 如需连接真机，请确保启动时未使用 --no-robot 参数")
            # 即使禁用真机，也标记为已连接（用于仿真）
            self.is_connected = True
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

                # ✅ 第八步：读取初始关节状态
                self._read_initial_state()
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

    def _read_initial_state(self):
        """从机器人读取初始关节状态（通过 ST3215Driver）。"""
        try:
            # 左臂
            if self.left_robot and self.left_arm_connected:
                left_arm_config = self.servo_ids.get('left_arm', {})
                angles = []
                
                for joint_name, joint_info in left_arm_config.items():
                    servo_id = joint_info.get('id')
                    if servo_id:
                        position = self.left_robot.get_position(servo_id)
                        if position is not None:
                            angle = (position / 4095.0) * 360.0 - 180.0
                            angles.append(angle)
                        else:
                            angles.append(0.0)
                    else:
                        angles.append(0.0)
                
                if len(angles) == NUM_JOINTS:
                    self.left_arm_angles = np.array(angles)
                    print(f"✅ 左臂初始状态: {self.left_arm_angles.round(1)}")
                else:
                    print(f"⚠️ 左臂舵机数量不匹配: {len(angles)} != {NUM_JOINTS}")
            
            # 右臂
            if self.right_robot and self.right_arm_connected:
                right_arm_config = self.servo_ids.get('right_arm', {})
                angles = []
                
                for joint_name, joint_info in right_arm_config.items():
                    servo_id = joint_info.get('id')
                    if servo_id:
                        position = self.right_robot.get_position(servo_id)
                        if position is not None:
                            angle = (position / 4095.0) * 360.0 - 180.0
                            angles.append(angle)
                        else:
                            angles.append(0.0)
                    else:
                        angles.append(0.0)
                
                if len(angles) == NUM_JOINTS:
                    self.right_arm_angles = np.array(angles)
                    print(f"✅ 右臂初始状态: {self.right_arm_angles.round(1)}")
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

    def disengage(self) -> bool:
        """禁能机器人电机(停止发送指令)。"""
        if not self.is_connected:
            print("机器人已断开")
            return True

        try:
            # 禁能前返回安全位置
            self.return_to_initial_position()

            # 禁能力矩
            self.disable_torque()

            self.is_engaged = False
            print("🔌 机器人电机已禁能 - 指令停止")
            return True

        except Exception as e:
            print(f"禁能机器人错误: {e}")
            return False

    def send_command(self) -> bool:
        """使用字典格式向机器人发送当前关节角度，并更新仿真。"""
        current_time = time.time()

        # 检查时间间隔（真机和仿真共用）
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # 未到发送时间

        # 1. 发送到真机（如果连接且使能）
        success = True
        if self.is_connected and self.is_engaged:
            try:
                self._send_to_hardware()
            except Exception as e:
                print(f"发送机器人指令错误: {e}")
                self.general_errors += 1
                if self.general_errors > self.max_general_errors:
                    self.is_connected = False
                    print("❌ 机器人接口因重复错误而断开")
                success = False
        elif not hasattr(self, '_diag_hw_skip_printed'):
            self._diag_hw_skip_printed = True
            print(f"[DIAG] 硬件派发跳过: is_connected={self.is_connected}, is_engaged={self.is_engaged}, "
                  f"servo_ids keys={list(self.servo_ids.keys())}, servo_ports={self.servo_ports}")

        # 2. 更新仿真（无论真机是否连接）
        if self.visualizer:
            self._update_simulation()

        # 更新时间戳
        self.last_send_time = current_time
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

    def return_to_initial_position(self):
        """将两个机械臂返回到初始位置。"""
        print("⏪ 正在将机器人返回到初始位置...")

        try:
            # 设置初始位置 - 无方向映射
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()

            # 发送几次指令以确保移动
            for i in range(10):
                self.send_command
                time.sleep(0.1)

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

    def _send_to_hardware(self):
        """发送指令到真机硬件。
        
        委托 adapter 构建结构化的硬件命令（位置 + 速度），
        本方法只负责无脑派发，不包含任何机器人特有的底盘/轮子逻辑。
        """
        if not self.motor_controller:
            return
        
        # 先同步状态到 adapter（确保 keyboard 设置的 base_velocity_target 生效）
        self._sync_to_adapter()
        
        actions = self.adapter.build_hardware_actions(self.servo_ids, self.servo_ports)

        # 一次性诊断：看看硬件命令是否生成
        if not hasattr(self, '_diag_hw_actions_printed'):
            self._diag_hw_actions_printed = True
            pos_cmds = actions.get("position_commands", [])
            spd_cmds = actions.get("speed_commands", [])
            print(f"[DIAG] build_hardware_actions: "
                  f"position_commands={len(pos_cmds)}, speed_commands={len(spd_cmds)}")
            for cmd in spd_cmds:
                print(f"  [DIAG] speed_cmd port={cmd['port']} targets={cmd['targets']}")

        # 派发位置命令（双臂关节角度）
        for cmd in actions.get("position_commands", []):
            if cmd.get("targets"):
                self.motor_controller.write_positions_sync(cmd["port"], cmd["targets"], time_ms=0)

        # 派发速度命令（底盘轮子 + 升降轴）
        # 使用批量同步写入，将 N 次串口往返压缩为 1 次广播写
        for cmd in actions.get("speed_commands", []):
            if not cmd.get("targets"):
                continue
            port = cmd["port"]
            # 确保所有目标电机处于速度模式
            for servo_id in cmd["targets"]:
                self._ensure_speed_mode(port, servo_id)
            # 批量写入：一次串口事务发送所有速度
            driver = self.motor_controller._get_or_create_driver(port)
            if driver and hasattr(driver, 'sync_write_spec_batch'):
                driver.sync_write_spec_batch(cmd["targets"])

    def _ensure_speed_mode(self, port, servo_id):
        """确保指定舵机处于速度模式（同一 port+servo 只切换一次）。"""
        key = f"_speed_mode_{port}_{servo_id}"
        if not hasattr(self, key):
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