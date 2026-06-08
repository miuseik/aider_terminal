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

from config.settings import (
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
        from controller.motor_controller import MotorController
        self.motor_controller = MotorController()

        # 各机械臂连接状态
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # 舵机 ID 配置
        self.servo_ids = {}
        
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
        from config.settings import _ROBOT_TYPE_CONFIGS
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
            from robots.aloha import AlohaAdapter
            self.adapter = AlohaAdapter()
            print("[RobotInterface] 使用 AlohaAdapter (6-DOF, SO100)")
        else:
            from robots.aider import AiderAdapter
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
        """设置舵机 ID 配置（从 Server 获取）"""
        if not config:
            print("⚠️ 收到空的舵机配置")
            return False
        
        self.servo_ids = config
        
        # 更新底盘和升降轴引用 (适配新的 base_lift_bus 结构)
        try:
            # 优先从 base_lift_bus 获取
            base_config = config.get('base_lift_bus', {}).get('base', {})
            lift_config = config.get('base_lift_bus', {}).get('lift_axis', {})
            
            if base_config:
                self.base_motors = list(base_config.keys())
                print(f"✅ 底盘舵机配置: {self.base_motors}")
            
            if lift_config and 'axis1' in lift_config:
                self.lift_motor = lift_config['axis1'].get('id')
                
            print(f"✅ 舵机配置已更新: {len(config)} 个总线配置")
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

            # ✅ 第一步：从 Server 获取舵机配置（包含端口和 ID 映射）
            from comm.api.client import ServerAPIClient
            api_client = ServerAPIClient()
            servo_config = api_client.get_servo_ids_config()
            
            if not servo_config:
                print("❌ 未能从 Server 获取舵机配置")
                return False
            
            # 保存配置
            self.set_servo_ids_config(servo_config)
            print("✅ 舵机配置已从 Server 同步")
            
            # ✅ 关键修改：自动发现舵机所在的串口（根据 ID 全局唯一性）
            from controller.motor_controller import MotorController
            
            # ✅ 复用 MotorController 的端口自动发现方法
            temp_mc = MotorController()
            servo_config, updated = temp_mc.auto_discover_ports_from_config(servo_config)
            
            if updated:
                self.set_servo_ids_config(servo_config)
            
            # ✅ 第二步：根据配置动态连接左臂、右臂、底盘和升降轴
            from drivers.feetech.st3215_driver import ST3215Driver

            # 连接左臂
            try:
                left_port = servo_config.get('left_bus', {}).get('port')
                if not left_port:
                    print("❌ 左臂端口未配置")
                    self.left_arm_connected = False
                else:
                    print(f"🔌 尝试连接左臂: {left_port}")
                    self.left_robot = ST3215Driver(port=left_port, baudrate=1000000)
                    success = self.left_robot.connect()
                    
                    if success:
                        self.left_arm_connected = True
                        print(f"✅ 左臂连接成功 ({left_port})")
                    else:
                        print(f"❌ 左臂连接失败 ({left_port})")
                        self.left_arm_connected = False
            except Exception as e:
                print(f"❌ 左臂连接异常: {e}")
                self.left_arm_connected = False

            # 连接右臂
            try:
                right_port = servo_config.get('right_bus', {}).get('port')
                if not right_port:
                    print("❌ 右臂端口未配置")
                    self.right_arm_connected = False
                else:
                    print(f"🔌 尝试连接右臂: {right_port}")
                    self.right_robot = ST3215Driver(port=right_port, baudrate=1000000)
                    success = self.right_robot.connect()

                    if success:
                        self.right_arm_connected = True
                        print(f"✅ 右臂连接成功 ({right_port})")
                    else:
                        print(f"❌ 右臂连接失败 ({right_port})")
                        self.right_arm_connected = False
            except Exception as e:
                print(f"❌ 右臂连接异常: {e}")
                self.right_arm_connected = False

            # ✅ 连接底盘和升降轴（base_lift_bus）
            try:
                base_port = servo_config.get('base_lift_bus', {}).get('port')
                if not base_port:
                    print("⚠️ 底盘端口未配置，跳过底盘连接")
                    self.base_connected = False
                    self.lift_connected = False
                else:
                    print(f"🔌 尝试连接底盘和升降轴: {base_port}")
                    
                    # 创建底盘驱动实例
                    self.base_robot = ST3215Driver(port=base_port, baudrate=1000000)
                    success = self.base_robot.connect()
                    
                    if success:
                        self.base_connected = True
                        self.lift_connected = True  # 升降轴与底盘共用同一串口
                        print(f"✅ 底盘和升降轴连接成功 ({base_port})")
                        
                        # 读取初始高度
                        lift_config = servo_config.get('base_lift_bus', {}).get('lift_axis', {})
                        if 'axis1' in lift_config:
                            lift_servo_id = lift_config['axis1'].get('id')
                            if lift_servo_id:
                                position = self.base_robot.get_position(lift_servo_id)
                                if position is not None:
                                    # 位置值转换为毫米（根据实际情况调整转换公式）
                                    self.lift_height_mm = int((position / 4095.0) * 1000)
                                    print(f"✅ 升降轴初始高度: {self.lift_height_mm}mm")
                    else:
                        print(f"❌ 底盘连接失败 ({base_port})")
                        self.base_connected = False
                        self.lift_connected = False
            except Exception as e:
                print(f"❌ 底盘连接异常: {e}")
                self.base_connected = False
                self.lift_connected = False

            # 至少一个组件连接成功即标记为已连接
            self.is_connected = (
                self.left_arm_connected or 
                self.right_arm_connected or 
                self.base_connected
            )

            if self.is_connected:
                # ✅ 第三步：初始化底层驱动（MotorController）
                # 优先使用左臂端口作为主控端口，如果没有则使用底盘端口
                control_port = None
                if servo_config.get('left_bus', {}).get('port'):
                    control_port = servo_config['left_bus']['port']
                elif servo_config.get('base_lift_bus', {}).get('port'):
                    control_port = servo_config['base_lift_bus']['port']
                
                if control_port and not self.motor_controller.driver:
                    try:
                        driver = ST3215Driver(port=control_port, baudrate=1000000)
                        if driver.connect():
                            self.motor_controller.driver = driver
                            print(f"✅ 底层驱动已在 {control_port} 初始化")
                        else:
                            print(f"⚠️ 无法连接到 {control_port}")
                    except Exception as e:
                        print(f"❌ 初始化驱动失败: {e}")

                # ✅ 第四步：读取初始关节状态
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
                left_arm_config = self.servo_ids.get('left_bus', {}).get('left_arm', {})
                angles = []
                
                for joint_name, joint_info in left_arm_config.items():
                    servo_id = joint_info.get('id')
                    if servo_id:
                        # 读取位置并转换为角度
                        position = self.left_robot.get_position(servo_id)
                        if position is not None:
                            # 位置值 (0-4095) 转换为角度 (-180~180)
                            angle = (position / 4095.0) * 360.0 - 180.0
                            angles.append(angle)
                        else:
                            angles.append(0.0)  # 默认值
                    else:
                        angles.append(0.0)
                
                if len(angles) == NUM_JOINTS:
                    self.left_arm_angles = np.array(angles)
                    print(f"✅ 左臂初始状态: {self.left_arm_angles.round(1)}")
                else:
                    print(f"⚠️ 左臂舵机数量不匹配: {len(angles)} != {NUM_JOINTS}")
            
            # 右臂
            if self.right_robot and self.right_arm_connected:
                right_arm_config = self.servo_ids.get('right_bus', {}).get('right_arm', {})
                angles = []
                
                for joint_name, joint_info in right_arm_config.items():
                    servo_id = joint_info.get('id')
                    if servo_id:
                        # 读取位置并转换为角度
                        position = self.right_robot.get_position(servo_id)
                        if position is not None:
                            # 位置值 (0-4095) 转换为角度 (-180~180)
                            angle = (position / 4095.0) * 360.0 - 180.0
                            angles.append(angle)
                        else:
                            angles.append(0.0)  # 默认值
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

    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict,
                         end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray,
                         joint_limits_max_deg: np.ndarray):
        """设置运动学解算器。委托给对应的适配器。"""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()

        self.adapter.setup(self.visualizer, self.config)
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
                    # 遍历左臂所有舵机，禁用扭矩
                    left_arm_config = self.servo_ids.get('left_bus', {}).get('left_arm', {})
                    for joint_name, joint_info in left_arm_config.items():
                        servo_id = joint_info.get('id')
                        if servo_id:
                            self.left_robot.set_torque(servo_id, False)

            if arm is None or arm == "right":
                if self.right_robot and self.right_arm_connected:
                    print("正在禁能右臂力矩...")
                    # 遍历右臂所有舵机，禁用扭矩
                    right_arm_config = self.servo_ids.get('right_bus', {}).get('right_arm', {})
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
        
        actions = self.adapter.build_hardware_actions(self.servo_ids)

        # 派发位置命令（双臂关节角度）
        for cmd in actions.get("position_commands", []):
            if cmd.get("targets"):
                self.motor_controller.sync_write_positions(cmd["port"], cmd["targets"], time_ms=0)

        # 派发速度命令（底盘轮子 + 升降轴）
        for cmd in actions.get("speed_commands", []):
            if not cmd.get("targets"):
                continue
            port = cmd["port"]
            for servo_id in cmd["targets"]:
                self._ensure_speed_mode(port, servo_id)
            self.motor_controller.sync_write_speeds(port, cmd["targets"])

    def _ensure_speed_mode(self, port, servo_id):
        """确保指定舵机处于速度模式（同一 port+servo 只切换一次）。"""
        key = f"_speed_mode_{port}_{servo_id}"
        if not hasattr(self, key):
            self.motor_controller.set_velocity_mode(port, servo_id)
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

        from inputs.base import ControlMode

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
