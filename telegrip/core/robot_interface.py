"""
SO100 遥操作系统的机器人接口模块。
提供带安全检查的机器人设备封装和便捷方法。
"""

import numpy as np
import time
import logging
import os
import sys
import yaml
from pathlib import Path
from typing import Optional, Dict, Tuple

# New lerobot structure imports

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE,
    WRIST_FLEX_INDEX, URDF_TO_INTERNAL_NAME_MAP
)
from .kinematics import ForwardKinematics, IKSolver

class RobotInterface:
    """带安全功能的 SO100 机器人控制高级接口。"""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        self.base_robot = None  # ✅ 底盘驱动实例
        self.is_connected = False
        self.is_engaged = False  # 电机使能状态
        
        # 初始化底层电机控制器（用于直接控制真机）
        from controller.motor_controller import MotorController
        self.motor_controller = MotorController()

        # 各机械臂连接状态
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # 舵机 ID 配置（由 Server 下发，初始为空）
        self.servo_ids = {}
        
        # 底盘和升降轴状态
        self.base_motors = []
        self.lift_motor = None

        # 关节状态
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)

        # 关节限位(由可视化器设置)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)

        # 运动学解算器(PyBullet 设置后初始化)
        self.fk_solvers = {'left': None, 'right': None}
        self.ik_solvers = {'left': None, 'right': None}

        # 控制时序
        self.last_send_time = 0

        # 错误跟踪 - 各机械臂独立
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3  # 每个机械臂允许的错误次数上限
        self.max_general_errors = 8  # 总错误次数上限

        # 安全关机的初始位置
        self.initial_left_arm = np.array([0, -100, 100, 60, 0, 0])
        self.initial_right_arm = np.array([0, -100, 100, 60, 0, 0])

        # 底盘状态 (由 control_loop 更新)
        self.base_connected = False
        self.base_velocity_target = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # 升降轴状态 (由 control_loop 更新)
        self.lift_connected = False
        self.lift_height_mm = 0  # 升降轴高度(毫米)
        self.lift_velocity = 0  # ✅ 升降轴速度（-1000~1000，负=升，正=降）

        # 仿真相关状态 (由 control_loop 更新)
        self.vr_raw_data = {}  # VR 原始数据
        self.left_arm_state = None  # 左臂状态对象
        self.right_arm_state = None  # 右臂状态对象
        self.visualizer = None  # PyBullet 可视化器
    
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
                self.base_motors = [
                    base_config.get('front_wheel', {}).get('id'),
                    base_config.get('left_wheel', {}).get('id'),
                    base_config.get('right_wheel', {}).get('id')
                ]
            
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
            from api.server_api_client import ServerAPIClient
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
        """使用 PyBullet 组件为两个机械臂设置运动学解算器。"""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()

        # 为两个机械臂设置解算器
        for arm in ['left', 'right']:
            self.fk_solvers[arm] = ForwardKinematics(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm]
            )

            self.ik_solvers[arm] = IKSolver(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm],
                joint_limits_min_deg, joint_limits_max_deg, arm_name=arm
            )

        print("两个机械臂的运动学解算器已初始化")

    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """获取指定机械臂的当前末端执行器位置。"""
        if arm == "left":
            angles = self.left_arm_angles
        elif arm == "right":
            angles = self.right_arm_angles
        else:
            raise ValueError(f"无效的机械臂: {arm}")

        if self.fk_solvers[arm]:
            position, _ = self.fk_solvers[arm].compute(angles)
            return position
        else:
            default_position = np.array([0.2, 0.0, 0.15])
            return default_position

    def solve_ik(self, arm: str, target_position: np.ndarray,
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """求解指定机械臂的逆运动学。"""
        if arm == "left":
            current_angles = self.left_arm_angles
        elif arm == "right":
            current_angles = self.right_arm_angles
        else:
            raise ValueError(f"无效的机械臂: {arm}")

        if self.ik_solvers[arm]:
            return self.ik_solvers[arm].solve(target_position, target_orientation, current_angles)
        else:
            return current_angles[:3]  # 如果没有 IK 解算器，返回当前角度

    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """将关节角度限制在安全范围内，对问题关节留出余量。"""
        # 创建副本以避免修改原始数据
        processed_angles = joint_angles.copy()

        # 首先，规范化可以环绕的角度(如 shoulder_pan)
        # 检查第一个关节 (shoulder_pan) 是否超出限位但可以环绕
        shoulder_pan_idx = 0
        shoulder_pan_angle = processed_angles[shoulder_pan_idx]
        min_limit = self.joint_limits_min_deg[shoulder_pan_idx]  # -120.3°
        max_limit = self.joint_limits_max_deg[shoulder_pan_idx]  # +120.3°

        # 尝试将角度环绕到限位内的等效角度
        if shoulder_pan_angle < min_limit or shoulder_pan_angle > max_limit:
            # 尝试 ±360° 环绕
            for offset in [-360.0, 360.0]:
                wrapped_angle = shoulder_pan_angle + offset
                if min_limit <= wrapped_angle <= max_limit:
                    print(f"将 shoulder_pan 从 {shoulder_pan_angle:.1f}° 环绕到 {wrapped_angle:.1f}°")
                    processed_angles[shoulder_pan_idx] = wrapped_angle
                    break

        # 对所有关节应用标准关节限位
        return np.clip(processed_angles, self.joint_limits_min_deg, self.joint_limits_max_deg)

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float):
        """使用 IK 解和直接腕部/夹爪控制更新指定机械臂的关节角度。"""
        if arm == "left":
            target_angles = self.left_arm_angles
        elif arm == "right":
            target_angles = self.right_arm_angles
        else:
            raise ValueError(f"无效的机械臂: {arm}")

        # 用 IK 解更新前 3 个关节
        target_angles[:3] = ik_angles

        # 直接设置腕部角度
        target_angles[3] = wrist_flex
        target_angles[4] = wrist_roll

        # 单独处理夹爪(限制在夹爪限位内)
        target_angles[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)

        # 对所有关节应用关节限位(除了我们特殊处理的夹爪)
        clamped_angles = self.clamp_joint_angles(target_angles)

        # 保留夹爪控制(如果有意设置则不限制夹爪)
        clamped_angles[5] = target_angles[5]

        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles

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
        """
        构造完整的机器人动作字典（真机和仿真共用）。
        包含：双臂角度、三轮底盘速度、升降轴速度。
        """
        from ..config import GRIPPER_INDEX

        # 1. 机械臂部分（应用夹爪映射）
        left_angles = self.left_arm_angles.copy()
        right_angles = self.right_arm_angles.copy()

        # 夹爪线性映射：VR trigger 0-1 → 角度 0°~-90°
        left_trigger = self.vr_raw_data.get('leftController', {}).get('trigger', None)
        right_trigger = self.vr_raw_data.get('rightController', {}).get('trigger', None)

        if left_trigger is not None and len(left_angles) > GRIPPER_INDEX:
            left_angles[GRIPPER_INDEX] = -left_trigger * 90.0

        if right_trigger is not None and len(right_angles) > GRIPPER_INDEX:
            right_angles[GRIPPER_INDEX] = -right_trigger * 90.0

        # 2. 底盘部分（三轮全向轮运动学）
        from .wheels import body_to_wheel_raw

        # ✅ 统一提高旋转速度增益（同时影响 VR 和键盘）
        ROTATION_GAIN = 100.0
        theta_scaled = self.base_velocity_target["theta"] * ROTATION_GAIN

        wheel_speeds = body_to_wheel_raw(
            self.base_velocity_target["x"],
            self.base_velocity_target["y"],
            theta_scaled  # 使用放大后的旋转速度
        )

        # 🔍 调试：打印运动学转换结果
        if abs(self.base_velocity_target["x"]) > 0.01 or abs(self.base_velocity_target["theta"]) > 0.01:
            print(f"🔧 运动学转换: x={self.base_velocity_target['x']}, y={self.base_velocity_target['y']}, theta={self.base_velocity_target['theta']}")
            print(f"   → front={wheel_speeds.get('base_back_wheel', 0)}, left={wheel_speeds.get('base_left_wheel', 0)}, right={wheel_speeds.get('base_right_wheel', 0)}")

        # 3. 组装完整 action
        action = {
            # 双臂角度
            "left_arm_angles": left_angles,
            "right_arm_angles": right_angles,
            # 底盘速度（与 servo_ids.yaml 中的键名对应）
            "base.front_wheel.vel": wheel_speeds["base_back_wheel"],   # 后轮输出 -> 前轮舵机
            "base.left_wheel.vel": wheel_speeds["base_left_wheel"],    # 左轮输出 -> 左轮舵机
            "base.right_wheel.vel": wheel_speeds["base_right_wheel"],  # 右轮输出 -> 右轮舵机
            # ✅ 升降轴速度（逆时针=升，顺时针=降）
            "lift.axis1.vel": self.lift_velocity,
        }

        return action

    def _send_to_hardware(self):
        """发送指令到真机硬件（双臂 + 底盘 + 升降轴）。"""
        # ✅ 复用 motor_controller 的批量同步控制
        if not self.motor_controller:
            return
        
        # 构建完整的动作字典
        action = self.build_robot_action()
        
        # ✅ 1. 收集左臂舵机角度
        left_targets = {}
        left_arm_config = self.servo_ids.get('left_bus', {}).get('left_arm', {})
        left_angles = action.get("left_arm_angles", [])
        for i, (joint_name, joint_info) in enumerate(left_arm_config.items()):
            if i < len(left_angles):
                servo_id = joint_info['id']
                angle = left_angles[i]
                left_targets[servo_id] = angle
        
        # ✅ 2. 收集右臂舵机角度
        right_targets = {}
        right_arm_config = self.servo_ids.get('right_bus', {}).get('right_arm', {})
        right_angles = action.get("right_arm_angles", [])
        for i, (joint_name, joint_info) in enumerate(right_arm_config.items()):
            if i < len(right_angles):
                servo_id = joint_info['id']
                angle = right_angles[i]
                right_targets[servo_id] = angle
        
        # ✅ 3. 批量同步写入左臂（50ms 运动时间）
        if left_targets:
            left_port = self.servo_ids.get('left_bus', {}).get('port')
            if left_port:
                self.motor_controller.sync_write_positions(left_port, left_targets, time_ms=0)
        
        # ✅ 4. 批量同步写入右臂（50ms 运动时间）
        if right_targets:
            right_port = self.servo_ids.get('right_bus', {}).get('port')
            if right_port:
                self.motor_controller.sync_write_positions(right_port, right_targets, time_ms=0)
        
        # ✅ 5. 处理底盘速度（如果配置了）
        base_config = self.servo_ids.get('base_lift_bus', {}).get('base', {})
        if base_config:
            base_port = self.servo_ids.get('base_lift_bus', {}).get('port')
            if base_port and hasattr(self.motor_controller, 'sync_write_speeds'):
                speed_targets = {}
                for joint_name, joint_info in base_config.items():
                    vel_key = f"base.{joint_name}.vel"
                    if vel_key in action:
                        vel = int(action[vel_key])
                        servo_id = joint_info['id']
                        
                        # ✅ 关键：先切换到速度模式（只切换一次）
                        if not hasattr(self, '_base_servos_mode_set') or not self._base_servos_mode_set:
                            print(f"🔄 切换底盘舵机 {servo_id} 到速度模式")
                            self.motor_controller.set_velocity_mode(base_port, servo_id)
                        
                        # ✅ 保留速度符号，负数=逆时针，正数=顺时针
                        speed_targets[servo_id] = vel
                    else:
                        print(f"⚠️ 未找到底盘速度键: {vel_key}")
                
                # 标记底盘舵机已切换到速度模式
                if speed_targets:
                    self._base_servos_mode_set = True
                    self.motor_controller.sync_write_speeds(base_port, speed_targets)
                else:
                    print(f"⚠️ 底盘速度目标为空，action keys: {list(action.keys())}")
        
        # ✅ 6. 处理升降轴速度（如果配置了）
        lift_axis_config = self.servo_ids.get('base_lift_bus', {}).get('lift_axis', {})
        if lift_axis_config:
            lift_port = self.servo_ids.get('base_lift_bus', {}).get('port')
            if lift_port and hasattr(self.motor_controller, 'sync_write_speeds'):
                speed_targets = {}
                for axis_name, axis_info in lift_axis_config.items():
                    vel_key = f"lift.{axis_name}.vel"
                    if vel_key in action:
                        vel = int(action[vel_key])
                        servo_id = axis_info['id']
                        
                        # ✅ 关键：先切换到速度模式（只切换一次）
                        if not hasattr(self, '_lift_axis_mode_set') or not self._lift_axis_mode_set:
                            print(f"🔄 切换升降轴舵机 {servo_id} 到速度模式")
                            self.motor_controller.set_velocity_mode(lift_port, servo_id)
                            self._lift_axis_mode_set = True
                        
                        # ✅ 保留速度符号，负数=逆时针（升），正数=顺时针（降）
                        speed_targets[servo_id] = vel
                
                if speed_targets:
                    self.motor_controller.sync_write_speeds(lift_port, speed_targets)

    def _update_simulation(self):
        """更新仿真可视化（使用 build_robot_action 统一构建的数据）。"""
        if not self.visualizer:
            print("⚠️ visualizer 未初始化，跳过仿真更新")
            return

        # 1. 构建完整的机器人动作（包含夹爪映射）
        action = self.build_robot_action()

        # ✅ 2. 仿真专用：根据升降轴速度积分计算高度（仅用于可视化）
        if self.config.aloha_enabled and hasattr(self, 'lift_velocity'):
            # 速度范围 -1000~1000，转换为 m/s
            MAX_LIFT_SPEED_MPS = 0.1  # 最大升降速度 0.1 m/s
            lift_speed_mps = (self.lift_velocity / 1000.0) * MAX_LIFT_SPEED_MPS
            
            # 积分计算高度：h = h + v * dt（使用浮点数避免精度丢失）
            from ..config import TelegripConfig
            dt = TelegripConfig().send_interval  # 时间步长 0.02s
            delta_m = lift_speed_mps * dt  # 每帧变化量（米）
            old_height_m = self.lift_height_mm / 1000.0
            new_height_m = old_height_m + delta_m
            self.lift_height_mm = new_height_m * 1000  # 存储为毫米（浮点数）
            
            # 更新仿真中的升降轴高度
            if self.visualizer:
                self.visualizer.set_aloha_height(new_height_m)
                # 只在速度非零时打印日志
                if self.lift_velocity != 0:
                    print(f"📊 仿真升降轴: {old_height_m*1000:.1f}mm → {self.lift_height_mm:.1f}mm (Δ={delta_m*1000:.2f}mm, vel={self.lift_velocity}, height={new_height_m:.4f}m)")

        # 3. 更新仿真中的底盘位置
        if self.config.aloha_enabled:
            sim_action = {
                "base.vx": self.base_velocity_target["x"],
                "base.vy": self.base_velocity_target["y"],
                "base.vtheta": self.base_velocity_target["theta"],
            }
            self.visualizer.update_mobile_base_simulation(sim_action)

        # 4. 提取双臂角度用于更新姿态
        left_angles = action["left_arm_angles"]
        right_angles = action["right_arm_angles"]

        # 5. 更新 SO100 机器人姿态
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')

        # 6. 如果启用了 Aloha,将 SO100 IK 结果映射到 Aloha 双臂
        if self.config.aloha_enabled and self.visualizer.aloha_id is not None:
            self.visualizer.update_aloha_arm_pose(left_angles, 'left')
            self.visualizer.update_aloha_arm_pose(right_angles, 'right')

        # 7. 更新可视化标记点
        from ..inputs.base import ControlMode
        if self.left_arm_state and self.left_arm_state.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm_state.target_position is not None:
                current_pos = self.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)

            if self.left_arm_state.goal_position is not None:
                self.visualizer.update_marker_position("left_goal", self.left_arm_state.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm_state.goal_position)
        else:
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")

        if self.right_arm_state and self.right_arm_state.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm_state.target_position is not None:
                current_pos = self.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)

            if self.right_arm_state.goal_position is not None:
                self.visualizer.update_marker_position("right_goal", self.right_arm_state.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm_state.goal_position)
        else:
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")

        # 8. 推进仿真
        self.visualizer.step_simulation()
