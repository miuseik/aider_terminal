"""
SO100 遥操作系统的机器人接口模块。
提供带安全检查的机器人设备封装和便捷方法。
"""

import numpy as np
import time
import logging
import os
import sys
import contextlib
import yaml
from pathlib import Path
from typing import Optional, Dict, Tuple

# New lerobot structure imports
from robots.so_follower import SOFollower, SOFollowerRobotConfig

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE,
    WRIST_FLEX_INDEX, URDF_TO_INTERNAL_NAME_MAP
)
from .kinematics import ForwardKinematics, IKSolver

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """上下文管理器，在文件描述符级别抑制标准输出和错误输出。"""
    # 保存原始文件描述符
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()

    # 保存原始文件描述符
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)

    try:
        # 打开 /dev/null
        devnull_fd = os.open(os.devnull, os.O_WRONLY)

        # 将标准输出和错误重定向到 /dev/null
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)

        yield

    finally:
        # 恢复原始文件描述符
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)

        # 关闭保存的文件描述符
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """带安全功能的 SO100 机器人控制高级接口。"""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        self.is_connected = False
        self.is_engaged = False  # 电机使能状态

        # 各机械臂连接状态
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # 舵机 ID 配置（由 Server 下发，先使用默认值）
        self.servo_ids = {
            'left_bus': {
                'port': '/dev/ttyUSB0',
                'left_arm': {'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3, 'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6},
                'base': {'left_wheel': 8, 'back_wheel': 9, 'right_wheel': 10},
                'lift_axis': 11,
                'neck': 12
            },
            'right_bus': {
                'port': '/dev/ttyUSB1',
                'right_arm': {'shoulder_pan': 13, 'shoulder_lift': 14, 'elbow_flex': 15, 'wrist_flex': 16, 'wrist_roll': 17, 'gripper': 18}
            }
        }
        
        # 底盘和升降轴状态
        self.base_motors = [
            self.servo_ids['left_bus']['base']['left_wheel'],
            self.servo_ids['left_bus']['base']['back_wheel'],
            self.servo_ids['left_bus']['base']['right_wheel']
        ]
        self.lift_motor = self.servo_ids['left_bus']['lift_axis']

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

        # 仿真相关状态 (由 control_loop 更新)
        self.vr_raw_data = {}  # VR 原始数据
        self.left_arm_state = None  # 左臂状态对象
        self.right_arm_state = None  # 右臂状态对象
        self.visualizer = None  # PyBullet 可视化器
    
    def set_servo_ids_config(self, config: dict):
        """设置舵机 ID 配置（从 Server 获取）"""
        if not config:
            logger.warning("⚠️ 收到空的舵机配置")
            return False
        
        self.servo_ids = config
        
        # 更新底盘和升降轴引用
        try:
            self.base_motors = [
                self.servo_ids['left_bus']['base']['left_wheel'],
                self.servo_ids['left_bus']['base']['back_wheel'],
                self.servo_ids['left_bus']['base']['right_wheel']
            ]
            self.lift_motor = self.servo_ids['left_bus']['lift_axis']
            logger.info(f"✅ 舵机配置已更新: {len(self.servo_ids)} 个总线")
            return True
        except Exception as e:
            logger.error(f"❌ 解析舵机配置失败: {e}")
            return False

    def setup_robot_configs(self) -> Tuple[SOFollowerRobotConfig, SOFollowerRobotConfig]:
        """为两个机械臂创建机器人配置。"""
        logger.info(f"设置机器人配置，端口: {self.config.follower_ports}")

        left_config = SOFollowerRobotConfig(
            port=self.config.follower_ports["left"],
            id="left_follower",
            use_degrees=True,  # 使用角度制便于调试
            disable_torque_on_disconnect=True
        )

        right_config = SOFollowerRobotConfig(
            port=self.config.follower_ports["right"],
            id="right_follower",
            use_degrees=True,  # 使用角度制便于调试
            disable_torque_on_disconnect=True
        )

        return left_config, right_config

    def connect(self) -> bool:
        """连接机器人硬件。"""
        if self.is_connected:
            logger.info("机器人接口已连接")
            return True

        if not self.config.enable_robot:
            logger.info("配置中禁用了机器人接口")
            self.is_connected = True  # 测试时标记为“已连接”
            return True

        # 根据日志级别设置输出抑制
        should_suppress = (self.config.log_level == "warning" or
                           self.config.log_level == "critical" or
                           self.config.log_level == "error")

        try:
            left_config, right_config = self.setup_robot_configs()
            if not should_suppress:
                logger.info("正在连接机器人...")

            # 连接左臂
            try:
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.left_robot = SOFollower(left_config)
                        self.left_robot.connect()
                else:
                    self.left_robot = SOFollower(left_config)
                    self.left_robot.connect()
                self.left_arm_connected = True
                logger.info("✅ 左臂连接成功")
            except Exception as e:
                logger.error(f"❌ 左臂连接失败: {e}")
                self.left_arm_connected = False

            # 连接右臂  
            try:
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.right_robot = SOFollower(right_config)
                        self.right_robot.connect()
                else:
                    self.right_robot = SOFollower(right_config)
                    self.right_robot.connect()
                self.right_arm_connected = True
                logger.info("✅ 右臂连接成功")
            except Exception as e:
                logger.error(f"❌ 右臂连接失败: {e}")
                self.right_arm_connected = False

            # 至少一个机械臂连接成功即标记为已连接
            self.is_connected = self.left_arm_connected or self.right_arm_connected

            if self.is_connected:
                # 初始化关节状态
                self._read_initial_state()
                logger.info(f"🤖 机器人接口已连接: 左臂={self.left_arm_connected}, 右臂={self.right_arm_connected}")
                
                # 从 Server 获取最新舵机配置
                from router.server_api_client import ServerAPIClient
                api_client = ServerAPIClient()
                config = api_client.get_servo_ids_config()
                if config:
                    self.set_servo_ids_config(config)
            else:
                logger.error("❌ 无法连接任何机械臂")

            return self.is_connected

        except Exception as e:
            logger.error(f"❌ 机器人连接异常: {e}")
            self.is_connected = False
            return False

    def _read_initial_state(self):
        """从机器人读取初始关节状态。"""
        try:
            if self.left_robot and self.left_arm_connected:
                observation = self.left_robot.get_observation()
                if observation:
                    # 从观测数据中提取关节位置
                    self.left_arm_angles = np.array([
                        observation['shoulder_pan.pos'],
                        observation['shoulder_lift.pos'],
                        observation['elbow_flex.pos'],
                        observation['wrist_flex.pos'],
                        observation['wrist_roll.pos'],
                        observation['gripper.pos']
                    ])
                    logger.info(f"左臂初始状态: {self.left_arm_angles.round(1)}")

            if self.right_robot and self.right_arm_connected:
                observation = self.right_robot.get_observation()
                if observation:
                    # 从观测数据中提取关节位置
                    self.right_arm_angles = np.array([
                        observation['shoulder_pan.pos'],
                        observation['shoulder_lift.pos'],
                        observation['elbow_flex.pos'],
                        observation['wrist_flex.pos'],
                        observation['wrist_roll.pos'],
                        observation['gripper.pos']
                    ])
                    logger.info(f"右臂初始状态: {self.right_arm_angles.round(1)}")

        except Exception as e:
            logger.error(f"读取初始状态错误: {e}")

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

        logger.info("两个机械臂的运动学解算器已初始化")

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
                    logger.debug(f"将 shoulder_pan 从 {shoulder_pan_angle:.1f}° 环绕到 {wrapped_angle:.1f}°")
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
        if not self.is_connected:
            logger.warning("无法使能机器人: 未连接")
            return False

        self.is_engaged = True
        logger.info("🔌 机器人电机已使能 - 将发送指令")
        return True

    def disengage(self) -> bool:
        """禁能机器人电机(停止发送指令)。"""
        if not self.is_connected:
            logger.info("机器人已断开")
            return True

        try:
            # 禁能前返回安全位置
            self.return_to_initial_position()

            # 禁能力矩
            self.disable_torque()

            self.is_engaged = False
            logger.info("🔌 机器人电机已禁能 - 指令停止")
            return True

        except Exception as e:
            logger.error(f"禁能机器人错误: {e}")
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
                logger.error(f"发送机器人指令错误: {e}")
                self.general_errors += 1
                if self.general_errors > self.max_general_errors:
                    self.is_connected = False
                    logger.error("❌ 机器人接口因重复错误而断开")
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
            logger.debug(f"读取 {arm} 实际关节角度错误: {e}")

        # 如果无法读取实际角度，回退到指令角度
        return self.get_arm_angles(arm)

    def return_to_initial_position(self):
        """将两个机械臂返回到初始位置。"""
        logger.info("⏪ 正在将机器人返回到初始位置...")

        try:
            # 设置初始位置 - 无方向映射
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()

            # 发送几次指令以确保移动
            for i in range(10):
                self.send_command
                time.sleep(0.1)

            logger.info("✅ 机器人已返回到初始位置")
        except Exception as e:
            logger.error(f"返回初始位置错误: {e}")

    def disable_torque(self, arm: str = None):
        """禁能机器人关节力矩。

        Args:
            arm: 'left', 'right', 或 None 表示两个机械臂
        """
        if not self.is_connected:
            return

        try:
            if arm is None or arm == "left":
                if self.left_robot and self.left_arm_connected:
                    logger.info("正在禁能左臂力矩...")
                    self.left_robot.bus.disable_torque()

            if arm is None or arm == "right":
                if self.right_robot and self.right_arm_connected:
                    logger.info("正在禁能右臂力矩...")
                    self.right_robot.bus.disable_torque()

        except Exception as e:
            logger.error(f"禁能力矩错误: {e}")

    def disconnect(self):
        """断开与机器人硬件的连接。"""
        if not self.is_connected:
            return

        logger.info("正在断开机器人连接...")

        # 如果已使能，先返回初始位置
        if self.is_engaged:
            try:
                self.return_to_initial_position()
            except Exception as e:
                logger.error(f"返回初始位置错误: {e}")

        # 断开两个机械臂
        if self.left_robot:
            try:
                self.left_robot.disconnect()
            except Exception as e:
                logger.error(f"断开左臂错误: {e}")
            self.left_robot = None

        if self.right_robot:
            try:
                self.right_robot.disconnect()
            except Exception as e:
                logger.error(f"断开右臂错误: {e}")
            self.right_robot = None

        self.is_connected = False
        self.is_engaged = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("🔌 机器人已断开")

    def get_arm_connection_status(self, arm: str) -> bool:
        """根据设备文件存在性获取特定机械臂的连接状态。"""
        # 只检查设备文件存在性 - 忽略整体机器人连接状态
        if arm == "left":
            device_path = self.config.follower_ports["left"]
            return os.path.exists(device_path)
        elif arm == "right":
            device_path = self.config.follower_ports["right"]
            return os.path.exists(device_path)
        else:
            return False

    def update_arm_connection_status(self):
        """根据设备文件存在性更新各机械臂的连接状态。"""
        if self.is_connected:
            self.left_arm_connected = os.path.exists(self.config.follower_ports["left"])
            self.right_arm_connected = os.path.exists(self.config.follower_ports["right"])

    def build_robot_action(self) -> dict:
        """
        构造完整的机器人动作字典（真机和仿真共用）。
        包含：双臂角度、三轮底盘速度、升降轴高度。
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
        wheel_speeds = body_to_wheel_raw(
            self.base_velocity_target["x"],
            self.base_velocity_target["y"],
            self.base_velocity_target["theta"]
        )

        # 3. 组装完整 action
        action = {
            # 双臂角度
            "left_arm_angles": left_angles,
            "right_arm_angles": right_angles,
            # 底盘速度
            "base.left_wheel.vel": wheel_speeds["base_left_wheel"],
            "base.back_wheel.vel": wheel_speeds["base_back_wheel"],
            "base.right_wheel.vel": wheel_speeds["base_right_wheel"],
            # 升降轴高度（米 → 毫米）
            "lift.height_mm": int(self.lift_height_mm),
        }

        return action

    def _send_to_hardware(self):
        """发送指令到真机硬件（双臂 + 底盘 + 升降轴）。"""
        # 构建完整的动作字典
        action = self.build_robot_action()

        # 1. 发送左臂指令
        if self.left_robot and self.left_arm_connected:
            left_action_dict = {
                "shoulder_pan.pos": float(action["left_arm_angles"][0]),
                "shoulder_lift.pos": float(action["left_arm_angles"][1]),
                "elbow_flex.pos": float(action["left_arm_angles"][2]),
                "wrist_flex.pos": float(action["left_arm_angles"][3]),
                "wrist_roll.pos": float(action["left_arm_angles"][4]),
                "gripper.pos": float(action["left_arm_angles"][5])
            }
            self.left_robot.send_action(left_action_dict)

        # 2. 发送右臂指令
        if self.right_robot and self.right_arm_connected:
            right_action_dict = {
                "shoulder_pan.pos": float(action["right_arm_angles"][0]),
                "shoulder_lift.pos": float(action["right_arm_angles"][1]),
                "elbow_flex.pos": float(action["right_arm_angles"][2]),
                "wrist_flex.pos": float(action["right_arm_angles"][3]),
                "wrist_roll.pos": float(action["right_arm_angles"][4]),
                "gripper.pos": float(action["right_arm_angles"][5])
            }
            self.right_robot.send_action(right_action_dict)

        # 3. 发送底盘指令（三轮全向轮）
        if self.base_connected and self.left_robot:
            base_ids = self.servo_ids['left_bus']['base']
            base_wheel_goal_vel = {
                base_ids['left_wheel']: int(action["base.left_wheel.vel"]),
                base_ids['back_wheel']: int(action["base.back_wheel.vel"]),
                base_ids['right_wheel']: int(action["base.right_wheel.vel"])
            }
            try:
                self.left_robot.bus.sync_write_velocity(base_wheel_goal_vel)
            except Exception as e:
                logger.error(f"发送底盘指令错误: {e}")
                
        # 4. 发送升降轴指令
        if self.lift_connected and self.left_robot:
            try:
                lift_id = self.servo_ids['left_bus']['lift_axis']
                # 将毫米转换为舵机位置 (需要根据实际机械结构计算)
                # 假设: 1mm = 10脉冲 (需要根据螺距调整)
                lift_position = int(action["lift.height_mm"] * 10)
                self.left_robot.bus.write_position(lift_id, lift_position)
            except Exception as e:
                logger.error(f"发送升降轴指令错误: {e}")


    def _update_simulation(self):
        """更新仿真可视化（使用 build_robot_action 统一构建的数据）。"""
        if not self.visualizer:
            logger.debug("⚠️ visualizer 未初始化，跳过仿真更新")
            return

        # 1. 构建完整的机器人动作（包含夹爪映射）
        action = self.build_robot_action()

        # 2. 更新仿真中的底盘位置
        if self.config.aloha_enabled:
            sim_action = {
                "lift.height_mm": action["lift.height_mm"],
                "base.vx": self.base_velocity_target["x"],
                "base.vy": self.base_velocity_target["y"],
                "base.vtheta": self.base_velocity_target["theta"],
            }
            self.visualizer.update_mobile_base_simulation(sim_action)

        # 3. 提取双臂角度用于更新姿态
        left_angles = action["left_arm_angles"]
        right_angles = action["right_arm_angles"]

        # 4. 更新 SO100 机器人姿态
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')

        # 5. 如果启用了 Aloha,将 SO100 IK 结果映射到 Aloha 双臂
        if self.config.aloha_enabled and self.visualizer.aloha_id is not None:
            self.visualizer.update_aloha_arm_pose(left_angles, 'left')
            self.visualizer.update_aloha_arm_pose(right_angles, 'right')

        # 6. 更新可视化标记点
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

        # 7. 推进仿真
        self.visualizer.step_simulation()
