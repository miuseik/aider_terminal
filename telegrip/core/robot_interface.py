"""
SO100 遥操作系统的机器人接口模块。
提供带有安全检查的机器人设备封装和便捷方法。
"""

import numpy as np
import torch
import time
import logging
import os
import sys
import contextlib
from typing import Optional, Dict, Tuple

# New lerobot structure imports
from lerobot.robots.so_follower.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE, 
    WRIST_FLEX_INDEX, URDF_TO_INTERNAL_NAME_MAP
)
from .kinematics import ForwardKinematics, IKSolver

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """带有安全功能的 SO100 机器人控制高级接口。"""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        self.base_bus = None      # 底盘电机总线
        self.lift_bus = None      # 升降轴电机总线
        self.is_connected = False
        self.is_engaged = False  # New state for motor engagement
        
        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # 关节状态
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        
        # 关节限制(将由 visualizer 设置)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        # 运动学求解器(将在 PyBullet 设置后配置)
        self.fk_solvers = {'left': None, 'right': None}
        self.ik_solvers = {'left': None, 'right': None}
        
        # 控制时序
        self.last_send_time = 0
        
        # 错误跟踪 - 每个机械臂单独跟踪
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3  # Allow fewer errors per arm before marking as disconnected
        self.max_general_errors = 8  # Allow more general errors before full disconnection
        
        # 安全关闭的初始位置 - 恢复原始值
        self.initial_left_arm = np.array([0, -100, 100, 60, 0, 0])
        self.initial_right_arm = np.array([0, -100, 100, 60, 0, 0])
    
    def setup_robot_configs(self) -> Tuple[SOFollowerRobotConfig, SOFollowerRobotConfig]:
        """Create robot configurations for both arms."""
        logger.info(f"Setting up robot configs with ports: {self.config.follower_ports}")

        left_config = SOFollowerRobotConfig(
            port=self.config.follower_ports["left"],
            id="left_follower",
            use_degrees=True,  # Use degrees for easier debugging
            disable_torque_on_disconnect=True
        )

        right_config = SOFollowerRobotConfig(
            port=self.config.follower_ports["right"],
            id="right_follower",
            use_degrees=True,  # Use degrees for easier debugging
            disable_torque_on_disconnect=True
        )

        return left_config, right_config
    
    def connect(self) -> bool:
        """连接到机器人硬件。"""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True
        
        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True  # Mark as "connected" for testing
            return True
        
        # Setup suppression if requested
        should_suppress = (self.config.log_level == "warning" or 
                          self.config.log_level == "critical" or 
                          self.config.log_level == "error")
        
        try:
            left_config, right_config = self.setup_robot_configs()
            if not should_suppress:
                logger.info("Connecting to robot...")
            
            # 连接左机械臂
            try:
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.left_robot = SOFollower(left_config)
                        self.left_robot.connect()
                else:
                    self.left_robot = SOFollower(left_config)
                    self.left_robot.connect()
                self.left_arm_connected = True
                logger.info("✅ Left arm connected successfully")
            except Exception as e:
                logger.error(f"❌ Left arm connection failed: {e}")
                self.left_arm_connected = False
            
            # 连接右机械臂  
            try:
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.right_robot = SOFollower(right_config)
                        self.right_robot.connect()
                else:
                    self.right_robot = SOFollower(right_config)
                    self.right_robot.connect()
                self.right_arm_connected = True
                logger.info("✅ Right arm connected successfully")
            except Exception as e:
                logger.error(f"❌ Right arm connection failed: {e}")
                self.right_arm_connected = False

            # 连接移动底盘 (AlohaMini)
            if self.config._config_data.get("robot", {}).get("mobile_base", {}).get("enabled"):
                try:
                    base_port = self.config._config_data["robot"]["mobile_base"]["port"]
                    # 定义底盘电机: ID 8, 9, 10 对应左、后、右轮
                    base_motors = {
                        "base_left_wheel": Motor(id=8, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
                        "base_back_wheel": Motor(id=9, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
                        "base_right_wheel": Motor(id=10, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
                    }
                    self.base_bus = FeetechMotorsBus(port=base_port, motors=base_motors)
                    self.base_bus.connect(handshake=False)
                    logger.info(f"✅ Mobile base connected on {base_port}")
                except Exception as e:
                    logger.error(f"❌ Mobile base connection failed: {e}")

            # 连接升降轴 (AlohaMini)
            if self.config._config_data.get("robot", {}).get("lift_axis", {}).get("enabled"):
                try:
                    lift_port = self.config._config_data["robot"]["lift_axis"]["port"]
                    # 定义升降轴电机: ID 11
                    lift_motors = {
                        "lift_axis": Motor(id=11, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
                    }
                    self.lift_bus = FeetechMotorsBus(port=lift_port, motors=lift_motors)
                    self.lift_bus.connect(handshake=False)
                    logger.info(f"✅ Lift axis connected on {lift_port}")
                except Exception as e:
                    logger.error(f"❌ Lift axis connection failed: {e}")
                
            # 如果至少一个机械臂连接成功,标记为已连接
            self.is_connected = self.left_arm_connected or self.right_arm_connected
            
            if self.is_connected:
                # 初始化关节状态
                self._read_initial_state()
                logger.info(f"🤖 Robot interface connected: Left={self.left_arm_connected}, Right={self.right_arm_connected}")
            else:
                logger.error("❌ Failed to connect any robot arms")
                
            return self.is_connected
            
        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False
    
    def _read_initial_state(self):
        """从机器人读取初始关节状态。"""
        try:
            if self.left_robot and self.left_arm_connected:
                observation = self.left_robot.get_observation()
                if observation:
                    # Extract joint positions from observation
                    self.left_arm_angles = np.array([
                        observation['shoulder_pan.pos'],
                        observation['shoulder_lift.pos'],
                        observation['elbow_flex.pos'],
                        observation['wrist_flex.pos'],
                        observation['wrist_roll.pos'],
                        observation['gripper.pos']
                    ])
                    logger.info(f"Left arm initial state: {self.left_arm_angles.round(1)}")
                    
            if self.right_robot and self.right_arm_connected:
                observation = self.right_robot.get_observation()
                if observation:
                    # Extract joint positions from observation
                    self.right_arm_angles = np.array([
                        observation['shoulder_pan.pos'],
                        observation['shoulder_lift.pos'],
                        observation['elbow_flex.pos'],
                        observation['wrist_flex.pos'],
                        observation['wrist_roll.pos'],
                        observation['gripper.pos']
                    ])
                    logger.info(f"Right arm initial state: {self.right_arm_angles.round(1)}")
                    
        except Exception as e:
            logger.error(f"Error reading initial state: {e}")
    
    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict, 
                        end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray, 
                        joint_limits_max_deg: np.ndarray):
        """使用 PyBullet 组件为两个机械臂配置运动学求解器。"""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()
        
        # 为两个机械臂配置求解器
        for arm in ['left', 'right']:
            self.fk_solvers[arm] = ForwardKinematics(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm]
            )
            
            self.ik_solvers[arm] = IKSolver(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm],
                joint_limits_min_deg, joint_limits_max_deg, arm_name=arm
            )
        
        logger.info("Kinematics solvers initialized for both arms")
    
    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """获取指定机械臂的当前末端执行器位置。"""
        if arm == "left":
            angles = self.left_arm_angles
        elif arm == "right":
            angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
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
            raise ValueError(f"Invalid arm: {arm}")
        
        if self.ik_solvers[arm]:
            return self.ik_solvers[arm].solve(target_position, target_orientation, current_angles)
        else:
            return current_angles[:3]  # Return current angles if no IK solver
    
    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """将关节角度限制到安全范围内,问题关节添加余量。"""
        # 创建副本以避免修改原始数据
        processed_angles = joint_angles.copy()
        
        # 首先,规范化可以环绕的角度(如 shoulder_pan)
        # 检查第一个关节(shoulder_pan)是否超出限制但可以环绕
        shoulder_pan_idx = 0
        shoulder_pan_angle = processed_angles[shoulder_pan_idx]
        min_limit = self.joint_limits_min_deg[shoulder_pan_idx]  # -120.3°
        max_limit = self.joint_limits_max_deg[shoulder_pan_idx]  # +120.3°
        
        # Try to wrap the angle to an equivalent angle within limits
        if shoulder_pan_angle < min_limit or shoulder_pan_angle > max_limit:
            # 尝试通过 ±360° 环绕角度
            for offset in [-360.0, 360.0]:
                wrapped_angle = shoulder_pan_angle + offset
                if min_limit <= wrapped_angle <= max_limit:
                    logger.debug(f"Wrapped shoulder_pan from {shoulder_pan_angle:.1f}° to {wrapped_angle:.1f}°")
                    processed_angles[shoulder_pan_idx] = wrapped_angle
                    break
        
        # 对所有关节应用标准关节限制
        return np.clip(processed_angles, self.joint_limits_min_deg, self.joint_limits_max_deg)
    
    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float):
        """使用 IK 解和直接腕部/夹爪控制更新指定机械臂的关节角度。"""
        if arm == "left":
            target_angles = self.left_arm_angles
        elif arm == "right":
            target_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        # 用 IK 解更新前 3 个关节
        target_angles[:3] = ik_angles
        
        # 直接设置腕部角度
        target_angles[3] = wrist_flex
        target_angles[4] = wrist_roll
        
        # 单独处理夹爪(限制到夹爪范围)
        target_angles[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
        
        # 对所有关节应用关节限制(夹爪除外,我们特殊处理)
        clamped_angles = self.clamp_joint_angles(target_angles)
        
        # 保留夹爪控制(如果有意设置则不限制夹爪)
        clamped_angles[5] = target_angles[5]
        
        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles
    
    def engage(self) -> bool:
        """接合机器人电机(开始发送命令)。"""
        if not self.is_connected:
            logger.warning("Cannot engage robot: not connected")
            return False
        
        self.is_engaged = True
        logger.info("🔌 Robot motors ENGAGED - commands will be sent")
        return True
    
    def disengage(self) -> bool:
        """断开机器人电机(停止发送命令)。"""
        if not self.is_connected:
            logger.info("Robot already disconnected")
            return True
        
        try:
            # 断开前先返回安全位置
            self.return_to_initial_position()
            
            # 禁用扭矩
            self.disable_torque()
            
            self.is_engaged = False
            logger.info("🔌 Robot motors DISENGAGED - commands stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error disengaging robot: {e}")
            return False
    
    def send_command(self, action_dict: dict = None) -> bool:
        """使用字典格式向机器人发送当前关节角度及底盘/升降指令。"""
        if not self.is_connected or not self.is_engaged:
            return False
        
        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # Don't send too frequently
        
        try:
            # 1. 发送机械臂命令 (原有逻辑)
            success = True
            
            # 发送左机械臂命令
            if self.left_robot and self.left_arm_connected:
                try:
                    arm_action = {
                        "shoulder_pan.pos": float(self.left_arm_angles[0]),
                        "shoulder_lift.pos": float(self.left_arm_angles[1]),
                        "elbow_flex.pos": float(self.left_arm_angles[2]),
                        "wrist_flex.pos": float(self.left_arm_angles[3]),
                        "wrist_roll.pos": float(self.left_arm_angles[4]),
                        "gripper.pos": float(self.left_arm_angles[5])
                    }
                    self.left_robot.send_action(arm_action)
                except Exception as e:
                    logger.error(f"Error sending left arm command: {e}")
                    success = False
            
            # 发送右机械臂命令
            if self.right_robot and self.right_arm_connected:
                try:
                    arm_action = {
                        "shoulder_pan.pos": float(self.right_arm_angles[0]),
                        "shoulder_lift.pos": float(self.right_arm_angles[1]),
                        "elbow_flex.pos": float(self.right_arm_angles[2]),
                        "wrist_flex.pos": float(self.right_arm_angles[3]),
                        "wrist_roll.pos": float(self.right_arm_angles[4]),
                        "gripper.pos": float(self.right_arm_angles[5])
                    }
                    self.right_robot.send_action(arm_action)
                except Exception as e:
                    logger.error(f"Error sending right arm command: {e}")
                    success = False

            # 2. 发送底盘和升降轴命令 (新增逻辑)
            if action_dict:
                # 处理底盘速度
                if self.base_bus:
                    wheel_cmds = {}
                    for name in ["base_left_wheel", "base_back_wheel", "base_right_wheel"]:
                        if f"{name}.vel" in action_dict:
                            wheel_cmds[name] = action_dict[f"{name}.vel"]
                    if wheel_cmds:
                        # 批量写入 Goal_Velocity 寄存器
                        names = list(wheel_cmds.keys())
                        vals = list(wheel_cmds.values())
                        self.base_bus.write("Goal_Velocity", vals, names, normalize=False)

                # 处理升降轴高度
                if self.lift_bus and "lift.height_mm" in action_dict:
                    from .axis import height_mm_to_ticks
                    target_ticks = height_mm_to_ticks(action_dict["lift.height_mm"])
                    # 写入 Goal_Position 寄存器
                    self.lift_bus.write("Goal_Position", "lift_axis", target_ticks, normalize=False)
            
            self.last_send_time = current_time
            return success
            
        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            return False
    
    def set_gripper(self, arm: str, closed: bool):
        """设置指定机械臂的夹爪状态。"""
        angle = GRIPPER_CLOSED_ANGLE if closed else GRIPPER_OPEN_ANGLE
        
        if arm == "left":
            self.left_arm_angles[5] = angle
        elif arm == "right":
            self.right_arm_angles[5] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")
    
    def get_arm_angles(self, arm: str) -> np.ndarray:
        """获取指定机械臂的当前关节角度。"""
        if arm == "left":
            angles = self.left_arm_angles.copy()
        elif arm == "right":
            angles = self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        return angles
    
    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """获取指定机械臂的当前关节角度,用于 PyBullet 可视化。"""
        # 返回原始角度而不进行任何修正以进行正确诊断
        return self.get_arm_angles(arm)
    
    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """从机器人硬件获取实际关节角度(非命令角度)。"""
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
            logger.debug(f"Error reading actual arm angles for {arm}: {e}")
        
        # 如果无法读取实际角度,回退到命令角度
        return self.get_arm_angles(arm)
    
    def return_to_initial_position(self):
        """将两个机械臂返回到初始位置。"""
        logger.info("⏪ Returning robot to initial position...")
        
        try:
            # 设置初始位置 - 无方向映射
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()
            
            # 发送几次命令以确保移动
            for i in range(10):
                self.send_command()
                time.sleep(0.1)
                
            logger.info("✅ Robot returned to initial position")
        except Exception as e:
            logger.error(f"Error returning to initial position: {e}")
    
    def disable_torque(self, arm: str = None):
        """禁用机器人关节扭矩。

        Args:
            arm: 'left'、'right' 或 None 表示两个机械臂
        """
        if not self.is_connected:
            return

        try:
            if arm is None or arm == "left":
                if self.left_robot and self.left_arm_connected:
                    logger.info("Disabling torque on LEFT arm...")
                    self.left_robot.bus.disable_torque()

            if arm is None or arm == "right":
                if self.right_robot and self.right_arm_connected:
                    logger.info("Disabling torque on RIGHT arm...")
                    self.right_robot.bus.disable_torque()

        except Exception as e:
            logger.error(f"Error disabling torque: {e}")
    
    def disconnect(self):
        """断开与机器人硬件的连接。"""
        if not self.is_connected:
            return
        
        logger.info("Disconnecting from robot...")
        
        # 如果已接合,返回初始位置
        if self.is_engaged:
            try:
                self.return_to_initial_position()
            except Exception as e:
                logger.error(f"Error returning to initial position: {e}")
        
        # 断开两个机械臂
        if self.left_robot:
            try:
                self.left_robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting left arm: {e}")
            self.left_robot = None
            
        if self.right_robot:
            try:
                self.right_robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting right arm: {e}")
            self.right_robot = None
        
        self.is_connected = False
        self.is_engaged = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("🔌 Robot disconnected")
    
    def get_arm_connection_status(self, arm: str) -> bool:
        """基于设备文件存在性获取特定机械臂的连接状态。"""
        # 仅检查设备文件存在性 - 忽略整体机器人连接状态
        if arm == "left":
            device_path = self.config.follower_ports["left"]
            return os.path.exists(device_path)
        elif arm == "right":
            device_path = self.config.follower_ports["right"] 
            return os.path.exists(device_path)
        else:
            return False

    def update_arm_connection_status(self):
        """基于设备文件存在性更新各个机械臂的连接状态。"""
        if self.is_connected:
            self.left_arm_connected = os.path.exists(self.config.follower_ports["left"])
            self.right_arm_connected = os.path.exists(self.config.follower_ports["right"])
    
    @property
    def status(self) -> Dict:
        """获取机器人状态信息。"""
        return {
            "connected": self.is_connected,
            "left_arm_connected": self.left_arm_connected,
            "right_arm_connected": self.right_arm_connected,
            "left_arm_angles": self.left_arm_angles.tolist(),
            "right_arm_angles": self.right_arm_angles.tolist(),
            "joint_limits_min": self.joint_limits_min_deg.tolist(),
            "joint_limits_max": self.joint_limits_max_deg.tolist(),
        } 