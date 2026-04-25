"""
SO100 遥操作系统的机器人接口模块。
提供带安全检查的机器人设备封装和便捷方法。
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
        """使用字典格式向机器人发送当前关节角度。"""
        if not self.is_connected or not self.is_engaged:
            return False
        
        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # 不要发送太频繁
        
        try:
            # 使用字典格式发送指令 - 无关节方向映射
            success = True
            
            # 发送左臂指令
            if self.left_robot and self.left_arm_connected:
                try:
                    action_dict = {
                        "shoulder_pan.pos": float(self.left_arm_angles[0]),
                        "shoulder_lift.pos": float(self.left_arm_angles[1]),
                        "elbow_flex.pos": float(self.left_arm_angles[2]),
                        "wrist_flex.pos": float(self.left_arm_angles[3]),
                        "wrist_roll.pos": float(self.left_arm_angles[4]),
                        "gripper.pos": float(self.left_arm_angles[5])
                    }
                    self.left_robot.send_action(action_dict)
                except Exception as e:
                    logger.error(f"发送左臂指令错误: {e}")
                    self.left_arm_errors += 1
                    if self.left_arm_errors > self.max_arm_errors:
                        self.left_arm_connected = False
                        logger.error("❌ 左臂因重复错误而断开")
                    success = False
            
            # 发送右臂指令
            if self.right_robot and self.right_arm_connected:
                try:
                    action_dict = {
                        "shoulder_pan.pos": float(self.right_arm_angles[0]),
                        "shoulder_lift.pos": float(self.right_arm_angles[1]),
                        "elbow_flex.pos": float(self.right_arm_angles[2]),
                        "wrist_flex.pos": float(self.right_arm_angles[3]),
                        "wrist_roll.pos": float(self.right_arm_angles[4]),
                        "gripper.pos": float(self.right_arm_angles[5])
                    }
                    self.right_robot.send_action(action_dict)
                except Exception as e:
                    logger.error(f"发送右臂指令错误: {e}")
                    self.right_arm_errors += 1
                    if self.right_arm_errors > self.max_arm_errors:
                        self.right_arm_connected = False
                        logger.error("❌ 右臂因重复错误而断开")
                    success = False
            
            self.last_send_time = current_time
            return success
            
        except Exception as e:
            logger.error(f"发送机器人指令错误: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("❌ 机器人接口因重复错误而断开")
            return False
    
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
                self.send_command()
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