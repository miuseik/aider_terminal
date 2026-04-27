"""
遥操作系统的主控制循环。

工作流程:
1. 从命令队列接收 VR/键盘的控制指令
2. 处理指令并更新机械臂目标位置
3. 通过逆运动学(IK)计算关节角度
4. 发送指令到真机或仿真环境
5. 更新 PyBullet 可视化
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
from typing import Dict, Optional

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
from controller.motor_controller import MotorController
from router.api_router import APICommandRouter
# PyBulletVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
from .core.wheels import body_to_wheel_raw
# WebKeyboardHandler will be imported on demand to avoid circular imports

logger = logging.getLogger(__name__)


class ArmState:
    """单个机械臂的状态跟踪。
    
    作用: 记录每个机械臂(左/右)的当前控制状态,
    包括控制模式、目标位置、腕部角度等。
    """
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name  # 'left' 或 'right'
        self.mode = ControlMode.IDLE  # 控制模式: IDLE(空闲) 或 POSITION_CONTROL(位置控制)
        self.target_position = None  # IK 解算的目标位置 [x, y, z]
        self.goal_position = None  # 用于可视化的目标点
        self.origin_position = None  # 握把激活时的初始位置(相对位移的基准点)
        self.origin_wrist_roll_angle = 0.0  # 握把激活时的腕部翻滚角
        self.origin_wrist_flex_angle = 0.0  # 握把激活时的腕部弯曲角
        self.current_wrist_roll = 0.0  # 当前腕部翻滚角
        self.current_wrist_flex = 0.0  # 当前腕部弯曲角
        
    def reset(self):
        """重置机械臂状态为空闲。"""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0


class ControlLoop:
    """处理命令队列并控制机器人的主控制循环。
    
    核心职责:
    - 从 VR/键盘接收控制指令
    - 解算逆运动学(IK)得到关节角度
    - 控制底盘移动和升降轴
    - 发送指令到真机或仿真环境
    - 更新 PyBullet 可视化
    """
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None):
        """初始化控制循环。
        
        Args:
            command_queue: 异步命令队列,接收 VR/键盘的控制指令
            config: 系统配置对象
            control_commands_queue: 可选的线程安全队列(暂未使用)
        """
        self.command_queue = command_queue  # 主命令队列
        self.control_commands_queue = control_commands_queue
        self.config = config  # 系统配置
        
        # === 核心组件 ===
        self.robot_interface = None  # 机器人硬件接口(连接真机)
        self.motor_controller = None  # 电机控制器
        self.api_router = None  # API命令路由器
        self.visualizer = None  # PyBullet 可视化器(仿真环境)
        self.web_keyboard_handler = None  # Web 键盘处理器引用
        
        # === 机械臂状态 ===
        self.left_arm = ArmState("left")   # 左臂状态
        self.right_arm = ArmState("right") # 右臂状态
        
        # === VR 原始数据存储 (用于底盘和升降轴控制) ===
        # 存储左右手摇杆数据和扳机值,每帧读取并转换为底盘速度
        self.vr_raw_data = {
            'leftController': {'joystick': {'x': 0, 'y': 0}, 'trigger': None},
            'rightController': {'joystick': {'x': 0, 'y': 0}, 'trigger': None}
        }
        
        # === Aloha 移动底盘状态 ===
        self.base_velocity_target = {"x": 0.0, "y": 0.0, "theta": 0.0}  # 底盘目标速度: x(前后), y(左右), theta(旋转)
        self.aloha_height = self.config.aloha_initial_height if hasattr(self.config, 'aloha_initial_height') else 0.3  # 升降轴高度(米)
        
        # === 控制时序 ===
        self.last_log_time = 0
        self.log_interval = 1.0  # 每秒记录一次状态日志
        
        # === 调试标志 ===
        self._queue_debug_logged = False
        self._process_debug_logged = False
        
        self.is_running = False  # 控制循环运行标志
    
    def setup(self) -> bool:
        """设置机器人接口和可视化器。
        
        作用: 初始化所有硬件和仿真组件
        - 连接真机机器人(如果启用)
        - 启动 PyBullet 仿真环境
        - 初始化运动学解算器(IK/FK)
        - 设置 Aloha 底盘初始高度
        
        Returns:
            bool: 是否成功设置
        """
        success = True
        setup_errors = []
        
        # === 1. 设置机器人接口(连接真机) ===
        try:
            self.robot_interface = RobotInterface(self.config)
            if not self.robot_interface.connect():
                # 真机连接失败
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                # 如果配置中启用了真机机器人,连接失败则标记setup失败
                # 如果只使用仿真(enable_robot=False),连接失败不影响setup
                if self.config.enable_robot:
                    success = False
            else:
                # 真机连接成功,初始化电机控制器和API路由器
                # 将robot_interface传入MotorController,使其能够:
                # 1. 访问机械臂角度数组(left/right_arm_angles)
                # 2. 调用send_action()发送指令到真机
                # 3. 读取实际关节角度(get_actual_arm_angles)
                # 初始化电机控制器
                self.motor_controller = MotorController(self.robot_interface)
                logger.info("✅ Motor controller initialized")
                
                # 初始化API命令路由器
                self.api_router = APICommandRouter(
                    motor_controller=self.motor_controller
                )
                logger.info("✅ API command router initialized")
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False
        
        # === 2. 设置 PyBullet 仿真、IK 和可视化器 ===
        if self.config.enable_pybullet:
            try:
                # 按需导入 PyBulletVisualizer
                from .core.visualizer import PyBulletVisualizer
                
                # 创建可视化器实例
                self.visualizer = PyBulletVisualizer(
                    self.config.get_absolute_urdf_path(),  # SO100 URDF 文件路径
                    use_gui=self.config.enable_pybullet_gui,  # 是否显示 GUI 窗口
                    log_level=self.config.log_level
                )
                
                # 启动 PyBullet 仿真环境
                if not self.visualizer.setup():
                    error_msg = "PyBullet visualizer setup failed"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                else:
                    # 将运动学解算器连接到机器人接口
                    # FK(正运动学): 关节角度 → 末端位置
                    # IK(逆运动学): 末端位置 → 关节角度
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,      # PyBullet 物理引擎客户端
                        self.visualizer.robot_ids,           # 两个机器人实例 ID (左/右)
                        self.visualizer.joint_indices,       # 两个关节索引映射
                        self.visualizer.end_effector_link_indices,  # 两个末端执行器索引
                        joint_limits_min,   # 关节最小限位
                        joint_limits_max    # 关节最大限位
                    )
                    
                    # 如果启用了 Aloha,设置初始高度
                    if self.config.aloha_enabled and self.visualizer.aloha_id is not None:
                        self.visualizer.set_aloha_height(self.config.aloha_initial_height)
                        logger.info(f"Aloha chassis initialized at height: {self.config.aloha_initial_height}m")
            except Exception as e:
                error_msg = f"PyBullet visualizer setup failed with exception: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
        
        # 报告所有设置问题
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
        # 在 Web 键盘处理器上设置机器人接口,使其能获取当前位置
        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on web keyboard handler")

        return success
    
    async def start(self):
        """启动控制循环。
        
        工作流程(每帧执行):
        1. 从命令队列读取 VR/键盘指令
        2. 更新机械臂目标位置和腕部角度
        3. 处理底盘和升降轴控制
        4. 通过 IK 解算关节角度
        5. 发送指令到真机或仿真
        6. 更新 PyBullet 可视化
        """
        if not self.setup():
            logger.error("Control loop setup failed")
            return
        
        self.is_running = True
        logger.info("Control loop started")
        
        # 用当前机器人位置初始化机械臂状态
        self._initialize_arm_states()
        
        # === 主控制循环 (每帧执行) ===
        while self.is_running:
            try:
                # 步骤 1: 处理命令队列(接收 VR/键盘指令)
                await self._process_commands()
                
                # 步骤 2-5: 更新机器人(解算 IK + 发送指令)
                self._update_robot_safely()
                
                # 步骤 6: 更新 PyBullet 可视化
                if self.visualizer:
                    self._update_visualization()
                
                # 定期打印状态日志
                self._periodic_logging()
                
                # 控制频率 (默认 50Hz,即每 0.02 秒一帧)
                await asyncio.sleep(self.config.send_interval)
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(0.1)  # 出错后短暂暂停
        
        logger.info("Control loop stopped")
    
    async def stop(self):
        """停止控制循环。"""
        self.is_running = False

        # 清理 - 先断开机器人 (返回 home 位置并禁用力矩)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                logger.info("🛑 Disengaging robot before shutdown...")
                self.robot_interface.disengage()
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()
    
    def _initialize_arm_states(self):
        """用当前机器人位置初始化机械臂状态。"""
        if self.robot_interface:
            # 获取当前末端执行器位置
            left_pos = self.robot_interface.get_current_end_effector_position("left")
            right_pos = self.robot_interface.get_current_end_effector_position("right")
            
            # 将目标位置初始化为当前位置(确保深拷贝)
            self.left_arm.target_position = left_pos.copy()
            self.left_arm.goal_position = left_pos.copy()
            self.right_arm.target_position = right_pos.copy()
            self.right_arm.goal_position = right_pos.copy()
            
            # 获取当前腕部旋转角度
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
            
            self.left_arm.current_wrist_roll = left_angles[WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[WRIST_ROLL_INDEX]
            
            self.left_arm.current_wrist_flex = left_angles[WRIST_FLEX_INDEX]
            self.right_arm.current_wrist_flex = right_angles[WRIST_FLEX_INDEX]
            
            logger.info(f"Initialized left arm at position: {left_pos.round(3)}")
            logger.info(f"Initialized right arm at position: {right_pos.round(3)}")
    
    async def _process_commands(self):
        """处理命令队列中的命令。"""
        try:
            # 处理常规控制目标
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """处理单个命令。"""
        action = command.get('action', '')
        logger.info(f"🔌 Processing control command: {action}")
        
        if action == 'enable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.start()
                logger.info("🎮 Keyboard control ENABLED via API")
        elif action == 'disable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.stop()
                logger.info("🎮 Keyboard control DISABLED via API")
        elif action == 'web_keypress':
            # Handle individual keypress events from web interface
            key = command.get('key')
            event = command.get('event')  # 'press' or 'release'

            if self.web_keyboard_handler and self.web_keyboard_handler.is_enabled:
                logger.debug(f"🌐 Processing web keypress: {key}_{event}")
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
            else:
                logger.warning("🎮 Web keyboard handler not enabled")
        elif action == 'robot_connect':
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # 无需同步键盘目标 - 统一系统自动处理
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # 机器人断开时将机械臂状态重置为 IDLE
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 Both arms: Position control DEACTIVATED after robot disconnect")
                    
                    # 隐藏可视化标记点
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        elif action.startswith('control_') or action == 'calibrate_motor':
            # API控制命令,交给路由器处理
            if not self.api_router:
                logger.warning("⚠️ API命令路由器未初始化")
                return
            
            success = self.api_router.route(command)
            if not success:
                logger.error(f"❌ 处理API命令失败: {action}")
        else:
            logger.warning(f"Unknown command: {action}")

    async def _execute_goal(self, goal: ControlGoal):
        """执行控制目标。"""
        
        # 1. 优先处理 Aloha 底盘的特殊控制指令
        if goal.metadata and goal.metadata.get("action") == "set_aloha_height":
            height_delta = goal.metadata.get("height_delta", 0)
            self.aloha_height = max(0.0, min(0.7854, self.aloha_height + height_delta))
            if self.visualizer:
                self.visualizer.set_aloha_height(self.aloha_height)
                logger.debug(f"⬆️ Aloha lift adjustment: {self.aloha_height:.3f}m (delta: {height_delta:.3f})")
            return
        
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        
        # 处理来自键盘空闲超时的特殊重置信号
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                # 将目标位置重置为当前机器人位置
                current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                current_angles = self.robot_interface.get_arm_angles(goal.arm)
                
                arm_state.target_position = current_position.copy()
                arm_state.goal_position = current_position.copy()
                arm_state.origin_position = current_position.copy()
                arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔄 {goal.arm.upper()} arm: Target position reset to current robot position (idle timeout)")
            return
        
        # 处理模式变化(仅在指定模式时)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # 激活位置控制 - 始终将目标重置为当前位置
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                if self.robot_interface:
                    current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    
                    # 将所有内容重置为当前位置(类似 VR 握把按下)
                    arm_state.target_position = current_position.copy()
                    arm_state.goal_position = current_position.copy()
                    arm_state.origin_position = current_position.copy()
                    arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔒 {goal.arm.upper()} grip activated - controlling {goal.arm} arm (target reset to current position)")
                
            elif goal.mode == ControlMode.IDLE:
                # 停用位置控制
                arm_state.reset()
                
                # 隐藏可视化标记点
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
        
        # 处理位置控制 - VR 和键盘现在工作方式相同(相对于原点的绝对偏移)
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            if goal.metadata and goal.metadata.get("relative_position", False):
                # VR 和键盘都发送相对于机器人原点位置的绝对偏移
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + goal.target_position
                    arm_state.goal_position = arm_state.target_position.copy()
                else:
                    # 尚未设置原点,使用当前位置作为基准
                    if self.robot_interface:
                        current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                        arm_state.target_position = current_position + goal.target_position
                        arm_state.goal_position = arm_state.target_position.copy()
            else:
                # 绝对位置(遗留 - 不应再使用)
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()
            
            # 处理腕部运动 - VR 和键盘都发送相对于原点的绝对偏移
            if goal.wrist_roll_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Both VR and keyboard send absolute wrist angle relative to origin
                    arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
                else:
                    # 绝对腕部旋转(遗留)
                    arm_state.current_wrist_roll = goal.wrist_roll_deg
            
            # 处理腕部弯曲 - VR 和键盘都发送相对于原点的绝对偏移
            if goal.wrist_flex_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Both VR and keyboard send absolute wrist angle relative to origin
                    arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
                else:
                    # 绝对腕部弯曲(遗留)
                    arm_state.current_wrist_flex = goal.wrist_flex_deg
        
        # 处理夹爪控制(独立于模式)
        if goal.gripper_closed is not None and self.robot_interface:
            # 从 metadata 中提取扳机值(如果有)
            trigger_value = goal.metadata.get("trigger_value") if goal.metadata else None
            
            # 调用 robot_interface.set_gripper() 进行线性映射
            # trigger 0-1 → 角度 90°-0°,直接替换 IK 结果的第6个值
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed, trigger_value)
        
        # 处理底盘控制(通过摇杆) - 只存储摇杆数据,在 _update_robot 中统一处理
        if goal.metadata and goal.metadata.get("base_control", False):
            # Check if it's from keyboard (has velocity fields)
            if "velocity_x" in goal.metadata:
                # Keyboard control: direct velocity values
                self.base_velocity_target["x"] = goal.metadata.get("velocity_x", 0)
                self.base_velocity_target["y"] = goal.metadata.get("velocity_y", 0)
                self.base_velocity_target["theta"] = goal.metadata.get("velocity_theta", 0)
                print(f"🎮 Base control received: x={self.base_velocity_target['x']}, y={self.base_velocity_target['y']}, theta={self.base_velocity_target['theta']}")
            else:
                # VR joystick control: store joystick data
                hand = goal.metadata.get("hand", "left")
                joystick_x = goal.metadata.get("joystick_x", 0)
                joystick_y = goal.metadata.get("joystick_y", 0)
                trigger_value = goal.metadata.get("trigger_value")
                
                # 存储摇杆数据到 vr_raw_data
                controller_key = f"{hand}Controller"
                if controller_key in self.vr_raw_data:
                    self.vr_raw_data[controller_key]['joystick'] = {'x': joystick_x, 'y': joystick_y}
                    if trigger_value is not None:
                        self.vr_raw_data[controller_key]['trigger'] = trigger_value
    
    def _update_mobile_base(self, vr_data: dict):
        """根据 VR 摇杆数据更新移动底盘（轮子）和升降轴的状态。"""
        # 1. 提取真实的摇杆数据
        left_joy = vr_data.get("leftController", {}).get("joystick", {"x": 0, "y": 0})
        right_joy = vr_data.get("rightController", {}).get("joystick", {"x": 0, "y": 0})
        
        lx, ly = left_joy.get("x", 0), left_joy.get("y", 0)
        rx, ry = right_joy.get("x", 0), right_joy.get("y", 0)

        # 2. 设置死区 (Deadzone)，防止摇杆漂移导致底盘微动
        DEADZONE = 0.1
        def apply_deadzone(val):
            return val if abs(val) > DEADZONE else 0.0

        lx, ly = apply_deadzone(lx), apply_deadzone(ly)
        rx, ry = apply_deadzone(rx), apply_deadzone(ry)

        # 3. 映射到底盘速度 (m/s 和 deg/s)
        MAX_LIN_SPEED = 1.0   # 线速度
        MAX_ANG_SPEED = 1.0   # 角速度
        
        # 左摇杆 Y: 前推(-1)/后推(1) -> 前进/后退
        self.base_velocity_target["x"] = ly * MAX_LIN_SPEED
        
        # 左摇杆 X: 左推(-1)/右推(1) -> 左移/右移
        self.base_velocity_target["y"] = -lx * MAX_LIN_SPEED
        
        # 右摇杆 X: 左推(-1)/右推(1) -> 左转/右转
        self.base_velocity_target["theta"] = -rx * MAX_ANG_SPEED

        # 4. 处理升降轴高度 (使用右摇杆 Y 轴作为增量控制)
        if abs(ry) > DEADZONE:
            delta_h = -ry * 0.005  # 负号修正方向，0.005 提高平滑度
            new_height_mm = max(0.0, min(1.0, self.aloha_height + delta_h))
            self.aloha_height = new_height_mm

    def _build_alohamini_action(self) -> dict:
        """构造发送给 LeRobot/AlohaMini 的完整 Action 字典。"""
        action = {}
        
        # 1. 机械臂部分 (从 robot_interface 获取最新的 IK 结果)
        if self.robot_interface:
            for arm in ["left", "right"]:
                angles = self.robot_interface.get_arm_angles(arm)
                for i, name in enumerate(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]):
                    action[f"{arm}_arm.{name}.pos"] = angles[i]

        # 2. 底盘部分 (调用运动学逆解)
        wheel_speeds = body_to_wheel_raw(
            self.base_velocity_target["x"],
            self.base_velocity_target["y"],
            self.base_velocity_target["theta"]
        )
        action["base.left_wheel.vel"] = wheel_speeds["base_left_wheel"]
        action["base.back_wheel.vel"] = wheel_speeds["base_back_wheel"]
        action["base.right_wheel.vel"] = wheel_speeds["base_right_wheel"]

        # 3. 升降轴部分 (注意：aloha_height 内部单位是米，Action 需要毫米)
        height_mm = int(self.aloha_height * 1000)
        action["lift.height_mm"] = height_mm
        
        return action
    
    def _update_robot_safely(self):
        """用当前控制目标更新机器人(带错误处理)。"""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # 不关闭,继续运行 - 机器人接口会处理连接问题
    
    def _update_robot(self):
        """用当前控制目标更新机器人。"""
        if not self.robot_interface:
            return
        
        # 1. 获取最新的 VR 数据 (从共享存储中读取)
        vr_data = self.vr_raw_data
        
        # 2. 更新移动底盘和升降轴
        # 检查是否有键盘底盘控制，如果有则跳过 VR 摇杆处理
        has_keyboard_base_control = (
            self.web_keyboard_handler and 
            self.web_keyboard_handler.base_state.get("base_control_active", False)
        )
        
        if not has_keyboard_base_control:
            # 只有在没有键盘控制时才处理 VR 摇杆
            self._update_mobile_base(vr_data)
        else:
            print(f"🎮 Using keyboard base control, skipping VR joystick")

        # 4. 更新左臂(仅在连接或纯仿真模式下)
        if (self.left_arm.mode == ControlMode.POSITION_CONTROL and 
            self.left_arm.target_position is not None):
            # 检查机械臂是否连接或处于无机器人模式
            arm_connected = self.robot_interface.get_arm_connection_status("left")
            should_update = arm_connected or not self.config.enable_robot
            
            if should_update:
                # 求解 IK
                ik_solution = self.robot_interface.solve_ik("left", self.left_arm.target_position)
                
                # Update robot angles
                current_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
                self.robot_interface.update_arm_angles("left", ik_solution, 
                                                     self.left_arm.current_wrist_flex, 
                                                     self.left_arm.current_wrist_roll, 
                                                     current_gripper)
                
                # 【夹爪线性控制】替换第6个关节角度
                left_trigger = self.vr_raw_data.get('leftController', {}).get('trigger', None)
                if left_trigger is not None:
                    self.robot_interface.left_arm_angles[GRIPPER_INDEX] = -left_trigger * 90.0
            else:
                logger.debug(f"Skipping left arm update: connected={arm_connected}, enable_robot={self.config.enable_robot}")

        # 更新右臂(仅在连接或纯仿真模式下)
        if (self.right_arm.mode == ControlMode.POSITION_CONTROL and 
            self.right_arm.target_position is not None):
            # 检查机械臂是否连接或处于无机器人模式
            arm_connected = self.robot_interface.get_arm_connection_status("right")
            should_update = arm_connected or not self.config.enable_robot
            
            if should_update:
                # 求解 IK
                ik_solution = self.robot_interface.solve_ik("right", self.right_arm.target_position)
                
                # Update robot angles
                current_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
                self.robot_interface.update_arm_angles("right", ik_solution, 
                                                      self.right_arm.current_wrist_flex, 
                                                      self.right_arm.current_wrist_roll, 
                                                      current_gripper)
                
                # 【夹爪线性控制】替换第6个关节角度
                right_trigger = self.vr_raw_data.get('rightController', {}).get('trigger', None)
                if right_trigger is not None:
                    self.robot_interface.right_arm_angles[GRIPPER_INDEX] = -right_trigger * 90.0
            else:
                logger.debug(f"Skipping right arm update: connected={arm_connected}, enable_robot={self.config.enable_robot}")


        # === 发送指令到真机 ===
        # send_command()负责将robot_interface的角度数组发送到真机硬件
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command()
        
        # === 同步状态到 robot_interface (用于 status 接口) ===
        if self.robot_interface:
            # 更新底盘状态
            self.robot_interface.base_velocity_target = {
                "x": self.base_velocity_target["x"],
                "y": self.base_velocity_target["y"],
                "theta": self.base_velocity_target["theta"]
            }
            
            # 更新升降轴状态 (aloha_height 单位是米,转换为毫米)
            self.robot_interface.lift_height_mm = int(self.aloha_height * 1000)
    
    def _update_visualization(self):
        """更新 PyBullet 可视化。"""
        if not self.visualizer:
            return
        
        # 1. 更新仿真中的底盘位置
        if self.config.aloha_enabled:
            sim_action = {
                "lift.height_mm": int(self.aloha_height * 1000),
                "base.vx": self.base_velocity_target["x"],
                "base.vy": self.base_velocity_target["y"],
                "base.vtheta": self.base_velocity_target["theta"],
            }
            self.visualizer.update_mobile_base_simulation(sim_action)
        
        # 2. 使用机器人硬件的实际角度更新两个机械臂的姿态
        # 在无机器人模式下,get_arm_angles 返回仿真角度
        left_angles = self.robot_interface.get_actual_arm_angles("left")
        right_angles = self.robot_interface.get_actual_arm_angles("right")
        
        # 【夹爪线性控制】从 VR 数据中提取 trigger 值,替换夹爪角度
        # trigger: 0.0 → -90° (完全打开), 1.0 → 0° (完全闭合)
        left_trigger = self.vr_raw_data.get('leftController', {}).get('trigger', None)
        right_trigger = self.vr_raw_data.get('rightController', {}).get('trigger', None)
        
        if left_trigger is not None and len(left_angles) > GRIPPER_INDEX:
            left_angles[GRIPPER_INDEX] = -left_trigger * 90.0
        
        if right_trigger is not None and len(right_angles) > GRIPPER_INDEX:
            right_angles[GRIPPER_INDEX] = -right_trigger * 90.0
        
        # 更新 SO100 机器人姿态
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
        # 如果启用了 Aloha,将 SO100 IK 结果映射到 Aloha 双臂
        if self.config.aloha_enabled and self.visualizer.aloha_id is not None:
            self.visualizer.update_aloha_arm_pose(left_angles, 'left')
            self.visualizer.update_aloha_arm_pose(right_angles, 'right')
        
        # 更新可视化标记点
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if self.left_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if self.right_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # 推进仿真
        self.visualizer.step_simulation()
    
    def _periodic_logging(self):
        """定期记录状态信息。"""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            
            active_arms = []
            if self.left_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("LEFT")
            if self.right_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("RIGHT")
            
            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}")
    
    @property
    def status(self) -> Dict:
        """获取当前控制循环状态。"""
        # 基础状态
        status = {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        }
        
        # 合并机器人硬件详细状态
        if self.robot_interface:
            status.update({
                # 连接状态
                "robot_connected": self.robot_interface.is_connected,
                "left_arm_connected": self.robot_interface.left_arm_connected,
                "right_arm_connected": self.robot_interface.right_arm_connected,
                
                # 机械臂角度
                "left_arm_angles": self.robot_interface.left_arm_angles.tolist(),
                "right_arm_angles": self.robot_interface.right_arm_angles.tolist(),
                "joint_limits_min": self.robot_interface.joint_limits_min_deg.tolist(),
                "joint_limits_max": self.robot_interface.joint_limits_max_deg.tolist(),
                
                # 底盘状态
                "base_connected": self.robot_interface.base_connected,
                "base_velocity_target": self.robot_interface.base_velocity_target,
                
                # 升降轴状态
                "lift_connected": self.robot_interface.lift_connected,
                "lift_height_mm": self.robot_interface.lift_height_mm
            })
        else:
            # 机器人未初始化时的默认值
            status.update({
                "robot_connected": False,
                "left_arm_connected": False,
                "right_arm_connected": False,
                "base_connected": False,
                "lift_connected": False
            })
        
        return status 
