"""
遥操作系统的主控制循环。
负责从命令队列中读取控制目标，并通过机器人接口驱动硬件执行。
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
from typing import Dict, Optional

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX, get_config_data
from .core.robot_interface import RobotInterface
from .robots.robot_controller import RobotController  # 第1行：引入robot模块
from .core.wheels import body_to_wheel_raw
from .core.axis import height_mm_to_ticks, clamp_height
# PyBulletVisualizer 将按需导入，避免启动时加载过重的物理引擎
from .inputs.base import ControlGoal, ControlMode
# WebKeyboardHandler 将按需导入，以防止模块间的循环引用

logger = logging.getLogger(__name__)


class ArmState:
    """用于跟踪单个机械臂的运行状态（如：当前模式、目标位置、原点等）。"""
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None  # 期望到达的目标位置（主要用于可视化显示）
        self.origin_position = None  # 激活位置控制时的初始参考点（类似 VR 的握持原点）
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        
    def reset(self):
        """将机械臂状态重置为空闲（IDLE），清空所有临时坐标。"""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0


class ControlLoop:
    """核心控制循环类：协调输入处理、运动学解算和硬件指令下发。"""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config
        
        # --- 核心组件初始化 ---
        self.robot_interface = None      # 机器人底层通信接口
        self.robot_controller = None     # 真机舵机控制器
        self.visualizer = None           # PyBullet 仿真可视化器
        self.web_keyboard_handler = None # Web 端键盘输入处理器
        
        # --- VR 原始数据存储 (用于底盘和升降轴控制) ---
        self.vr_raw_data = {} 
        
        # --- 双臂状态管理 ---
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # --- Aloha 移动底盘状态 ---
        self.aloha_height = self.config.aloha_initial_height # 当前升降轴高度（米）
        self.base_velocity_target = {"x": 0.0, "y": 0.0, "theta": 0.0} # 底盘线速度和角速度目标
        
        # --- 性能与日志控制 ---
        self.last_log_time = 0
        self.log_interval = 1.0  # 状态日志输出频率：每秒 1 次
        
        # --- 调试标记 ---
        self._queue_debug_logged = False
        self._process_debug_logged = False

        self.is_running = False
    
    def setup(self) -> bool:
        """系统初始化：建立机器人连接并启动仿真环境。"""
        success = True
        setup_errors = []
        
        # 1. 初始化机器人硬件接口
        try:
            self.robot_interface = RobotInterface(self.config)
            if not self.robot_interface.connect():
                error_msg = "机器人接口连接失败，请检查串口权限或设备占用情况"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
            
            # 初始化真机舵机控制器（自动从配置加载所有设备）
            config_data = get_config_data()
            self.robot_controller = RobotController(config_data)
            
        except Exception as e:
            error_msg = f"机器人接口初始化异常: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False
        
        # 2. 初始化 PyBullet 仿真与逆运动学 (IK) 引擎
        if self.config.enable_pybullet:
            try:
                from .sim.sim2real.visualizer import PyBulletVisualizer
                
                self.visualizer = PyBulletVisualizer(
                    self.config.get_absolute_urdf_path(), 
                    use_gui=self.config.enable_pybullet_gui,
                    log_level=self.config.log_level
                )
                if not self.visualizer.setup():
                    error_msg = "PyBullet 仿真环境搭建失败"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                else:
                    # 将仿真环境的关节限制信息同步给机器人接口，用于 IK 解算
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,
                        self.visualizer.robot_ids,       # 左右臂的物理实体 ID
                        self.visualizer.joint_indices,   # 关节名称到索引的映射表
                        self.visualizer.end_effector_link_indices, # 末端执行器（夹爪尖端）索引
                        joint_limits_min,
                        joint_limits_max
                    )
            except Exception as e:
                error_msg = f"PyBullet 可视化器初始化异常: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
        
        # 汇总并报告所有初始化阶段的错误
        if setup_errors:
            logger.error("系统初始化失败，存在以下阻碍项:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
        # 将机器人接口句柄传递给 Web 键盘处理器，使其能实时获取机械臂当前位置以设置原点
        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
            logger.info("已成功向 Web 键盘处理器注入机器人接口引用")

        return success
    
    async def start(self):
        """启动控制循环。"""
        if not self.setup():
            logger.error("控制循环设置失败")
            return
        
        self.is_running = True
        logger.info("控制循环已启动")
        
        # 用当前机器人位置初始化机械臂状态
        self._initialize_arm_states()
        
        # 主控制循环
        while self.is_running:
            try:
                # 处理命令队列
                await self._process_commands()
                
                # 更新机器人 (带错误恢复)
                self._update_robot_safely()
                
                # 更新可视化
                if self.visualizer:
                    self._update_visualization()
                
                # 定期日志记录
                self._periodic_logging()
                
                # 控制频率
                await asyncio.sleep(self.config.send_interval)
                
            except Exception as e:
                logger.error(f"控制循环出错: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("控制循环已停止")
    
    async def stop(self):
        """停止控制循环。"""
        self.is_running = False

        # 清理 - 先断开机器人 (返回 home 位置并禁用力矩)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                logger.info("🛑 关闭前断开机器人...")
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
            
            # 将目标位置初始化为当前位置 (确保深拷贝)
            self.left_arm.target_position = left_pos.copy()
            self.left_arm.goal_position = left_pos.copy()
            self.right_arm.target_position = right_pos.copy()
            self.right_arm.goal_position = right_pos.copy()
            
            # 获取当前手腕旋转角度
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
            
            self.left_arm.current_wrist_roll = left_angles[WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[WRIST_ROLL_INDEX]
            
            self.left_arm.current_wrist_flex = left_angles[WRIST_FLEX_INDEX]
            self.right_arm.current_wrist_flex = right_angles[WRIST_FLEX_INDEX]
            
            logger.info(f"左臂初始化位置: {left_pos.round(3)}")
            logger.info(f"右臂初始化位置: {right_pos.round(3)}")
    
    async def _process_commands(self):
        """处理命令队列中的命令。"""
        try:
            # 处理常规控制目标
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"处理命令时出错: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """处理来自 Web 端或控制台的单个管理指令。"""
        action = command.get('action', '')
        logger.info(f"🔌 收到管理指令: {action}")
        
        if action == 'enable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.start()
                logger.info("🎮 键盘控制已通过 API 启用")
        elif action == 'disable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.stop()
                logger.info("🎮 键盘控制已通过 API 停用")
        elif action == 'web_keypress':
            # 处理来自 Web 界面的单独按键事件
            key = command.get('key')
            event = command.get('event')  # 'press' 或 'release'

            if self.web_keyboard_handler and self.web_keyboard_handler.is_enabled:
                logger.debug(f"🌐 处理 Web 按键: {key}_{event}")
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
            else:
                logger.warning("🎮 Web 键盘处理器未启用")
        elif action == 'set_aloha_height':
            # 处理 Aloha 移动底盘的升降高度调整
            height = command.get('height', self.aloha_height)
            self.aloha_height = height
            if self.visualizer:
                self.visualizer.set_aloha_height(height)
                logger.info(f"⬆️ Aloha 底盘高度已更新为 {height:.3f}m")
        elif action == 'robot_connect':
            logger.info("🔌 处理 robot_connect 命令")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 机器人接口可用且已连接: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 机器人电机已成功接合 (Engaged)")
                    # 统一系统会自动同步键盘目标，无需手动干预
                else:
                    logger.error("❌ 机器人电机接合失败，请检查硬件连接")
            else:
                logger.warning(f"无法接合机器人: 接口存在={self.robot_interface is not None}, 连接状态={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 处理 robot_disconnect 命令")
            if self.robot_interface:
                logger.info(f"🔌 机器人接口可用")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 机器人电机已成功断开 (Disengaged)")
                    # 断开时强制重置双臂状态为 IDLE，防止意外运动
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 双臂: 机器人断开后位置控制已停用")
                    
                    # 隐藏可视化标记
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ 机器人电机关闭失败")
            else:
                logger.warning("无法执行断开操作：机器人接口未初始化")
        else:
            logger.warning(f"收到未知的管理指令: {action}")

    async def _execute_goal(self, goal: ControlGoal):
        """解析并执行具体的运动控制目标（VR 或键盘输入）。"""
        
        # 【核心修改】提取摇杆数据并存入共享状态
        if goal.left_joystick is not None:
            self.vr_raw_data['leftController'] = self.vr_raw_data.get('leftController', {})
            self.vr_raw_data['leftController']['joystick'] = goal.left_joystick
        if goal.right_joystick is not None:
            self.vr_raw_data['rightController'] = self.vr_raw_data.get('rightController', {})
            self.vr_raw_data['rightController']['joystick'] = goal.right_joystick
        
        # 【秘密武器】存储 trigger 线性值
        if goal.metadata and 'trigger_value' in goal.metadata:
            arm = goal.arm
            controller_key = f'{arm}Controller'
            self.vr_raw_data[controller_key] = self.vr_raw_data.get(controller_key, {})
            self.vr_raw_data[controller_key]['trigger'] = goal.metadata['trigger_value']

        # 1. 优先处理 Aloha 底盘的特殊控制指令
        if goal.metadata and goal.metadata.get("action") == "set_aloha_height":
            height_delta = goal.metadata.get("height_delta", 0)
            self.aloha_height = max(0.0, min(0.7854, self.aloha_height + height_delta))
            if self.visualizer:
                self.visualizer.set_aloha_height(self.aloha_height)
                logger.debug(f"⬆️ Aloha 底盘微调: {self.aloha_height:.3f}m (变化量: {height_delta:.3f})")
            return
        
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        
        # 2. 处理键盘空闲超时后的原点重置逻辑
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface:
                # 将目标点拉回当前物理位置，实现“软复位”
                current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                current_angles = self.robot_interface.get_arm_angles(goal.arm)
                                
                print(f"\n🔴 [BEFORE RESET] {goal.arm.upper()} 臂:")
                print(f"   握把按下前 - goal_position: {arm_state.goal_position}")
                print(f"   握把按下前 - origin_position: {arm_state.origin_position}")
                print(f"   当前机械臂位置: {current_position}")
                        
                arm_state.target_position = current_position.copy()
                arm_state.goal_position = current_position.copy()
                arm_state.origin_position = current_position.copy()
                arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                        
                print(f"   ✅ 重置后 - goal_position: {arm_state.goal_position}")
                print(f"   ✅ 重置后 - origin_position: {arm_state.origin_position}\n")
                                
                logger.info(f"🔄 {goal.arm.upper()} 臂: 因空闲超时，目标点已重置到当前位置")
            # 不要 return，继续执行后续的 mode 切换逻辑
        
        # 3. 处理机械臂模式切换（如：从 IDLE 切换到 POSITION_CONTROL）
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # 激活位置控制：必须以当前物理位置为新原点，防止跳变
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                if self.robot_interface:
                    current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    
                    # 记录所有参考坐标（原点、手腕角度等），确保 VR 握持或键盘激活瞬间的平滑过渡
                    arm_state.target_position = current_position.copy()
                    arm_state.goal_position = current_position.copy()
                    arm_state.origin_position = current_position.copy()
                    arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                
                logger.info(f"🔒 {goal.arm.upper()} 臂: 位置控制已激活 (原点已锁定在当前位置)")
                
            elif goal.mode == ControlMode.IDLE:
                # 停用位置控制：释放机械臂进入自由状态
                arm_state.reset()
                
                # 隐藏可视化标记
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} 臂: 位置控制已停用，进入空闲状态")
        
        # 4. 处理位置偏移计算（支持 VR 和键盘的相对位移逻辑）
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            if goal.metadata and goal.metadata.get("relative_position", False):
                # 核心逻辑：目标位置 = 初始原点 + 累计偏移量
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + goal.target_position
                    arm_state.goal_position = arm_state.target_position.copy()
                    
                    print(f"🟡 [{goal.arm.upper()}] VR 相对位移控制:")
                    print(f"   VR 传来的 relative_delta: {goal.target_position}")
                    print(f"   origin_position: {arm_state.origin_position}")
                    print(f"   计算后 goal_position: {arm_state.goal_position}\n")
                else:
                    # 如果尚未设置原点（极少情况），则以当前位置为基准进行累加
                    if self.robot_interface:
                        current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                        arm_state.target_position = current_position + goal.target_position
                        arm_state.goal_position = arm_state.target_position.copy()
                        
                        print(f"⚠️  [{goal.arm.upper()}] origin_position 为空，使用当前位置作为基准")
                        print(f"   当前位置: {current_position}")
                        print(f"   relative_delta: {goal.target_position}")
                        print(f"   计算后 goal_position: {arm_state.goal_position}\n")
            else:
                # 绝对坐标模式（遗留代码，建议不再使用）
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()
            
            # 5. 处理手腕旋转（Wrist Roll）的相对角度累加
            if goal.wrist_roll_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # 核心逻辑：当前角度 = 初始角度 + 偏移角度
                    arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
                else:
                    # 绝对角度赋值（遗留代码）
                    arm_state.current_wrist_roll = goal.wrist_roll_deg
            
            # 6. 处理手腕弯曲（Wrist Flex）的相对角度累加
            if goal.wrist_flex_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # 核心逻辑：当前角度 = 初始角度 + 偏移角度
                    arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
                else:
                    # 绝对角度赋值（遗留代码）
                    arm_state.current_wrist_flex = goal.wrist_flex_deg
        
        # 7. 处理夹爪开合指令（独立于位置控制模式，可随时触发）
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)

    def _update_mobile_base(self, vr_data: dict):
        """根据 VR 摇杆数据更新移动底盘（轮子）和升降轴的状态。"""
        # 1. 提取真实的摇杆数据 (来自 Vue 的 dualControllerData)
        left_joy = vr_data.get("leftController", {}).get("joystick", {"x": 0, "y": 0})
        right_joy = vr_data.get("rightController", {}).get("joystick", {"x": 0, "y": 0})
        
        lx, ly = left_joy.get("x", 0), left_joy.get("y", 0)
        rx, ry = right_joy.get("x", 0), right_joy.get("y", 0)

        # 调试打印：确认函数被调用且有数据
        # print(f"🕹️ [Loop] ry={ry:.2f}, aloha_height={self.aloha_height:.1f}")

        # 2. 设置死区 (Deadzone)，防止摇杆漂移导致底盘微动
        DEADZONE = 0.1
        def apply_deadzone(val):
            return val if abs(val) > DEADZONE else 0.0

        lx, ly = apply_deadzone(lx), apply_deadzone(ly)
        rx, ry = apply_deadzone(rx), apply_deadzone(ry)

        # 3. 映射到底盘速度 (m/s 和 deg/s)
        MAX_LIN_SPEED = 1   # 线速度（提高5倍）
        MAX_ANG_SPEED = 1.0  # 角速度（提高6倍）
        
        # 【测试前后+左右+转向】
        # 左摇杆 Y: 前推(-1)/后推(1) -> 前进/后退
        self.base_velocity_target["x"] = ly * MAX_LIN_SPEED
        
        # 左摇杆 X: 左推(-1)/右推(1) -> 左移/右移
        self.base_velocity_target["y"] = -lx * MAX_LIN_SPEED
        
        # 右摇杆 X: 左推(-1)/右推(1) -> 左转/右转
        self.base_velocity_target["theta"] = -rx * MAX_ANG_SPEED

        # 4. 处理升降轴高度 (使用右摇杆 Y 轴作为增量控制)
        # VR 下推 Y=1 -> 我们希望下降，所以取反 (-ry)
        if abs(ry) > DEADZONE:
            delta_h = -ry * 0.005 # 负号修正方向，0.005 提高平滑度
            new_height_mm = clamp_height(self.aloha_height + delta_h)
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
        # print(f"📏 [BuildAction] aloha_height={self.aloha_height:.3f}m -> {height_mm}mm")
        action["lift.height_mm"] = height_mm
        
        return action
    
    def _update_robot_safely(self):
        """带异常捕获的机器人更新入口，确保单次 IK 失败不会导致整个循环崩溃。"""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"更新机器人底层状态时出错: {e}")
            # 保持循环运行，让底层接口自行处理断连重连
    
    def _update_robot(self):
        """核心更新逻辑：执行 IK 解算并下发关节角度指令。"""
        if not self.robot_interface:
            return
        
        # 0. 处理 Web 键盘的底盘控制
        if self.web_keyboard_handler and self.web_keyboard_handler.is_enabled:
            base_state = self.web_keyboard_handler.base_state
            if base_state["base_control_active"]:
                self.base_velocity_target["x"] = base_state["velocity_x"]
                self.base_velocity_target["y"] = base_state["velocity_y"]
                self.base_velocity_target["theta"] = base_state["velocity_theta"]
        
        # 1. 获取最新的 VR 数据 (从共享存储中读取)
        vr_data = self.vr_raw_data
        
        # 2. 只有在键盘未控制底盘时，才使用 VR 摇杆数据
        keyboard_controlling = (self.web_keyboard_handler and 
                               self.web_keyboard_handler.is_enabled and 
                               self.web_keyboard_handler.base_state["base_control_active"])
        
        if not keyboard_controlling:
            self._update_mobile_base(vr_data)

        # 2. 构造完整的 AlohaMini Action 字典
        action_dict = self._build_alohamini_action()
        
        # 3. 调试打印：查看生成的完整指令 (每 20 帧打印一次，防止刷屏)
        if hasattr(self, '_frame_count'):
            self._frame_count += 1
        else:
            self._frame_count = 0
        
        if self._frame_count % 20 == 0 and action_dict:
            # 专门打印一下升降高度，方便调试
            logger.info(f"📦 Action: lift.height_mm={action_dict.get('lift.height_mm')}, base_vel_y={action_dict.get('base.back_wheel.vel')}")

        # --- 左臂运动学解算与更新 ---
        if (self.left_arm.mode == ControlMode.POSITION_CONTROL and 
            self.left_arm.target_position is not None):
            
            # 调用 PyBullet 求解逆运动学 (IK)
            ik_solution = self.robot_interface.solve_ik("left", self.left_arm.target_position)
            
            # 整合手腕和夹爪角度，更新左臂所有关节目标值
            current_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("left", ik_solution, 
                                                 self.left_arm.current_wrist_flex, 
                                                 self.left_arm.current_wrist_roll, 
                                                 current_gripper)

        # --- 右臂运动学解算与更新 ---
        if (self.right_arm.mode == ControlMode.POSITION_CONTROL and 
            self.right_arm.target_position is not None):
            
            # 调用 PyBullet 求解逆运动学 (IK)
            ik_solution = self.robot_interface.solve_ik("right", self.right_arm.target_position)
            
            # 整合手腕和夹爪角度，更新右臂所有关节目标值
            current_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("right", ik_solution, 
                                                  self.right_arm.current_wrist_flex, 
                                                  self.right_arm.current_wrist_roll, 
                                                  current_gripper)

        # --- 硬件指令下发 ---
        # 发送指令到真机
        if self.robot_controller and self.robot_controller.is_connected:
            self.robot_controller.send_action(action_dict)
    
    def _update_visualization(self):
        """同步 PyBullet 仿真环境中的模型姿态与可视化标记。"""
        if not self.visualizer:
            return
        
        # 1. 构造当前的 Action 字典用于仿真同步
        action_dict = self._build_alohamini_action()
        
        # 2. 更新移动底盘和升降轴在仿真中的状态（传入原始车身速度）
        sim_action = {
            "lift.height_mm": action_dict.get("lift.height_mm", 0),
            "base.vx": self.base_velocity_target["x"],
            "base.vy": self.base_velocity_target["y"],
            "base.vtheta": self.base_velocity_target["theta"],
        }
        self.visualizer.update_mobile_base_simulation(sim_action)

        # 3. 根据最新关节角度更新双臂的物理模型姿态
        left_angles = self.robot_interface.get_arm_angles("left")
        right_angles = self.robot_interface.get_arm_angles("right")
        
        # 【秘密武器】从 VR 数据中提取 trigger 线性值，替换夹爪角度
        left_trigger = self.vr_raw_data.get('leftController', {}).get('trigger', None)
        right_trigger = self.vr_raw_data.get('rightController', {}).get('trigger', None)
        
        # print(f"🔧 Trigger L:{left_trigger} R:{right_trigger}")
        
        if left_trigger is not None and len(left_angles) > GRIPPER_INDEX:
            # trigger: 0.0 -> 0°, 1.0 -> -90°
            left_angles[GRIPPER_INDEX] = -left_trigger * 90.0
        
        if right_trigger is not None and len(right_angles) > GRIPPER_INDEX:
            right_angles[GRIPPER_INDEX] = -right_trigger * 90.0
        
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
        # 4. 【秘密武器】将 SO100 IK 结果映射到 AlohaMini 机械臂
        if self.visualizer.aloha_id is not None:

            self.visualizer.update_aloha_arm_pose(left_angles, 'left')
            self.visualizer.update_aloha_arm_pose(right_angles, 'right')
        
        # 2. 更新空间中的目标点（Goal）和当前点（Target）可视化标记
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # 显示当前末端执行器的实际物理位置
                current_pos = self.robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if self.left_arm.goal_position is not None:
                # 显示用户期望到达的目标位置（绿色球体）
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # 非控制状态下隐藏所有相关标记，保持界面整洁
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # 显示当前末端执行器的实际物理位置
                current_pos = self.robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if self.right_arm.goal_position is not None:
                # 显示用户期望到达的目标位置（绿色球体）
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # 非控制状态下隐藏所有相关标记，保持界面整洁
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # 3. 步进物理引擎，刷新画面
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
                logger.info(f"🤖 活动控制: {', '.join(active_arms)} | 左: {left_angles.round(1)} | 右: {right_angles.round(1)}")
    
    @property
    def status(self) -> Dict:
        """获取当前控制循环状态。"""
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "right_arm_connected": self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        } 
