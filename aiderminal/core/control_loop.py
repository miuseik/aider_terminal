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
from aiderminal.core.kinematic.base_ramp import BaseVelocityRamp
import queue  # Add import for thread-safe queue
from typing import Dict, Optional

from aiderminal.config.settings import TelegripConfig
import aiderminal.config.settings as _settings
from aiderminal.core.robot_interface import RobotInterface
from aiderminal.controller.actuator_controller import ActuatorController
from aiderminal.router.actuator_router import ActuatorRouter
# Visualizer 工厂函数将在 setup() 中按需导入
from aiderminal.inputs.base import ControlGoal, ControlMode
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
        self.origin_wrist_roll_angle = 0.0  # 握把激活时的腕部翻滚角 (arm5)
        self.origin_wrist_flex_angle = 0.0  # 握把激活时的腕部弯曲角 (arm6)
        self.origin_wrist_yaw_angle = 0.0   # 握把激活时的腕部偏航角 (arm7)
        self.current_wrist_roll = 0.0  # 当前腕部翻滚角
        self.current_wrist_flex = 0.0  # 当前腕部弯曲角
        self.current_wrist_yaw = 0.0   # 当前腕部偏航角
        # 全位姿 TCP IK: 握把激活时的 TCP 姿态(旋转矩阵) 与 目标姿态四元数[x,y,z,w]
        self.origin_orientation = None
        self.target_orientation = None
        # 腰部参考系: 握把激活时的腰部位姿(位置+旋转)，目标点跟随腰部
        self.grip_waist_pos = None
        self.grip_waist_rot = None

    def reset(self):
        """重置机械臂状态为空闲。"""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.origin_wrist_yaw_angle = 0.0
        self.origin_orientation = None
        self.target_orientation = None
        self.grip_waist_pos = None
        self.grip_waist_rot = None


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
        """初始化控制循环。"""
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config
        # 进程启动时间戳（秒）：随 hardware_status 推送，前端据此检测 Terminal 重启（Docker 拉起新进程时值变化）
        self._started_at = time.time()
        
        # === 核心组件 ===
        self.robot_interface = None
        self.motor_controller = None
        self.api_router = None
        self.visualizer = None
        self.web_keyboard_handler = None
        self.dispatcher = None
        
        # === 控制模式 (全局) ===
        # "keyboard" | "pure_vr" | "exo_vr_mixed"
        self.control_mode: str = "pure_vr"
        # 外骨骼是否正在主动控制（A键或自动启用）
        self._exo_controlling: bool = False
        # 外骨骼处理器引用 (由 TelegripSystem 注入)
        self.exo_handler = None

        # === 机械臂状态 ===
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # === VR 原始数据存储 ===
        self.vr_raw_data = {
            'leftController': {'joystick': {'x': 0, 'y': 0}, 'trigger': None},
            'rightController': {'joystick': {'x': 0, 'y': 0}, 'trigger': None}
        }
        
        # === 底盘状态 ===
        self.base_velocity_target = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # === 底盘速度斜坡 (消除顿挫) ===
        # 斜坡业务逻辑已封装到 aiderminal.core.kinematic.base_ramp.BaseVelocityRamp
        self.base_ramp = BaseVelocityRamp()
        
        # === 身体关节状态 (腰 + 头) ===
        self.body_joint_deltas = {}  # {joint_name: accumulated_delta_rad_per_tick}  键盘增量
        
        # === 控制时序 ===
        self.last_log_time = 0
        self.log_interval = 1.0
        
        # === 调试标志 ===
        self._queue_debug_logged = False
        self._process_debug_logged = False
        
        self.is_running = False

        # === 动作录制 (VR 录制/回放，terminal 侧) ===
        self._recording: Optional["Recording"] = None
        self._playback = None  # PlaybackProvider 实例（懒加载）
    
    async def setup(self) -> bool:
        """设置机器人接口和可视化器。
        
        初始化所有硬件和仿真组件:
        - 连接真机机器人(如果启用)
        - 启动 PyBullet 仿真加载机器人 URDF
        - 初始化运动学解算器(IK/FK)
        
        Returns:
            bool: 是否成功设置
        """
        success = True
        setup_errors = []
        # === 1. 设置机器人接口(不自动连接) ===
        # 连接由用户通过 UI 的「连接」按钮显式触发，
        # 避免用户在未点击连接的情况下启用键盘就能控制真机
        try:
            self.robot_interface = RobotInterface(self.config)

            # ✅ 关键修改：无论真机是否连接，都初始化 ActuatorController
            # 这样 API 命令（如扫描舵机）在 setup() 之前到达时也能正常工作
            self.motor_controller = ActuatorController()
            print("✅ 电机控制器已初始化")
            
            # 初始化API命令路由器
            self.actuator_router = ActuatorRouter(
                control_loop=self
            )
            print("✅ API命令路由器已初始化")

            # 绑定 ServoConfigManager（连接真机时 brand → motor_type 映射）
            if self.robot_interface and hasattr(self.robot_interface, 'servo_config_manager'):
                self.actuator_router.bind_servo_config(
                    self.robot_interface.servo_config_manager
                )
                print("✅ ServoConfigManager 已绑定到 ActuatorRouter")
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            print(error_msg)
        
        # === 2. 设置 PyBullet 仿真、IK 和可视化器 ===
        if self.config.enable_pybullet:
            try:
                # 写文件诊断
                try:
                    with open('/tmp/pybullet_diag.log', 'a') as _df:
                        _df.write(f"[ControlLoop.setup] enable_pybullet={self.config.enable_pybullet} "
                                  f"enable_pybullet_gui={self.config.enable_pybullet_gui} "
                                  f"robot_type={self.config.robot_type}\n")
                except Exception:
                    pass

                # 按需导入可视化器类
                from aiderminal.robots.aider.visualizer import AiderVisualizer
                from aiderminal.robots.aloha.visualizer import AlohaVisualizer
                
                urdf_path = self.config.get_absolute_urdf_path()
                aloha_urdf_path = self.config.get_absolute_aloha_urdf_path() if hasattr(self.config, 'aloha_urdf_path') else None
                
                if self.config.robot_type == "aloha":
                    self.visualizer = AlohaVisualizer(
                        urdf_path=urdf_path,
                        use_gui=self.config.enable_pybullet_gui,
                        log_level=self.config.log_level,
                        aloha_urdf_path=aloha_urdf_path,
                    )
                else:
                    self.visualizer = AiderVisualizer(
                        urdf_path=urdf_path,
                        use_gui=self.config.enable_pybullet_gui,
                        log_level=self.config.log_level,
                    )
                
                # 启动 PyBullet 仿真环境
                if not self.visualizer.setup():
                    error_msg = "PyBullet visualizer setup failed"
                    print(error_msg)
                    self.visualizer = None
                else:
                    # ✅ 关键：提前把 visualizer 注入 robot_interface，
                    # 否则 setup_kinematics 内部 adapter.setup() 永远不会执行
                    self.robot_interface.visualizer = self.visualizer

                    # 将运动学解算器连接到机器人接口
                    # FK(正运动学): 关节角度 → 末端位置
                    # IK(逆运动学): 末端位置 → 关节角度
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    await self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,      # PyBullet 物理引擎客户端
                        self.visualizer.robot_ids,           # 两个机器人实例 ID (左/右)
                        self.visualizer.joint_indices,       # 两个关节索引映射
                        self.visualizer.end_effector_link_indices,  # 两个末端执行器索引
                        joint_limits_min,   # 关节最小限位
                        joint_limits_max    # 关节最大限位
                    )
                    
                    # ---- 诊断: 检查 adapter 初始化状态 ----
                    adapter = self.robot_interface.adapter
                    print(f"[DIAG] adapter.is_setup={adapter.is_setup} | "
                          f"{self.visualizer.get_diagnostic_info()}")
            except Exception as e:
                error_msg = f"PyBullet visualizer setup failed with exception: {e}"
                print(error_msg)
                self.visualizer = None

        
            # 在 Web 键盘处理器上设置机器人接口,使其能获取当前位置
        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
        
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
        if not await self.setup():
            print("控制循环设置失败")
            return
        
        self.is_running = True
        print("控制循环已启动")
        
        # 用当前机器人位置初始化机械臂状态
        self._initialize_arm_states()
        
        # === 主控制循环 (每帧执行) ===
        while self.is_running:
            try:
                # 步骤 1: 处理命令队列(接收 VR/键盘指令)
                await self._process_commands()
                
                # 步骤 2-5: 更新机器人(解算 IK + 发送指令 + 更新仿真)
                await self._update_robot_safely()
                
                # 定期打印状态日志
                self._periodic_logging()
                
                # 控制频率 (默认 50Hz,即每 0.02 秒一帧)
                await asyncio.sleep(self.config.send_interval)
                
            except Exception as e:
                print(f"控制循环错误: {e}")
                await asyncio.sleep(0.1)  # 出错后短暂暂停
        
        print("控制循环已停止")
    
    async def stop(self):
        """停止控制循环。"""
        self.is_running = False

        # 清理 - 先断开机器人 (返回 home 位置并禁用力矩)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                print("🛑 关闭前断开机器人...")
                await self.robot_interface.disengage()
            await self.robot_interface.disconnect()

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
            
            self.left_arm.current_wrist_roll = left_angles[_settings.WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[_settings.WRIST_ROLL_INDEX]
            
            self.left_arm.current_wrist_flex = left_angles[_settings.WRIST_FLEX_INDEX]
            self.right_arm.current_wrist_flex = right_angles[_settings.WRIST_FLEX_INDEX]
            
            self.left_arm.current_wrist_yaw = left_angles[_settings.WRIST_YAW_INDEX]
            self.right_arm.current_wrist_yaw = right_angles[_settings.WRIST_YAW_INDEX]
            
            print(f"左臂初始位置: {left_pos.round(3)}")
            print(f"右臂初始位置: {right_pos.round(3)}")
    
    async def _process_commands(self):
        """处理命令队列中的命令。"""
        try:
            # 处理常规控制目标
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            print(f"处理命令错误: {e}")
            import traceback
            print(f"Traceback: {traceback.format_exc()}")
    
    async def _execute_goal(self, goal: ControlGoal):
        """执行控制目标。"""
        
        # 0. 身体关节控制 (腰/头/升降) — 键盘增量
        if goal.metadata and "body_joint_name" in goal.metadata:
            joint_name = goal.metadata["body_joint_name"]
            delta_rad = goal.metadata.get("body_joint_delta", 0.0)
            self.body_joint_deltas[joint_name] = self.body_joint_deltas.get(joint_name, 0.0) + delta_rad
            return

        # 0.5 外骨骼直接关节角度控制 — 不经过 IK，直接写入 adapter 角度数组
        if goal.metadata and "exo_joint_angle" in goal.metadata:
            if self.robot_interface and self.robot_interface.adapter:
                arm = goal.arm
                joint_name = goal.metadata["exo_joint_name"]  # e.g. "arm1"
                angle_deg = goal.metadata["exo_joint_angle"]
                # 将 arm1-arm4 直接写入 adapter 的角度数组
                joint_idx = int(joint_name.replace("arm", "")) - 1  # "arm1" → 0
                adapter = self.robot_interface.adapter
                angles = adapter.left_angles if arm == "left" else adapter.right_angles
                if joint_idx < len(angles):
                    angles[joint_idx] = angle_deg
                    # 钳制到软限位（物理限位 - 安全余量），避免顶死硬限位
                    adapter._clamp_arm_angles(arm, angles)
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
                arm_state.current_wrist_roll = current_angles[_settings.WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[_settings.WRIST_FLEX_INDEX]
                arm_state.current_wrist_yaw = current_angles[_settings.WRIST_YAW_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[_settings.WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[_settings.WRIST_FLEX_INDEX]
                arm_state.origin_wrist_yaw_angle = current_angles[_settings.WRIST_YAW_INDEX]
                
                print(f"🔄 {goal.arm.upper()}臂: 目标位置重置为当前机器人位置（空闲超时）")
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
                    arm_state.current_wrist_roll = current_angles[_settings.WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[_settings.WRIST_FLEX_INDEX]
                    arm_state.current_wrist_yaw = current_angles[_settings.WRIST_YAW_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[_settings.WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[_settings.WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_yaw_angle = current_angles[_settings.WRIST_YAW_INDEX]

                    # 全位姿 TCP IK: 捕获握把激活时的 TCP 姿态作为旋转基准
                    try:
                        _, _origin_rot = self.robot_interface.get_end_effector_pose(goal.arm)
                        arm_state.origin_orientation = _origin_rot.copy()
                        # 初始目标姿态 = 当前姿态（四元数 [x,y,z,w]）
                        from scipy.spatial.transform import Rotation as _R
                        arm_state.target_orientation = _R.from_matrix(_origin_rot).as_quat()
                    except Exception:
                        arm_state.origin_orientation = None
                        arm_state.target_orientation = None

                    # 腰部参考系: 记录握把激活时的腰部位姿
                    try:
                        _wpos, _wrot = self.robot_interface.adapter.compute_waist_pose()
                        arm_state.grip_waist_pos = _wpos.copy()
                        arm_state.grip_waist_rot = _wrot.copy()
                    except Exception:
                        arm_state.grip_waist_pos = None
                        arm_state.grip_waist_rot = None

                print(f"🔒 {goal.arm.upper()}握把激活 - 控制{goal.arm}臂（目标重置为当前位置）")
                
            elif goal.mode == ControlMode.IDLE:
                # 停用位置控制
                arm_state.reset()
                
                # 隐藏可视化标记点
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                print(f"🔓 {goal.arm.upper()}臂: 位置控制已停用")
        
        # AI / 脚本直接控制末端: 绝对 TCP 位姿 (不走相对偏移, 不依赖 origin)
        if (goal.absolute_tcp is not None and
                arm_state.mode == ControlMode.POSITION_CONTROL):
            _tcp = goal.absolute_tcp
            if _tcp.get("position") is not None:
                arm_state.target_position = np.asarray(_tcp["position"], dtype=float).copy()
                arm_state.goal_position = arm_state.target_position.copy()
                arm_state.origin_position = arm_state.target_position.copy()
            if _tcp.get("orientation") is not None:
                arm_state.target_orientation = np.asarray(_tcp["orientation"], dtype=float).copy()
                arm_state.origin_orientation = arm_state.target_orientation.copy()

        # 处理位置控制 - VR 和键盘发送相对于机器人原点位置的绝对偏移
        # (绝对 TCP 模式已在上面 absolute_tcp 分支处理完, 这里不再覆盖)
        if (goal.target_position is not None
                and arm_state.mode == ControlMode.POSITION_CONTROL):
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

        # 处理目标姿态 - VR 手柄相对旋转 → TCP 目标姿态 (全位姿 TCP IK)
        if (arm_state.mode == ControlMode.POSITION_CONTROL and
                goal.metadata and goal.metadata.get("relative_quaternion") is not None and
                arm_state.origin_orientation is not None):
            try:
                from scipy.spatial.transform import Rotation as _R
                from aiderminal.core.kinematic.utils import vr_rotation_to_robot
                _rel_robot = vr_rotation_to_robot(goal.metadata["relative_quaternion"])  # [x,y,z,w]
                _rel_rot = _R.from_quat(_rel_robot).as_matrix()
                _target_rot = _rel_rot @ arm_state.origin_orientation
                arm_state.target_orientation = _R.from_matrix(_target_rot).as_quat()
            except Exception as _e:
                pass
        
        # 处理腕部运动 — 独立于位置控制模式（VR/键盘/外骨骼都会设置腕部角度）
        # 外骨骼激活时，VR/键盘的手腕值不覆盖 arm_state（避免 exo 绝对角度被 VR 相对角度冲掉）
        from aiderminal.inputs.base import _ACTIVE_INPUT_SOURCES as _wrist_sources
        exo_active_for_wrist = "exo" in _wrist_sources
        is_exo_source = goal.metadata and goal.metadata.get("source") == "exo"

        if goal.wrist_roll_deg is not None:
            if not (exo_active_for_wrist and not is_exo_source):
                if goal.metadata and goal.metadata.get("relative_position", False):
                    arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
                else:
                    arm_state.current_wrist_roll = goal.wrist_roll_deg

        if goal.wrist_flex_deg is not None:
            if not (exo_active_for_wrist and not is_exo_source):
                if goal.metadata and goal.metadata.get("relative_position", False):
                    arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
                else:
                    arm_state.current_wrist_flex = goal.wrist_flex_deg

        if goal.wrist_yaw_deg is not None:
            if not (exo_active_for_wrist and not is_exo_source):
                if goal.metadata and goal.metadata.get("relative_position", False):
                    arm_state.current_wrist_yaw = arm_state.origin_wrist_yaw_angle + goal.wrist_yaw_deg
                else:
                    arm_state.current_wrist_yaw = goal.wrist_yaw_deg
        
        # 处理夹爪控制(独立于模式)
        if goal.gripper_closed is not None and self.robot_interface:
            # 从 metadata 中提取扳机值(如果有)
            trigger_value = goal.metadata.get("trigger_value") if goal.metadata else None
            
            # 调用 robot_interface.set_gripper() 进行线性映射
            # trigger 0-1 → 角度 90°-0°,直接替换 IK 结果的第6个值
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed, trigger_value)
        
        # 处理底盘控制(通过摇杆) - 只存储摇杆数据,在 _update_robot 中统一处理
        if goal.metadata and goal.metadata.get("base_control", False):
            # 检查是否来自键盘（有速度字段）
            if "velocity_x" in goal.metadata:
                # 键盘控制：直接使用速度值
                self.base_velocity_target["x"] = goal.metadata.get("velocity_x", 0)
                self.base_velocity_target["y"] = goal.metadata.get("velocity_y", 0)
                self.base_velocity_target["theta"] = goal.metadata.get("velocity_theta", 0)
                pass  # 底盘控制已接收（不打印，避免刷屏）
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
        """根据 VR 摇杆数据更新底盘和升降轴状态。委托给适配器处理。"""
        if not self.robot_interface:
            return

        adapter = self.robot_interface.adapter
        adapter.update_from_vr_joystick(vr_data)

        # 同步回 control_loop 的本地状态
        self.base_velocity_target["x"] = adapter.base_vx
        self.base_velocity_target["y"] = adapter.base_vy
        self.base_velocity_target["theta"] = adapter.base_vtheta
        self.robot_interface.lift_velocity = adapter.lift_velocity

    async def _update_robot_safely(self):
        """用当前控制目标更新机器人(带错误处理)。"""
        if not self.robot_interface:
            return
        
        try:
            await self._update_robot()
            # 每次 PyBullet stepSimulation 后 yield，防止阻塞事件循环
            await asyncio.sleep(0)
        except Exception as e:
            import traceback
            print(f"更新机器人错误: {e}")
            traceback.print_exc()
    
    async def _update_robot(self):
        """用当前控制目标更新机器人。"""
        if not self.robot_interface:
            return
        
        # 0. 应用身体关节增量 (腰/头/升降) — 键盘
        if self.body_joint_deltas:
            adapter = self.robot_interface.adapter
            for jname, delta in self.body_joint_deltas.items():
                adapter.set_body_joint_delta(jname, delta)
            self.body_joint_deltas.clear()
        
        # 0.5 头显 → 身体关节 (adapter 做校准+映射+限位)
        headset = self.vr_raw_data.get('headset')
        if headset and hasattr(self.robot_interface.adapter, 'feed_headset_raw'):
            self.robot_interface.adapter.feed_headset_raw(headset)
        
        # 1. 获取最新的 VR 数据
        vr_data = self.vr_raw_data
        
        # 2. 更新移动底盘和升降轴
        has_keyboard_base_control = (
            self.web_keyboard_handler and 
            self.web_keyboard_handler.base_state.get("base_control_active", False)
        )
        
        if not has_keyboard_base_control:
            self._update_mobile_base(vr_data)

        # 检查外骨骼是否激活，获取 exo 实际控制的关节索引
        from aiderminal.inputs.base import _ACTIVE_INPUT_SOURCES
        exo_active = "exo" in _ACTIVE_INPUT_SOURCES
        exo_arm_joints: Dict[str, set] = {"left": set(), "right": set()}
        if exo_active and self.exo_handler:
            exo_arm_joints = self.exo_handler.get_controlled_arm_joints()

        # 更新左/右臂
        for arm_name in ("left", "right"):
            arm_state = self.left_arm if arm_name == "left" else self.right_arm

            if exo_active and self.robot_interface and self.robot_interface.adapter:
                # === 外骨骼全控模式：完全不走 IK，直接用 adapter 角度 ===
                # adapter angles 由 _execute_goal 的 exo_joint_angle 路径写入 (arm0-arm3)
                # arm_state.current_wrist_* 由 exo 的 wrist 目标写入 (arm4-arm6)
                adapter = self.robot_interface.adapter
                angles = adapter.left_angles if arm_name == "left" else adapter.right_angles
                current_gripper = angles[_settings.GRIPPER_INDEX] if len(angles) > _settings.GRIPPER_INDEX else 0.0
                self.robot_interface.update_arm_angles(arm_name, angles,
                                                      arm_state.current_wrist_flex,
                                                      arm_state.current_wrist_roll,
                                                      current_gripper,
                                                      arm_state.current_wrist_yaw)

            elif (arm_state.mode == ControlMode.POSITION_CONTROL and
                  arm_state.target_position is not None):
                # === IK 模式（纯VR/键盘，无外骨骼） ===
                # 位置 IK（对齐 aloha）：Pink solver 默认 lock_wrist=True，
                # IK 只约束 TCP 位置、不约束姿态；手腕(arm5/6/7)锁到直控值，
                # 由 update_arm_angles 的 wrist_* 参数决定（与 aloha 一致）。
                _target_pos = arm_state.target_position
                _target_ori = arm_state.target_orientation
                # 腰部参考系: 目标点跟随腰部（lift/waist 变化时保持臂可达）
                if arm_state.grip_waist_pos is not None and self.robot_interface.adapter:
                    try:
                        _wpos, _wrot = self.robot_interface.adapter.compute_waist_pose()
                        # 腰部自握把以来的增量旋转
                        _R_delta = _wrot @ arm_state.grip_waist_rot.T
                        _target_pos = _R_delta @ (arm_state.target_position - arm_state.grip_waist_pos) + _wpos
                        if _target_ori is not None:
                            from scipy.spatial.transform import Rotation as _R
                            _Rt = _R.from_quat(_target_ori).as_matrix()
                            _target_ori = _R.from_matrix(_R_delta @ _Rt).as_quat()
                    except Exception:
                        pass
                # 手腕锁模式 IK 不依赖 orientation 约束，但保留传入（被 solver 忽略）。
                ik_solution = self.robot_interface.solve_ik(
                    arm_name, _target_pos,
                    target_orientation=_target_ori)
                current_gripper = self.robot_interface.get_arm_angles(arm_name)[_settings.GRIPPER_INDEX]
                # 手腕由直控值决定（键盘 R/T/F/G/Z/X 键 + VR 手柄均写入 current_wrist_*），
                # override_wrist=True 确保 IK 锁定的手腕值被直控值覆盖。
                self.robot_interface.update_arm_angles(arm_name, ik_solution,
                                                      arm_state.current_wrist_flex,
                                                      arm_state.current_wrist_roll,
                                                      current_gripper,
                                                      arm_state.current_wrist_yaw,
                                                      override_wrist=True)

            # VR 扳机 → 夹爪
            trigger_key = f"{arm_name}Controller"
            trigger = self.vr_raw_data.get(trigger_key, {}).get('trigger', None)
            if trigger is not None and self.robot_interface and self.robot_interface.adapter:
                self.robot_interface.adapter.apply_gripper_from_trigger(arm_name, trigger)


        # === 底盘速度斜坡: 将方波目标平滑为线性过渡, 消除顿挫 ===
        dt = self.config.send_interval if (hasattr(self, "config") and self.config) else 0.02
        self.base_velocity_target = self.base_ramp.step(self.base_velocity_target, dt)

        # === 同步状态到 robot_interface (用于 send_command 内部使用) ===
        if self.robot_interface:
            # 更新底盘状态
            self.robot_interface.base_velocity_target = {
                "x": self.base_velocity_target["x"],
                "y": self.base_velocity_target["y"],
                "theta": self.base_velocity_target["theta"]
            }
            
            # ✅ 升降轴速度已经通过 VR 摇杆或键盘直接设置，这里不需要额外处理
            # self.robot_interface.lift_velocity 已经在 _update_mobile_base() 或 web_keyboard 中设置
            
            # 更新仿真相关状态
            self.robot_interface.vr_raw_data = self.vr_raw_data
            self.robot_interface.left_arm_state = self.left_arm
            self.robot_interface.right_arm_state = self.right_arm
            self.robot_interface.visualizer = self.visualizer
        
        # === 发送指令到真机并更新仿真 ===
        await self.robot_interface.send_command()

        # === 动作录制: 每帧解算后采集 (在 send_command 之后，状态已最终确定) ===
        if self._recording is not None:
            try:
                self._recording.sample(self.left_arm, self.right_arm, self.robot_interface)
            except Exception as e:
                logger.warning(f"录制采样失败: {e}")

    def _periodic_logging(self):
        """定期打印诊断信息（每2秒一次）。"""
        current_time = time.time()
        if current_time - self.last_log_time >= 2.0:
            self.last_log_time = current_time
            
            # 检查 IK 是否在工作
            left_ik_ok = (self.left_arm.mode == ControlMode.POSITION_CONTROL and self.left_arm.target_position is not None)
            right_ik_ok = (self.right_arm.mode == ControlMode.POSITION_CONTROL and self.right_arm.target_position is not None)
            
            # 检查 adapter 状态
            adapter = self.robot_interface.adapter if self.robot_interface else None
            ik_solver_count = 0
            if adapter:
                if hasattr(adapter, 'ik_solvers'):
                    ik_solver_count = len(adapter.ik_solvers)
                elif hasattr(adapter, 'ik_solver') and adapter.ik_solver is not None:
                    ik_solver_count = 1  # AiderAdapter 使用 Pink IK
            
            # 底盘速度
            bv = self.base_velocity_target
            base_active = abs(bv["x"]) > 0.001 or abs(bv["y"]) > 0.001 or abs(bv["theta"]) > 0.001
            

    
    def set_control_mode(self, mode: str, exo_active: bool = False):
        """设置系统控制模式 (由 Server 广播触发)。
        
        Args:
            mode: "keyboard" | "pure_vr" | "exo_vr_mixed"
            exo_active: 外骨骼是否启停活跃
        """
        old_mode = self.control_mode
        self.control_mode = mode
        self._exo_controlling = exo_active
        
        # 模式切换时同步全局输入活跃状态
        from aiderminal.inputs.base import mark_input_active, mark_input_inactive
        if mode == "exo_vr_mixed" and exo_active:
            mark_input_active("exo")
        else:
            mark_input_inactive("exo")

        if mode == "keyboard":
            mark_input_active("keyboard")
        else:
            mark_input_inactive("keyboard")
        
        if old_mode != mode:
            print(f"🎮 [ControlLoop] 控制模式: {old_mode} → {mode} | exo_controlling={self._exo_controlling}")
    
    @property
    def status(self) -> Dict:
        """获取当前控制循环状态。"""
        # 网络信息（终端在机器人上运行，IP/SSID/hostname 应从这里取）
        try:
            from aiderminal.utils.network_utils import get_network_info
            net = get_network_info()
        except Exception as e:
            net = {"ip": "--", "ssid": "--", "hostname": "--"}
            print(f"[ControlLoop] get_network_info 失败: {e}")

        # 基础状态
        status = {
            "running": self.is_running,
            "control_mode": self.control_mode,
            "exo_controlling": self._exo_controlling,
            # 进程启动时间戳：Terminal 重启（Docker 拉起）后值变化，前端据此弹"已重启"确认
            "started_at": self._started_at,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
            # 网络信息（随硬件状态推送至 Server /api/status）
            "ip": net["ip"],
            "ssid": net["ssid"],
            "hostname": net["hostname"],
        }
        
        # 合并机器人硬件详细状态
        if self.robot_interface:
            status.update({
                # 连接状态
                "robot_connected": self.robot_interface.is_connected,
                "is_engaged": self.robot_interface.is_engaged,
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
                "lift_height_mm": self.robot_interface.lift_height_mm,

                # 在线舵机列表（连接时自动发现，无需额外扫描）
                "online_servos": [
                    {"id": sid, "port": port}
                    for sid, port in self.robot_interface.online_servos.items()
                ],
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
        
        # 录制状态（前端菜单据此渲染列表）
        try:
            from aiderminal.core.recorder import list_recordings
            recordings = list_recordings()
        except Exception:
            recordings = []
        status["recordings"] = recordings
        status["recording_active"] = self._recording is not None
        status["recording_name"] = self._recording.name if self._recording else None

        return status

    # ============ VR 动作录制 / 回放 ============
    async def handle_rec_command(self, msg: Dict):
        """处理前端发来的录制指令。

        msg 格式:
          {"type":"rec_command","action":"start","rec_type":"target"|"joint","name":"xxx"}
          {"type":"rec_command","action":"stop"}
          {"type":"rec_command","action":"play","name":"xxx","rec_type":"target"|"joint"}
          {"type":"rec_command","action":"rename","old_name":"a","new_name":"b"}
          {"type":"rec_command","action":"list"}
        """
        action = msg.get("action")
        from aiderminal.core.recorder import Recording, PlaybackProvider, rename_recording, list_recordings

        if action == "start":
            rec_type = msg.get("rec_type", "target")
            name = msg.get("name") or f"rec_{int(time.time())}"
            if self._recording is not None:
                # 先停掉旧的
                self._recording.save()
            self._recording = Recording(name, rec_type)
            print(f"🎬 [Rec] 开始录制: {name} (type={rec_type})")

        elif action == "stop":
            if self._recording is not None:
                path = self._recording.save()
                print(f"💾 [Rec] 录制结束已保存: {path} ({len(self._recording.frames)} 帧)")
                self._recording = None
            else:
                print("⚠️ [Rec] 没有正在进行的录制")

        elif action == "play":
            name = msg.get("name")
            rec_type = msg.get("rec_type", "target")
            if not name:
                print("⚠️ [Rec] play 缺少 name")
                return
            if self._playback is None:
                self._playback = PlaybackProvider(self)
            # 异步回放，不阻塞控制循环
            asyncio.create_task(self._playback.play(name, rec_type))

        elif action == "rename":
            old = msg.get("old_name")
            new = msg.get("new_name")
            ok = rename_recording(old, new) if (old and new) else False
            print(f"✏️ [Rec] 重命名 {old} → {new}: {'OK' if ok else 'FAIL'}")

        elif action == "list":
            print(f"📋 [Rec] 列表: {list_recordings()}")

        else:
            print(f"⚠️ [Rec] 未知 action: {action}")