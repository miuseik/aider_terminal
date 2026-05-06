"""
Web键盘输入处理器,用于遥操作控制。
该模块通过Web UI提供键盘控制,无需X11环境。
通过HTTP API从浏览器接收键盘输入。
"""

import asyncio
import numpy as np
import logging
import time
from typing import Dict, Optional

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig, POS_STEP, ANGLE_STEP, WRIST_ROLL_INDEX, WRIST_FLEX_INDEX

logger = logging.getLogger(__name__)


class WebKeyboardHandler(BaseInputProvider):
    """基于Web的键盘输入提供者,用于双臂遥操作。

    该处理器处理从Web UI通过HTTP接收的键盘输入。
    可在无头/SSH环境中工作,无需X11。
    """

    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config

        # 机器人接口引用(将由控制循环设置)
        self.robot_interface = None

        # 断开连接回调(将由TelegripSystem设置)
        self.disconnect_callback = None

        # 双臂控制状态(VR式行为)
        self.left_arm_state = {
            "origin_position": None,
            "origin_wrist_roll": 0.0,
            "origin_wrist_flex": 0.0,
            "current_offset": np.zeros(3),
            "current_wrist_roll_offset": 0.0,
            "current_wrist_flex_offset": 0.0,
            "delta_pos": np.zeros(3),
            "delta_wrist_roll": 0.0,
            "delta_wrist_flex": 0.0,
            "position_control_active": False,
            "gripper_closed": False,
            "last_key_time": 0.0,
            "any_key_pressed": False
        }

        self.right_arm_state = {
            "origin_position": None,
            "origin_wrist_roll": 0.0,
            "origin_wrist_flex": 0.0,
            "current_offset": np.zeros(3),
            "current_wrist_roll_offset": 0.0,
            "current_wrist_flex_offset": 0.0,
            "delta_pos": np.zeros(3),
            "delta_wrist_roll": 0.0,
            "delta_wrist_flex": 0.0,
            "position_control_active": False,
            "gripper_closed": False,
            "last_key_time": 0.0,
            "any_key_pressed": False
        }

        # 底盘控制状态
        self.base_state = {
            "velocity_x": 0.0,      # 前后速度
            "velocity_y": 0.0,      # 左右速度
            "velocity_theta": 0.0,  # 旋转速度
            "base_control_active": False,
            "mode_enabled": False   # 底盘控制模式开关
        }

        # 空闲超时重新定位目标(秒)
        self.idle_timeout = 1.0

        # 控制循环任务
        self._control_task = None

    def set_robot_interface(self, robot_interface):
        """设置机器人接口引用以获取当前位置。"""
        self.robot_interface = robot_interface

    @property
    def is_enabled(self) -> bool:
        """检查Web键盘控制是否启用。"""
        return self.is_running

    async def start(self):
        """启动Web键盘处理器。"""
        self.is_running = True

        # 启动控制循环
        self._control_task = asyncio.create_task(self._control_loop())

        print("Web键盘处理器已启动(无需X11)")

    async def stop(self):
        """停止Web键盘处理器。"""
        self.is_running = False

        if self._control_task:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass

        print("Web键盘处理器已停止")

    def _set_keyboard_origin(self, arm: str):
        """设置键盘控制的原点位置(类似VR握把按下)。"""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state

        if self.robot_interface:
            try:
                current_position = self.robot_interface.get_current_end_effector_position(arm)
                current_angles = self.robot_interface.get_arm_angles(arm)

                arm_state["origin_position"] = current_position.copy()
                arm_state["origin_wrist_roll"] = current_angles[WRIST_ROLL_INDEX]
                arm_state["origin_wrist_flex"] = current_angles[WRIST_FLEX_INDEX]

                # 重置当前偏移量
                arm_state["current_offset"] = np.zeros(3)
                arm_state["current_wrist_roll_offset"] = 0.0
                arm_state["current_wrist_flex_offset"] = 0.0

                print(f"🌐 {arm.upper()} arm web keyboard origin set at position: {current_position.round(3)}")

                # 发送重置信号到控制循环
                reset_goal = ControlGoal(
                    arm=arm,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=None,
                    metadata={
                        "source": f"web_keyboard_grip_reset_{arm}",
                        "reset_target_to_current": True
                    }
                )
                try:
                    self.command_queue.put_nowait(reset_goal)
                except:
                    pass

            except Exception as e:
                print(f"设置网页键盘原点失败 {arm}臂: {e}")

    def _update_key_activity(self, arm: str, is_movement_key: bool = True):
        """更新机械臂的最后按键活动时间。"""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        if is_movement_key:
            arm_state["last_key_time"] = time.time()
            arm_state["any_key_pressed"] = True

    def _auto_activate_arm_if_needed(self, arm: str):
        """如果机械臂未激活,自动激活其位置控制。"""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state

        if not arm_state["position_control_active"]:
            arm_state["position_control_active"] = True
            print(f"{arm.upper()}臂位置控制: 自动激活(网页)")
            self._send_mode_change_goal(arm)
            self._set_keyboard_origin(arm)

    def on_key_press(self, key: str):
        """处理来自Web UI的按键按下事件。"""
        try:
            # 左臂控制(WASD + QE)
            if key == 'w':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][1] = -POS_STEP
            elif key == 's':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][1] = POS_STEP
            elif key == 'a':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][0] = POS_STEP
            elif key == 'd':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][0] = -POS_STEP
            elif key == 'q':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][2] = -POS_STEP
            elif key == 'e':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_pos"][2] = POS_STEP

            # 左手腕翻滚角
            elif key == 'z':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_roll"] = -ANGLE_STEP
            elif key == 'x':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_roll"] = ANGLE_STEP

            # 左手腕弯曲角(俯仰)
            elif key == 'r':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_flex"] = -ANGLE_STEP
            elif key == 't':
                self._auto_activate_arm_if_needed("left")
                self._update_key_activity("left")
                self.left_arm_state["delta_wrist_flex"] = ANGLE_STEP

            # 左夹爪控制
            elif key == 'f':
                self.left_arm_state["gripper_closed"] = not self.left_arm_state["gripper_closed"]
                print(f"左夹爪: {'闭合' if self.left_arm_state['gripper_closed'] else '打开'} (网页)")
                self._send_gripper_goal("left")

            # 右臂控制(UIOJKL)
            elif key == 'i':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][1] = -POS_STEP
            elif key == 'k':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][1] = POS_STEP
            elif key == 'j':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][0] = POS_STEP
            elif key == 'l':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][0] = -POS_STEP
            elif key == 'u':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][2] = -POS_STEP
            elif key == 'o':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_pos"][2] = POS_STEP

            # 右手腕翻滚角
            elif key == 'n':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_roll"] = -ANGLE_STEP
            elif key == 'm':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_roll"] = ANGLE_STEP

            # 右手腕弯曲角(俯仰)
            elif key == 'h':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_flex"] = -ANGLE_STEP
            elif key == 'y':
                self._auto_activate_arm_if_needed("right")
                self._update_key_activity("right")
                self.right_arm_state["delta_wrist_flex"] = ANGLE_STEP

            # 右夹爪控制
            elif key == ';':
                self.right_arm_state["gripper_closed"] = not self.right_arm_state["gripper_closed"]
                print(f"右夹爪: {'闭合' if self.right_arm_state['gripper_closed'] else '打开'} (网页)")
                self._send_gripper_goal("right")

            # 底盘控制(方向键 + 数字键 + V/B)
            elif key == 'arrowup':
                self.base_state["base_control_active"] = True
                self.base_state["velocity_x"] = 0.5  # 前进
                print(f"🎮key 底盘控制: 前进 velocity_x=0.5")
            elif key == 'arrowdown':
                self.base_state["base_control_active"] = True
                self.base_state["velocity_x"] = -0.5  # 后退
                print(f"🎮key 底盘控制: 后退 velocity_x=-0.5")
            elif key == 'arrowleft':
                self.base_state["base_control_active"] = True
                self.base_state["velocity_theta"] = 0.5  # 左转
                print(f"🎮key 底盘控制: 左转 velocity_theta=0.5")
            elif key == 'arrowright':
                self.base_state["base_control_active"] = True
                self.base_state["velocity_theta"] = -0.5  # 右转
                print(f"🎮key 底盘控制: 右转 velocity_theta=-0.5")
            elif key == '7':
                self.base_state["base_control_active"] = True
                self.base_state["velocity_y"] = 0.3  # 左平移
                print(f"🎮key 底盘控制: 左平移 velocity_y=0.3")
            elif key == '9':
                self.base_state["base_control_active"] = True
                self.base_state["velocity_y"] = -0.3  # 右平移
                print(f"🎮key 底盘控制: 右平移 velocity_y=-0.3")
            elif key == 'v':
                self.base_state["base_control_active"] = True
                # 升降轴上升 - 发送高度增量命令
                goal = ControlGoal(
                    arm="lift",
                    mode=ControlMode.IDLE,
                    metadata={"action": "set_aloha_height", "height_delta": 0.01}
                )
                try:
                    self.command_queue.put_nowait(goal)
                except:
                    pass
            elif key == 'b':
                self.base_state["base_control_active"] = True
                # 升降轴下降 - 发送高度减量命令
                goal = ControlGoal(
                    arm="lift",
                    mode=ControlMode.IDLE,
                    metadata={"action": "set_aloha_height", "height_delta": -0.01}
                )
                try:
                    self.command_queue.put_nowait(goal)
                except:
                    pass

            # 特殊按键
            elif key == 'tab':
                self.left_arm_state["position_control_active"] = not self.left_arm_state["position_control_active"]
                print(f"左臂位置控制: {'激活' if self.left_arm_state['position_control_active'] else '停用'} (网页)")
                self._send_mode_change_goal("left")
            elif key == 'enter':
                self.right_arm_state["position_control_active"] = not self.right_arm_state["position_control_active"]
                print(f"右臂位置控制: {'激活' if self.right_arm_state['position_control_active'] else '停用'} (网页)")
                self._send_mode_change_goal("right")
            elif key == 'esc':
                print("通过Web按下ESC - 断开机器人连接")
                if self.disconnect_callback:
                    self.disconnect_callback()

        except Exception as e:
            print(f"处理网页按键按下 '{key}' 错误: {e}")

    def on_key_release(self, key: str):
        """处理来自Web UI的按键释放事件。"""
        try:
            # 左臂 - 按键释放时重置增量
            if key in ('w', 's'):
                self.left_arm_state["delta_pos"][1] = 0
                self._check_if_all_keys_released("left")
            elif key in ('a', 'd'):
                self.left_arm_state["delta_pos"][0] = 0
                self._check_if_all_keys_released("left")
            elif key in ('q', 'e'):
                self.left_arm_state["delta_pos"][2] = 0
                self._check_if_all_keys_released("left")
            elif key in ('z', 'x'):
                self.left_arm_state["delta_wrist_roll"] = 0
                self._check_if_all_keys_released("left")
            elif key in ('r', 't'):
                self.left_arm_state["delta_wrist_flex"] = 0
                self._check_if_all_keys_released("left")

            # 右臂 - 按键释放时重置增量
            elif key in ('i', 'k'):
                self.right_arm_state["delta_pos"][1] = 0
                self._check_if_all_keys_released("right")
            elif key in ('j', 'l'):
                self.right_arm_state["delta_pos"][0] = 0
                self._check_if_all_keys_released("right")
            elif key in ('u', 'o'):
                self.right_arm_state["delta_pos"][2] = 0
                self._check_if_all_keys_released("right")
            elif key in ('n', 'm'):
                self.right_arm_state["delta_wrist_roll"] = 0
                self._check_if_all_keys_released("right")
            elif key in ('h', 'y'):
                self.right_arm_state["delta_wrist_flex"] = 0
                self._check_if_all_keys_released("right")
            
            # 底盘 - 按键释放时重置速度
            elif key == 'arrowup' or key == 'arrowdown':
                self.base_state["velocity_x"] = 0.0
            elif key == 'arrowleft' or key == 'arrowright':
                self.base_state["velocity_theta"] = 0.0
            elif key == '7' or key == '9':
                self.base_state["velocity_y"] = 0.0
            elif key == 'v' or key == 'b':
                pass
            
            # 检查所有底盘速度是否为0,如果是则禁用底盘控制
            if (self.base_state["velocity_x"] == 0.0 and 
                self.base_state["velocity_y"] == 0.0 and 
                self.base_state["velocity_theta"] == 0.0):
                self.base_state["base_control_active"] = False

        except Exception as e:
            print(f"处理网页按键释放 '{key}' 错误: {e}")

    def _check_if_all_keys_released(self, arm: str):
        """检查机械臂的所有移动按键是否已释放。"""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state

        if (np.all(arm_state["delta_pos"] == 0) and
            arm_state["delta_wrist_roll"] == 0 and
            arm_state["delta_wrist_flex"] == 0):
            arm_state["any_key_pressed"] = False

    def _send_gripper_goal(self, arm: str):
        """发送夹爪控制目标到队列。"""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        goal = ControlGoal(
            arm=arm,
            mode=ControlMode.IDLE,
            gripper_closed=arm_state["gripper_closed"],
            metadata={"source": f"web_keyboard_gripper_{arm}"}
        )
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass

    def _send_mode_change_goal(self, arm: str):
        """发送模式切换目标到队列。"""
        arm_state = self.left_arm_state if arm == "left" else self.right_arm_state
        mode = ControlMode.POSITION_CONTROL if arm_state["position_control_active"] else ControlMode.IDLE
        goal = ControlGoal(
            arm=arm,
            mode=mode,
            metadata={"source": f"web_keyboard_mode_{arm}"}
        )
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass

    def _send_idle_reset_signal(self, arm: str):
        """发送因空闲超时而重置目标位置的信号。"""
        self._set_keyboard_origin(arm)

    async def _control_loop(self):
        """主控制循环,处理Web键盘输入并发送命令。"""
        print("Web键盘控制循环已启动")

        while self.is_running:
            try:
                # Process both arms
                for arm, arm_state in [("left", self.left_arm_state), ("right", self.right_arm_state)]:
                    if arm_state["position_control_active"]:

                        # 检查空闲超时(1秒无活动后重置原点)
                        current_time = time.time()
                        if (not arm_state["any_key_pressed"] and
                            arm_state["last_key_time"] > 0 and
                            current_time - arm_state["last_key_time"] >= self.idle_timeout):

                            self._send_idle_reset_signal(arm)
                            arm_state["last_key_time"] = 0

                        # 根据增量更新当前偏移量
                        arm_state["current_offset"] += arm_state["delta_pos"]
                        arm_state["current_wrist_roll_offset"] += arm_state["delta_wrist_roll"]
                        arm_state["current_wrist_flex_offset"] += arm_state["delta_wrist_flex"]

                        # 如果有活动移动,发送位置更新
                        if (np.any(arm_state["delta_pos"] != 0) or
                            arm_state["delta_wrist_roll"] != 0 or
                            arm_state["delta_wrist_flex"] != 0):

                            print(f"🎮key 发送{arm}臂控制: offset={arm_state['current_offset']}")
                            goal = ControlGoal(
                                arm=arm,
                                mode=ControlMode.POSITION_CONTROL,
                                target_position=arm_state["current_offset"].copy(),
                                wrist_roll_deg=arm_state["current_wrist_roll_offset"],
                                wrist_flex_deg=arm_state["current_wrist_flex_offset"],
                                metadata={
                                    "source": f"web_keyboard_{arm}",
                                    "relative_position": True
                                }
                            )
                            await self.send_goal(goal)

                # 处理底盘控制
                if self.base_state["base_control_active"]:
                    print(f"🎮key 发送底盘控制: x={self.base_state['velocity_x']}, y={self.base_state['velocity_y']}, theta={self.base_state['velocity_theta']}")
                    goal = ControlGoal(
                        arm="base",
                        mode=ControlMode.IDLE,
                        metadata={
                            "base_control": True,
                            "velocity_x": self.base_state["velocity_x"],
                            "velocity_y": self.base_state["velocity_y"],
                            "velocity_theta": self.base_state["velocity_theta"],
                            "source": "web_keyboard_base"
                        }
                    )
                    try:
                        self.command_queue.put_nowait(goal)
                    except:
                        pass

                # 控制频率: 20Hz
                await asyncio.sleep(0.05)

            except Exception as e:
                print(f"网页键盘控制循环错误: {e}")
                await asyncio.sleep(0.1)

        print("Web键盘控制循环已停止")
