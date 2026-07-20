"""
外骨骼 (Exoskeleton) 处理器。

接收 ESP32 通过 WebSocket 发送的外骨骼关节角度数据 (24 路电位器, 2 片 CD74HC4067 × 12ch)，
将其映射为机器人的 ControlGoal，通过 command_queue 驱动真实机器人。

数据流:
  ESP32 (24路电位器) → WebSocket → aider_server → terminal
  → ExoHandler.process_exo_data() → ControlGoal → command_queue
  → ControlLoop._execute_goal() → 电机

与 VR/键盘一样，外骨骼是另一种控制输入源，通过 mark_input_active('exo')
标记活跃状态，防止与其他输入源冲突。
"""

import asyncio
import json
import logging
import numpy as np
from typing import Dict, List, Optional

from aiderminal.inputs.base import (
    BaseInputProvider, ControlGoal, ControlMode,
    mark_input_active, mark_input_inactive,
)

logger = logging.getLogger(__name__)

# ======================== 外骨骼 → 机器人关节映射 ========================
# 外骨骼 24 路电位器 (2 片 CD74HC4067 × 12ch) → (机器人手臂, 机器人关节索引 0-7)
# 通道分配: 0-11=MUX0(GPIO32), 12-23=MUX1(GPIO33)
# 关节索引: 0=arm1(肩旋转), 1=arm2(肩抬升), 2=arm3(肘弯曲),
#           3=arm4(腕部), 4=arm5(腕翻滚), 5=arm6(腕弯曲),
#           6=arm7(腕偏航), 7=arm8(夹爪)
#
# 默认映射（可根据实际外骨骼结构调整）:
#   左臂: 外骨骼通道 0-7  → 机器人左臂 arm1-arm8  (MUX0 local 0-7)
#   右臂: 外骨骼通道 12-19 → 机器人右臂 arm1-arm8  (MUX1 local 0-7)
#   预留: 通道 8-11, 20-23 (身体关节等)
EXO_TO_ROBOT_MAP: Dict[int, Dict] = {
    # ---- 左臂 (外骨骼通道 0-7 → 机器人左臂关节 0-7) ----
    0:  {"arm": "left",  "joint_index": 0},   # arm1 肩旋转
    1:  {"arm": "left",  "joint_index": 1},   # arm2 肩抬升
    2:  {"arm": "left",  "joint_index": 2},   # arm3 肘弯曲
    3:  {"arm": "left",  "joint_index": 3},   # arm4 腕部
    4:  {"arm": "left",  "joint_index": 4},   # arm5 腕翻滚
    5:  {"arm": "left",  "joint_index": 5},   # arm6 腕弯曲
    6:  {"arm": "left",  "joint_index": 6},   # arm7 腕偏航
    7:  {"arm": "left",  "joint_index": 7},   # arm8 夹爪

    # ---- 右臂 (外骨骼通道 12-19 → 机器人右臂关节 0-7) ----
    12: {"arm": "right", "joint_index": 0},   # arm1 肩旋转
    13: {"arm": "right", "joint_index": 1},   # arm2 肩抬升
    14: {"arm": "right", "joint_index": 2},   # arm3 肘弯曲
    15: {"arm": "right", "joint_index": 3},   # arm4 腕部
    16: {"arm": "right", "joint_index": 4},   # arm5 腕翻滚
    17: {"arm": "right", "joint_index": 5},   # arm6 腕弯曲
    18: {"arm": "right", "joint_index": 6},   # arm7 腕偏航
    19: {"arm": "right", "joint_index": 7},   # arm8 夹爪

    # ---- 预留: 通道 8-11 (MUX0 空余), 20-23 (MUX1 空余) ----
    # 8:  {"arm": "body", "joint_name": "waist_Link"},
    # ...
}


class ExoHandler(BaseInputProvider):
    """外骨骼数据处理器 - 将外骨骼关节角度映射为机器人控制目标。"""

    # 输入源标识（用于 mark_input_active / mark_input_inactive）
    INPUT_SOURCE = "exo"

    def __init__(self, command_queue: asyncio.Queue):
        super().__init__(command_queue)
        self._joint_map = dict(EXO_TO_ROBOT_MAP)

        # 角度校准偏移 (外骨骼原始角度 + offset = 目标机器人角度)
        # key: 外骨骼通道索引, value: 偏移量(度)
        self._angle_offsets: Dict[int, float] = {}

        # 角度缩放因子 (外骨骼角度 * scale = 机器人角度)
        # 默认为 1.0，可调整为负值来反转方向
        self._angle_scales: Dict[int, float] = {}

        # 死区阈值 (度): 角度变化小于此值不发送命令
        self.dead_zone_deg: float = 0.5

        # 是否已激活（收到第一帧数据后自动激活）
        self._activated = False

        # 上一帧角度（用于死区检测）
        self._last_angles: Dict[int, float] = {}

        # 控制循环引用（由 ControlLoop 注入）
        self.control_loop = None

        # 夹爪阈值: trigger 值超过此值认为夹爪闭合
        self.gripper_close_threshold: float = 0.5

        logger.info(f"ExoHandler 初始化: {len(self._joint_map)} 个关节映射")

    async def start(self):
        """启动外骨骼处理器。"""
        self.is_running = True
        logger.info("✅ 外骨骼处理器已启动")

    async def stop(self):
        """停止外骨骼处理器。"""
        self.is_running = False
        if self._activated:
            mark_input_inactive(self.INPUT_SOURCE)
            self._activated = False
        logger.info("🛑 外骨骼处理器已停止")

    async def process_message(self, message: str):
        """处理来自 WebSocket 的外骨骼数据消息。

        兼容两种格式:
        1. JSON 字符串: '{"type":"exo_data","joints":[...],...}'
        2. 已解析的 dict
        """
        try:
            if isinstance(message, str):
                data = json.loads(message)
            else:
                data = message

            if data.get('type') == 'exo_data':
                await self.process_exo_data(data)
        except json.JSONDecodeError:
            logger.warning(f"ExoHandler: 非 JSON 消息: {message[:100]}")
        except Exception as e:
            logger.error(f"ExoHandler: 处理消息错误: {e}")

    async def process_exo_data(self, data: dict):
        """处理外骨骼关节角度数据。

        data 格式:
        {
            "type": "exo_data",
            "device": "exoskeleton",
            "client_id": "exoskeleton",
            "timestamp": 123456,
            "joints": [angle0, angle1, ..., angle23]  # 24路角度(度)
        }
        """
        if not self.is_running:
            return

        joints = data.get('joints', [])
        if not joints:
            return

        # 首次收到数据时激活
        if not self._activated:
            self._activated = True
            mark_input_active(self.INPUT_SOURCE)
            logger.info(f"🦴 外骨骼已激活: {len(joints)} 路关节角度")

        # 逐通道处理
        for exo_ch, mapping in self._joint_map.items():
            if exo_ch >= len(joints):
                continue

            raw_angle = joints[exo_ch]
            if raw_angle is None:
                continue

            # 应用校准
            offset = self._angle_offsets.get(exo_ch, 0.0)
            scale = self._angle_scales.get(exo_ch, 1.0)
            target_angle = raw_angle * scale + offset

            # 死区检测
            last = self._last_angles.get(exo_ch)
            if last is not None and abs(target_angle - last) < self.dead_zone_deg:
                continue
            self._last_angles[exo_ch] = target_angle

            # 根据映射类型生成 ControlGoal
            arm = mapping.get('arm')
            if arm in ('left', 'right'):
                await self._send_arm_joint_goal(arm, mapping['joint_index'], target_angle)
            elif arm == 'body':
                await self._send_body_joint_goal(mapping['joint_name'], target_angle)

    async def _send_arm_joint_goal(self, arm: str, joint_index: int, angle_deg: float):
        """发送单关节角度目标。

        使用 POSITION_CONTROL 模式，通过 target_position 控制末端位置。
        对于腕部关节 (arm5/arm6/arm7) 和夹爪 (arm8)，使用直接角度控制。
        """
        # 夹爪: arm8 (joint_index=7)，通过 gripper_closed + trigger_value 控制
        if joint_index == 7:
            # 将角度映射为 trigger_value (0-1): 假设外骨骼角度范围 -90°~0°
            trigger_value = max(0.0, min(1.0, -angle_deg / 90.0))
            goal = ControlGoal(
                arm=arm,
                gripper_closed=(trigger_value > self.gripper_close_threshold),
                metadata={
                    "source": "exo",
                    "trigger_value": trigger_value,
                    "exo_channel": self._find_exo_channel(arm, joint_index),
                }
            )
            await self.send_goal(goal)
            return

        # 腕部翻滚: arm5 (joint_index=4)
        if joint_index == 4:
            goal = ControlGoal(
                arm=arm,
                wrist_roll_deg=angle_deg,
                metadata={
                    "source": "exo",
                    "exo_channel": self._find_exo_channel(arm, joint_index),
                }
            )
            await self.send_goal(goal)
            return

        # 腕部弯曲: arm6 (joint_index=5)
        if joint_index == 5:
            goal = ControlGoal(
                arm=arm,
                wrist_flex_deg=angle_deg,
                metadata={
                    "source": "exo",
                    "exo_channel": self._find_exo_channel(arm, joint_index),
                }
            )
            await self.send_goal(goal)
            return

        # 腕部偏航: arm7 (joint_index=6)
        if joint_index == 6:
            goal = ControlGoal(
                arm=arm,
                wrist_yaw_deg=angle_deg,
                metadata={
                    "source": "exo",
                    "exo_channel": self._find_exo_channel(arm, joint_index),
                }
            )
            await self.send_goal(goal)
            return

        # arm1-arm4: 通过直接关节角度控制
        # 使用 ControlGoal 的 body_joint_name 机制传递关节名
        # 注意: 这里复用 body_joint 机制来传递 arm 关节角度
        joint_name = f"arm{joint_index + 1}"
        goal = ControlGoal(
            arm=arm,
            mode=ControlMode.POSITION_CONTROL,
            metadata={
                "source": "exo",
                "exo_channel": self._find_exo_channel(arm, joint_index),
                "exo_joint_angle": angle_deg,
                "exo_joint_name": joint_name,
            }
        )
        await self.send_goal(goal)

    async def _send_body_joint_goal(self, joint_name: str, angle_deg: float):
        """发送身体关节目标。"""
        goal = ControlGoal(
            arm="left",  # body 关节不区分左右
            metadata={
                "source": "exo",
                "body_joint_name": joint_name,
                "body_joint_delta_deg": angle_deg,
            }
        )
        await self.send_goal(goal)

    def _find_exo_channel(self, arm: str, joint_index: int) -> Optional[int]:
        """根据 arm + joint_index 反查外骨骼通道。"""
        for ch, m in self._joint_map.items():
            if m.get('arm') == arm and m.get('joint_index') == joint_index:
                return ch
        return None

    # ======================== 配置方法 ========================

    def set_joint_mapping(self, mapping: Dict[int, Dict]):
        """设置外骨骼→机器人关节映射。

        Args:
            mapping: {exo_channel: {"arm": "left"|"right"|"body", "joint_index": 0-7}}
        """
        self._joint_map = dict(mapping)
        logger.info(f"外骨骼关节映射已更新: {len(self._joint_map)} 个通道")

    def set_angle_offset(self, exo_channel: int, offset_deg: float):
        """设置指定通道的角度偏移。"""
        self._angle_offsets[exo_channel] = offset_deg

    def set_angle_scale(self, exo_channel: int, scale: float):
        """设置指定通道的角度缩放因子。"""
        self._angle_scales[exo_channel] = scale

    def set_batch_offsets(self, offsets: Dict[int, float]):
        """批量设置角度偏移。"""
        self._angle_offsets.update(offsets)

    def set_batch_scales(self, scales: Dict[int, float]):
        """批量设置角度缩放因子。"""
        self._angle_scales.update(scales)

    def get_joint_mapping(self) -> Dict[int, Dict]:
        """获取当前关节映射。"""
        return dict(self._joint_map)

    def get_stats(self) -> dict:
        """获取外骨骼处理器状态。"""
        return {
            "activated": self._activated,
            "is_running": self.is_running,
            "mapped_channels": len(self._joint_map),
            "last_angles_count": len(self._last_angles),
        }
