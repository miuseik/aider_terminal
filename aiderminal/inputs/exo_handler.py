"""
外骨骼 (Exoskeleton) 处理器。

接收 ESP32 通过 WebSocket 发送的外骨骼关节角度数据 (16 路扫描, 1 片 CD74HC4067)，
仅 ch0 (左腕偏航) 和 ch13 (右腕偏航) 接了电位器。

数据流:
  ESP32 (16路电位器) → WebSocket → aider_server → terminal
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
# 外骨骼 16 路 (1 片 CD74HC4067 / HW178, ch0-ch15)
# 关节索引: 0=arm1(肩俯仰), 1=arm2(肩偏航), 2=arm3(肩翻滚),
#           3=arm4(肘), 4=arm5(腕翻滚), 5=arm6(腕弯曲),
#           6=arm7(腕偏航), 7=arm8(夹爪)
#
# 实际接线:
#   ch0  → 左臂 arm7 (腕偏航)
#   ch13 → 右臂 arm7 (腕偏航)
#   其余通道未接电位器
EXO_TO_ROBOT_MAP: Dict[int, Dict] = {
    # ---- 左腕偏航 (外骨骼 ch0 → 机器人左臂 arm7) ----
    0:  {"arm": "left",  "joint_index": 6},

    # ---- 右腕偏航 (外骨骼 ch13 → 机器人右臂 arm7) ----
    13: {"arm": "right", "joint_index": 6},
}


class ExoHandler(BaseInputProvider):
    """外骨骼数据处理器 - 将外骨骼关节角度映射为机器人控制目标。"""

    # 输入源标识（用于 mark_input_active / mark_input_inactive）
    INPUT_SOURCE = "exo"

    def __init__(self, command_queue: asyncio.Queue, server_url: str = None):
        super().__init__(command_queue)
        self._joint_map = dict(EXO_TO_ROBOT_MAP)
        self._server_url = server_url.rstrip("/") if server_url else None

        # ---- 校准数据（从 server 拉取） ----
        # {channel: {pot_min, pot_max, angle_min, angle_max, pot_zero, enabled}}
        self._calibration: Dict[int, dict] = {}

        # ---- 旧式 offset/scale（校准数据不可用时兜底） ----
        # key: 外骨骼通道索引, value: 偏移量(度)
        self._angle_offsets: Dict[int, float] = {}
        # key: 外骨骼通道索引, value: 缩放因子
        self._angle_scales: Dict[int, float] = {}

        # 死区阈值 (度): 平滑后角度变化小于此值不发送命令
        self.dead_zone_deg: float = 1.5

        # EMA 指数平滑: alpha 越小越平滑 (0~1, 0=全旧值, 1=全原始值)
        self._ema_alpha: float = 0.25
        self._ema_values: Dict[int, float] = {}  # {channel: smoothed_angle}

        # 是否已激活（收到第一帧数据后自动激活）
        self._activated = False

        # 用户是否手动启用外骨骼控制（A键切换 / auto-enable in exo+VR mode）
        self._user_enabled = False

        # 上一帧角度（用于死区检测）
        self._last_angles: Dict[int, float] = {}

        # 控制循环引用（由 ControlLoop 注入）
        self.control_loop = None

        # 夹爪阈值: trigger 值超过此值认为夹爪闭合
        self.gripper_close_threshold: float = 0.5

        # 最后收到数据的时间戳（用于超时检测）
        self._last_data_time: float = 0.0

        # 超时秒数：超过此时间没收到外骨骼数据则自动停用
        self._timeout_secs: float = 3.0

        logger.info(f"ExoHandler 初始化: {len(self._joint_map)} 个关节映射"
                    f"{' | server=' + self._server_url if self._server_url else ''}")

    async def start(self):
        """启动外骨骼处理器。"""
        self.is_running = True
        # 从 server 拉取校准数据
        if self._server_url:
            await self.load_calibration_from_server()
        # 启动超时看门狗
        asyncio.create_task(self._timeout_watchdog())
        logger.info("✅ 外骨骼处理器已启动")

    async def handle_toggle(self):
        """A键切换外骨骼启停。"""
        self.set_enabled(not self._user_enabled, source="A键")

    def set_enabled(self, enabled: bool, source: str = "system"):
        """设置外骨骼控制启用/停用。
        
        Args:
            enabled: True=启用控制, False=停用
            source: 触发来源 (用于日志)
        """
        if self._user_enabled == enabled:
            return  # 状态未变化
        self._user_enabled = enabled
        if enabled:
            mark_input_active(self.INPUT_SOURCE)
            print(f"🦴 [ExoHandler] {source} → 外骨骼控制已启用")
        else:
            mark_input_inactive(self.INPUT_SOURCE)
            print(f"🦴 [ExoHandler] {source} → 外骨骼控制已停用")

    async def _timeout_watchdog(self):
        """超时看门狗：外骨骼数据断连超时自动停用。"""
        while self.is_running:
            await asyncio.sleep(1.0)
            if not self._user_enabled:
                continue
            import time
            elapsed = time.time() - self._last_data_time
            if elapsed > self._timeout_secs and self._last_data_time > 0:
                self._user_enabled = False
                mark_input_inactive(self.INPUT_SOURCE)
                print(f"⚠️  [ExoHandler] 外骨骼数据超时 ({elapsed:.1f}s)，自动停用")

    # ======================== 校准数据加载 ========================

    async def load_calibration_from_server(self) -> bool:
        """从 aider_server 的 /api/exo/calibration 拉取校准数据。

        校准数据格式:
        [{"channel": 0, "pot_min": -135, "pot_max": 0, "angle_min": -90, "angle_max": 90, "enabled": true}, ...]

        映射公式 (线性插值):
            ratio = (raw_pot - pot_min) / (pot_max - pot_min)
            target_angle = angle_min + ratio * (angle_max - angle_min)
        """
        if not self._server_url:
            logger.warning("未配置 server_url，跳过校准数据加载")
            return False

        import asyncio
        url = f"{self._server_url}/api/exo/calibration"

        def _fetch():
            import requests
            import urllib3
            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
            resp = requests.get(url, timeout=5, verify=False)
            if resp.status_code != 200:
                raise Exception(f"HTTP {resp.status_code}")
            return resp.json().get("data", [])

        try:
            loop = asyncio.get_running_loop()
            entries = await loop.run_in_executor(None, _fetch)

            self._calibration.clear()
            loaded = 0
            for entry in entries:
                if not entry.get("enabled"):
                    continue
                ch = entry["channel"]
                self._calibration[ch] = {
                    "pot_min": entry["pot_min"],
                    "pot_max": entry["pot_max"],
                    "angle_min": entry["angle_min"],
                    "angle_max": entry["angle_max"],
                    "pot_zero": entry.get("pot_zero", None),
                    "reverse": entry.get("reverse", False),
                    # 混合控制: 存储 arm/joint_index，用于 process_exo_data 直接取
                    "arm": entry.get("arm", "left"),
                    "joint_index": entry.get("joint_index", 0),
                }
                loaded += 1

            logger.info(f"✅ 已加载 {loaded}/{len(entries)} 条外骨骼校准数据")
            print(f"✅ [ExoHandler] 已加载 {loaded}/{len(entries)} 条外骨骼校准数据 | channels={list(self._calibration.keys())}")
            return loaded > 0

        except Exception as e:
            logger.warning(f"加载校准数据失败: {e}，将使用默认 offset/scale")
            print(f"⚠️  [ExoHandler] 加载校准数据失败: {e}")
            return False

    def _apply_calibration(self, exo_ch: int, raw_angle: float) -> float:
        """将外骨骼原始电位器角度映射为机器人关节角度。

        支持两种模式:
        1. 中点模式 (pot_zero 已配置):
           - pot_zero 为物理中点参考值 (手写, YAML 中配置)
           - raw >= pot_zero → 正方向, 映射到 [0, angle_max]
           - raw <  pot_zero → 负方向, 映射到 [angle_min, 0]
           - reverse=true 时翻转输出符号 (正反转)
        2. 旧式线性插值 (无 pot_zero 时兜底):
           - pot_min/pot_max → angle_min/angle_max 线性映射
        """
        calib = self._calibration.get(exo_ch)
        if not calib:
            # 兜底: 旧式 offset + scale
            offset = self._angle_offsets.get(exo_ch, 0.0)
            scale = self._angle_scales.get(exo_ch, 1.0)
            return raw_angle * scale + offset

        reverse = calib.get("reverse", False)
        pot_zero = calib.get("pot_zero", None)
        pot_min = calib["pot_min"]
        pot_max = calib["pot_max"]
        angle_min = calib["angle_min"]
        angle_max = calib["angle_max"]

        # ---- 中点模式 (pot_zero 有配置) ----
        if pot_zero is not None and isinstance(pot_zero, (int, float)):
            if raw_angle >= pot_zero:
                # 正方向: pot_zero → pot_max  映射到  0 → angle_max
                span = pot_max - pot_zero
                if span < 0.001:
                    angle = 0.0
                else:
                    ratio = (raw_angle - pot_zero) / span
                    ratio = max(0.0, min(1.0, ratio))
                    angle = ratio * angle_max
            else:
                # 负方向: pot_min → pot_zero  映射到  angle_min → 0
                span = pot_zero - pot_min
                if span < 0.001:
                    angle = 0.0
                else:
                    ratio = (pot_zero - raw_angle) / span
                    ratio = max(0.0, min(1.0, ratio))
                    angle = -ratio * abs(angle_min)
        else:
            # ---- 旧式线性插值 (向后兼容, pot_zero 未配置) ----
            if abs(pot_max - pot_min) < 0.001:
                return raw_angle
            ratio = (raw_angle - pot_min) / (pot_max - pot_min)
            ratio = max(0.0, min(1.0, ratio))
            angle = angle_min + ratio * (angle_max - angle_min)

        # ---- 正反转 ----
        if reverse:
            angle = -angle

        return angle

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

        # 更新最后收到数据的时间
        import time
        self._last_data_time = time.time()

        # 首次收到数据时标记已连接（但不自动启用控制）
        if not self._activated:
            self._activated = True
            logger.info(f"🦴 外骨骼已连接: {len(joints)} 路关节角度")

        # 逐通道处理 — 只处理 _calibration 中已配置的通道
        # 只有 _user_enabled=True 时才发送控制指令
        if not self._user_enabled:
            return

        for exo_ch, calib in self._calibration.items():
            if exo_ch >= len(joints):
                continue

            raw_angle = joints[exo_ch]
            if raw_angle is None:
                continue

            # 应用校准映射
            target_angle = self._apply_calibration(exo_ch, raw_angle)

            # EMA 指数平滑滤波 — 消除 ADC 噪声抖动
            prev_ema = self._ema_values.get(exo_ch)
            if prev_ema is None:
                self._ema_values[exo_ch] = target_angle
                smoothed = target_angle
            else:
                smoothed = self._ema_alpha * target_angle + (1 - self._ema_alpha) * prev_ema
                self._ema_values[exo_ch] = smoothed

            # 死区检测（基于平滑后的角度）
            last = self._last_angles.get(exo_ch)
            if last is not None and abs(smoothed - last) < self.dead_zone_deg:
                continue
            self._last_angles[exo_ch] = smoothed

            # 生成 ControlGoal
            arm = calib.get("arm", "left")
            joint_index = calib.get("joint_index", 0)
            if arm in ('left', 'right'):
                await self._send_arm_joint_goal(arm, joint_index, smoothed)

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
            if not hasattr(ExoHandler, '_wrist_yaw_count'):
                ExoHandler._wrist_yaw_count = 0
            ExoHandler._wrist_yaw_count += 1
            if ExoHandler._wrist_yaw_count % 50 == 1:
                print(f"🦾 [ExoHandler] arm7腕偏航 → {arm} arm7 = {angle_deg:.1f}°")
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

    def get_controlled_arm_joints(self) -> Dict[str, set]:
        """返回 exo 当前控制的 arm 关节索引 (0=arm1, ...,6=arm7)。
        
        Returns:
            {"left": {0, 3, 6}, "right": {6}} — 只包含 exo 实际启用且映射到的关节
            如果 exo 未启用 (_user_enabled=False)，返回空 set。
        """
        result: Dict[str, set] = {"left": set(), "right": set()}
        if not self._user_enabled:
            return result
        for exo_ch, calib in self._calibration.items():
            arm = calib.get("arm", "")
            jidx = calib.get("joint_index", -1)
            if arm in ("left", "right") and 0 <= jidx <= 6:
                result[arm].add(jidx)
        return result

    def get_stats(self) -> dict:
        """获取外骨骼处理器状态。"""
        return {
            "activated": self._activated,
            "is_running": self.is_running,
            "mapped_channels": len(self._joint_map),
            "last_angles_count": len(self._last_angles),
        }
