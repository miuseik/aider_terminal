"""
舵机配置管理器 — 封装 servo_ids.yaml 配置数据，提供 brand / joint_name / motor_type 查询。

数据来源：Server 端 servo_ids.yaml → HTTP API → 扁平 dict
扁平结构示例：
{
    "left_arm": {
        "left_arm1": {"id": 10, "brand": "robstride_04", "joint_name": "肩俯仰", ...},
        ...
    },
    "right_arm": {...},
    "base": {...},
    "lift_axis": {...},
    "body_joints": {...},
}
"""

import logging
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class ServoConfigManager:
    """舵机 ID 配置管理器。

    封装从 Server 获取的 servo_ids.yaml 配置，提供：
      - ID → brand 查询
      - ID → joint_name 查询
      - 构建 motor_type_overrides（用于 ActuatorController）
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        """
        Args:
            config: 扁平化的舵机配置 dict（来自 Server API，无 bus 层级）
        """
        self._raw: Dict[str, Any] = config

        # 构建 {id → {brand, joint_name, part, default_angle, ...}} 快速查找表
        self._id_map: Dict[int, Dict[str, Any]] = {}
        for part_name, part_config in config.items():
            if not isinstance(part_config, dict):
                continue
            for joint_key, joint_info in part_config.items():
                if not isinstance(joint_info, dict):
                    continue
                sid = joint_info.get("id")
                if sid is None:
                    continue
                self._id_map[sid] = {
                    "brand": joint_info.get("brand", ""),
                    "joint_name": joint_info.get("joint_name", str(joint_key)),
                    "part": part_name,
                    "default_angle": joint_info.get("default_angle", 0),
                    "max_angle": joint_info.get("max_angle"),
                    "min_angle": joint_info.get("min_angle"),
                    "zero_offset": joint_info.get("zero_offset", 0),
                }

        logger.info(
            "ServoConfigManager loaded: %d servos from %d parts",
            len(self._id_map),
            sum(1 for v in config.values() if isinstance(v, dict)),
        )

    # ── 查询接口 ──────────────────────────────────────────────

    def get_all_servo_ids(self) -> List[int]:
        """获取所有配置中的执行器 ID。"""
        return sorted(self._id_map.keys())

    def get_servo_count(self) -> int:
        """获取执行器总数。"""
        return len(self._id_map)

    def get_brand(self, servo_id: int) -> str:
        """查询指定 ID 的品牌型号（如 'robstride_04', 'feetech_st3215'）。"""
        return self._id_map.get(servo_id, {}).get("brand", "")

    def get_joint_name(self, servo_id: int) -> str:
        """查询指定 ID 的关节名称（如 '肩俯仰'）。"""
        return self._id_map.get(servo_id, {}).get("joint_name", "")

    def get_info(self, servo_id: int) -> Optional[Dict[str, Any]]:
        """获取指定 ID 的完整配置信息。"""
        return self._id_map.get(servo_id)

    # ── motor_type 构建 ───────────────────────────────────────

    def build_motor_type_overrides(self) -> Dict[int, int]:
        """构建 {motor_id: MotorType.int} 映射（仅 RobStride 电机）。

        brand 格式 "robstride_04" → 取 "_" 后的 "04" → int 4 → MotorType.RS04

        Returns:
            Dict[int, int]: {motor_id: motor_type_int}, e.g. {10: 4, 11: 6, ...}
        """
        overrides: Dict[int, int] = {}
        for sid, info in self._id_map.items():
            brand = info.get("brand", "")
            if not brand.startswith("robstride"):
                continue
            try:
                model_num = int(brand.rsplit("_", 1)[-1])
                overrides[sid] = model_num
            except (ValueError, IndexError):
                logger.warning("Cannot parse motor type from brand '%s' for id=%d", brand, sid)
                continue
        logger.info("Built motor_type_overrides: %s", overrides)
        return overrides

    # ── 分组查询 ──────────────────────────────────────────────

    def get_ids_by_brand_prefix(self, prefix: str) -> List[int]:
        """按品牌前缀筛选 ID 列表。"""
        return [
            sid for sid, info in self._id_map.items()
            if info.get("brand", "").startswith(prefix)
        ]

    def get_robstride_ids(self) -> List[int]:
        """获取所有 RobStride 电机 ID。"""
        return self.get_ids_by_brand_prefix("robstride")

    def get_feetech_ids(self) -> List[int]:
        """获取所有 Feetech 舵机 ID。"""
        return self.get_ids_by_brand_prefix("feetech")

    def get_ids_by_part(self, part_name: str) -> List[int]:
        """按部位名称筛选 ID 列表（如 'left_arm', 'base'）。"""
        return [
            sid for sid, info in self._id_map.items()
            if info.get("part") == part_name
        ]
