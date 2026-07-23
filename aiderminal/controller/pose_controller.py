"""
姿态控制器 — 机器人姿态运动的功能性工具模块。

将 goto_pose / return_to_initial_position 从 RobotInterface 中抽取出来，
Interface 只保留数据/状态管理，业务逻辑放到这里。
"""

import asyncio
import math
import numpy as np
from typing import Dict


async def goto_pose(ri, arm: str, pose_name: str, duration: float = 5.0) -> Dict:
    """将机器人和身体关节平滑移动到命名姿态（smoothstep 缓动）。

    Args:
        ri: RobotInterface 实例
        arm: 'left', 'right', 或 'both'（仅控制手臂，身体关节始终移动）
        pose_name: 姿态名称，需在 POSES 字典中存在
        duration: 过渡时长（秒），默认 5.0

    Returns:
        {"success": bool, "message": str}
    """
    poses = ri.list_poses()
    if pose_name not in poses:
        return {"success": False, "message": f"未知姿态 '{pose_name}'，可选: {list(poses.keys())}"}

    pose = poses[pose_name]
    arms_to_move = ["left", "right"] if arm == "both" else [arm]

    # ---- 记录起始位置 ----
    start_left = ri.left_arm_angles.copy()
    start_right = ri.right_arm_angles.copy()
    start_body = {
        "waist":      ri.adapter.waist_angle,
        "head_yaw":   ri.adapter.head_yaw,
        "head_pitch": ri.adapter.head_pitch,
        "lift":       ri.adapter.lift_height_mm,
    }

    # ---- 解析目标位置 ----
    target_left = None
    target_right = None
    target_body = {}

    for target_arm in arms_to_move:
        targets = pose.get(target_arm)
        if targets is None:
            print(f"  ⚠️ 姿态 '{pose_name}' 未定义 {target_arm} 臂角度")
            continue
        targets_arr = np.array(targets, dtype=float)
        if target_arm == "left":
            target_left = targets_arr
            print(f"  🎯 左臂 → '{pose_name}': {targets_arr.round(1)}")
        else:
            target_right = targets_arr
            print(f"  🎯 右臂 → '{pose_name}': {targets_arr.round(1)}")

    # 身体关节（度 → 弧度，lift 保持 mm）
    body_cfg = pose.get("body", {})
    if body_cfg:
        target_body = {
            "waist":      math.radians(body_cfg.get("waist", 0)),
            "head_yaw":   math.radians(body_cfg.get("head_yaw", 0)),
            "head_pitch": math.radians(body_cfg.get("head_pitch", 0)),
            "lift":       float(body_cfg.get("lift", 0)),
        }
        print(f"  🎯 身体 → '{pose_name}': waist={body_cfg.get('waist',0)}° "
              f"head_yaw={body_cfg.get('head_yaw',0)}° "
              f"head_pitch={body_cfg.get('head_pitch',0)}° "
              f"lift={body_cfg.get('lift',0)}mm")

    if target_left is None and target_right is None and not target_body:
        return {"success": False, "message": f"姿态 '{pose_name}' 无有效角度定义"}

    # 真机连接且尚未使能时才使能。
    # 注意：每次 engage() 都会 force_reinitialize 所有电机（先 disable 再 enable），
    # 若无条件调用，切换姿态时会让电机短暂失能一下。仅在真正未使能时调用即可避免。
    if ri.is_connected and not getattr(ri, "is_engaged", False):
        ri.engage()

    # ---- 平滑插值过渡 ----
    steps = max(20, int(duration / 0.05))
    step_s = duration / steps

    for i in range(steps):
        t = (i + 1) / steps
        eased = t * t * (3.0 - 2.0 * t)  # smoothstep
        if target_left is not None:
            ri.left_arm_angles = start_left + (target_left - start_left) * eased
        if target_right is not None:
            ri.right_arm_angles = start_right + (target_right - start_right) * eased
        _apply_body(ri, start_body, target_body, eased)
        ri.last_send_time = 0
        await ri.send_command()
        await asyncio.sleep(step_s)

    # ---- 最终帧：精确设为目标值 ----
    if target_left is not None:
        ri.left_arm_angles = target_left
    if target_right is not None:
        ri.right_arm_angles = target_right
    _apply_body(ri, start_body, target_body, 1.0)
    ri.last_send_time = 0
    await ri.send_command()

    print(f"✅ 已发送 goto_pose 指令: arm={arm}, pose={pose_name}")
    return {"success": True, "message": f"已移动到 '{pose_name}' 姿态"}


async def return_to_initial_position(ri, duration: float = 5.0):
    """将双臂和身体关节平滑移动到初始位置（smoothstep 缓动）。"""
    print("⏪ 正在将机器人平滑返回到初始位置...")
    try:
        target_left = ri.initial_left_arm.copy()
        target_right = ri.initial_right_arm.copy()
        start_left = ri.left_arm_angles.copy()
        start_right = ri.right_arm_angles.copy()
        start_body = {
            "waist":      ri.adapter.waist_angle,
            "head_yaw":   ri.adapter.head_yaw,
            "head_pitch": ri.adapter.head_pitch,
            "lift":       ri.adapter.lift_height_mm,
        }
        target_body = {"waist": 0.0, "head_yaw": 0.0, "head_pitch": 0.0, "lift": 0.0}

        steps = max(20, int(duration / 0.05))
        step_s = duration / steps

        for i in range(steps):
            t = (i + 1) / steps
            eased = t * t * (3.0 - 2.0 * t)  # smoothstep
            ri.left_arm_angles = start_left + (target_left - start_left) * eased
            ri.right_arm_angles = start_right + (target_right - start_right) * eased
            _apply_body(ri, start_body, target_body, eased)
            ri.last_send_time = 0
            await ri.send_command()
            await asyncio.sleep(step_s)

        # 最终帧：精确设为目标值
        ri.left_arm_angles = target_left
        ri.right_arm_angles = target_right
        _apply_body(ri, start_body, target_body, 1.0)
        ri.last_send_time = 0
        await ri.send_command()

        print("✅ 机器人已返回到初始位置")
    except Exception as e:
        print(f"返回初始位置错误: {e}")


def _apply_body(ri, start_body: dict, target_body: dict, eased: float):
    """将 eased 插值结果写入 adapter 的身体关节属性。"""
    for key in target_body:
        val = start_body[key] + (target_body[key] - start_body[key]) * eased
        if key == "lift":
            ri.adapter.lift_height_mm = val
        elif key == "waist":
            ri.adapter.waist_angle = val
        elif key == "head_yaw":
            ri.adapter.head_yaw = val
        elif key == "head_pitch":
            ri.adapter.head_pitch = val
