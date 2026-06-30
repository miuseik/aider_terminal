#!/usr/bin/env python3
"""Inverse kinematics computer — Damped Least Squares (DLS) solver.

Solves IK for the aider_pro dual-arm robot.
Each arm has 8 revolute joints plus shared lift/waist.
Uses numerical Jacobian + DLS for robust convergence.
"""
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .fk_computer import FKComputer
from config.settings import get_joint_limits_deg


# 左右臂各自的关节链（不含 lift/waist，这些在 IK 中固定）
LEFT_ARM_JOINTS  = [f"left_arm{i}" for i in range(1, 9)]
RIGHT_ARM_JOINTS = [f"right_arm{i}" for i in range(1, 9)]
SHARED_JOINTS    = ["lift_Link", "waist_Link"]


class IKComputer:
    """DLS IK for a single kinematic chain.

    支持身体避碰: 通过 null-space 次级任务将肘部连杆
    排斥在身体圆柱 (Z 轴为中心) 之外。
    """

    def __init__(
        self,
        fk: FKComputer,
        joint_names: List[str],
        ee_link: str,
        max_iter: int = 50,
        tol: float = 1e-4,
        damping: float = 0.05,
        dt: float = 0.1,
        body_avoid_links: Optional[List[str]] = None,
        body_radius: float = 0.12,
        body_avoid_weight: float = 0.3,
    ):
        self.fk = fk
        self.joint_names = joint_names
        self.ee_link = ee_link
        self.max_iter = max_iter
        self.tol = tol
        self.damping = damping
        self.dt = dt
        self.body_avoid_links = body_avoid_links or []
        self.body_radius = body_radius
        self.body_avoid_weight = body_avoid_weight

    def solve(
        self,
        target: np.ndarray,                     # (3,) 目标世界坐标
        current_joint_values: Dict[str, float], # 所有关节当前值
    ) -> Optional[Dict[str, float]]:
        """返回 IK 解（该臂关节的新值），失败返回 None。

        current_joint_values 应包含所有关节（包括 shared）的值，
        但只优化 self.joint_names 中的关节。
        """
        n = len(self.joint_names)
        q = np.array([current_joint_values.get(j, 0.0) for j in self.joint_names])

        for _ in range(self.max_iter):
            jv = dict(current_joint_values)
            for i, name in enumerate(self.joint_names):
                jv[name] = float(q[i])
            ee_now = np.array(self.fk.pos(self.ee_link, jv))

            error = target - ee_now
            if np.linalg.norm(error) < self.tol:
                break

            J = self._numerical_jacobian(jv, epsilon=1e-4)

            I3 = np.eye(3)
            JJt = J @ J.T + self.damping ** 2 * I3
            try:
                Jt_pinv = J.T @ np.linalg.solve(JJt, np.eye(3))
            except np.linalg.LinAlgError:
                Jt_pinv = J.T @ np.linalg.pinv(JJt)

            delta_q = Jt_pinv @ error * self.dt

            max_step = 0.3
            if np.max(np.abs(delta_q)) > max_step:
                delta_q = delta_q * (max_step / np.max(np.abs(delta_q)))

            q = q + delta_q

            # ── 身体避碰: null-space 次级任务 ──
            if self.body_avoid_links:
                avoid_grad = self._avoidance_gradient(jv)
                grad_norm = float(np.linalg.norm(avoid_grad))
                if grad_norm > 1e-8:
                    N = np.eye(n) - Jt_pinv @ J
                    q_avoid = N @ avoid_grad * self.body_avoid_weight * self.dt
                    q = q + q_avoid

            q = self._clamp(q)

        result: Dict[str, float] = {}
        for i, name in enumerate(self.joint_names):
            result[name] = float(q[i])
        return result

    def _numerical_jacobian(self, joint_values: Dict[str, float], epsilon: float = 1e-4) -> np.ndarray:
        """中心差分计算 3×n Jacobian."""
        n = len(self.joint_names)
        J = np.zeros((3, n))

        jv = dict(joint_values)
        for i, name in enumerate(self.joint_names):
            orig = jv[name]

            jv[name] = orig + epsilon
            pos_plus = np.array(self.fk.pos(self.ee_link, jv))

            jv[name] = orig - epsilon
            pos_minus = np.array(self.fk.pos(self.ee_link, jv))

            J[:, i] = (pos_plus - pos_minus) / (2 * epsilon)

            jv[name] = orig

        return J

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        """限制关节值在限位范围内。
        
        优先级: settings.py 中的 joint_limits_deg > URDF 值 > [-π, π] 回退。
        """
        limits_cfg = get_joint_limits_deg()  # {"arm1": {lower, upper}, ...}
        # 构建 internal_name → limit 映射 (arm1, arm2, ...)
        limits_by_internal = {}
        for joint_name, lim in limits_cfg.items():
            limits_by_internal[joint_name] = (math.radians(lim["lower"]),
                                               math.radians(lim["upper"]))

        for i, urdf_name in enumerate(self.joint_names):
            # urdf_name 如 "left_arm3" → internal 名 "arm3"
            internal = urdf_name.split("_", 1)[1] if "_" in urdf_name else urdf_name

            if internal in limits_by_internal:
                lo, hi = limits_by_internal[internal]
            else:
                # 回退: URDF 或 [-π, π]
                jinfo = self.fk._joints.get(urdf_name, {})
                lo = jinfo.get("lower", -math.pi)
                hi = jinfo.get("upper", math.pi)
                if lo == 0 and hi == 0:
                    lo, hi = -math.pi, math.pi

            q[i] = max(lo, min(hi, q[i]))
        return q

    # ── 身体避碰 ──────────────────────────────────────

    def _avoidance_gradient(self, jv: Dict[str, float]) -> np.ndarray:
        """计算身体避碰梯度 (关节空间下降方向)。"""
        n = len(self.joint_names)
        grad = np.zeros(n)

        for link_name in self.body_avoid_links:
            if link_name not in self.fk._links:
                continue
            pos = np.array(self.fk.pos(link_name, jv))
            r = float(np.linalg.norm(pos[:2]))

            if r >= self.body_radius or r < 1e-8:
                continue

            penetration = self.body_radius - r
            direction_xy = pos[:2] / r

            for i, name in enumerate(self.joint_names):
                orig = jv[name]
                eps = 1e-4
                jv[name] = orig + eps
                pos_plus = np.array(self.fk.pos(link_name, jv))
                jv[name] = orig - eps
                pos_minus = np.array(self.fk.pos(link_name, jv))
                jv[name] = orig

                dpos_xy = (pos_plus[:2] - pos_minus[:2]) / (2 * eps)
                grad[i] += penetration * float(np.dot(direction_xy, dpos_xy))

        return grad


class DualArmIKComputer:
    """双臂 IK 求解器 — 封装左右臂两个 IKComputer.

    默认启用身体避碰: 将 arm3/arm4 连杆排斥在半径 0.12m 圆柱外。
    """

    def __init__(
        self,
        fk: FKComputer | None = None,
        body_radius: float = 0.12,
        body_avoid_weight: float = 0.3,
    ):
        self.fk = fk or FKComputer()
        self.left = IKComputer(
            self.fk, LEFT_ARM_JOINTS, ee_link="left_arm8",
            max_iter=50, tol=1e-4, damping=0.05, dt=0.15,
            body_avoid_links=["left_arm3", "left_arm4"],
            body_radius=body_radius,
            body_avoid_weight=body_avoid_weight,
        )
        self.right = IKComputer(
            self.fk, RIGHT_ARM_JOINTS, ee_link="right_arm8",
            max_iter=50, tol=1e-4, damping=0.05, dt=0.15,
            body_avoid_links=["right_arm3", "right_arm4"],
            body_radius=body_radius,
            body_avoid_weight=body_avoid_weight,
        )

    def solve(
        self,
        left_target: Tuple[float, float, float],
        right_target: Tuple[float, float, float],
        current_joint_values: Dict[str, float],
    ) -> Dict[str, object]:
        """求解双臂 IK。

        返回: {
            "joint_values": {joint_name: value, ...},
            "left_reached": bool,
            "right_reached": bool,
            "left_error": float,
            "right_error": float,
        }
        """
        result_jv = dict(current_joint_values)

        left_sol  = self.left.solve(np.array(left_target), current_joint_values)
        right_sol = self.right.solve(np.array(right_target), current_joint_values)

        left_reached = left_sol is not None
        right_reached = right_sol is not None

        if left_sol:
            result_jv.update(left_sol)
        if right_sol:
            result_jv.update(right_sol)

        left_err = right_err = -1.0
        fk_jv = dict(result_jv)
        if "left_arm8" in self.fk._links:
            le = np.array(self.fk.pos("left_arm8", fk_jv))
            left_err = float(np.linalg.norm(np.array(left_target) - le))
        if "right_arm8" in self.fk._links:
            re = np.array(self.fk.pos("right_arm8", fk_jv))
            right_err = float(np.linalg.norm(np.array(right_target) - re))

        return {
            "joint_values": result_jv,
            "left_reached": left_reached,
            "right_reached": right_reached,
            "left_error": round(left_err, 5),
            "right_error": round(right_err, 5),
        }
