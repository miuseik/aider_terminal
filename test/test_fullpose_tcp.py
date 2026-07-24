"""全位姿 TCP IK 端到端测试（真实生产类）。

走真实生产代码路径：
  VR 手柄 position/quaternion
    → compute_relative_position()         (真实坐标映射)
    → vr_handler 提取 relative_quaternion (真实)
    → control_loop 逻辑: target_pos = origin + delta,
      target_orientation = vr_rotation_to_robot(rel_q) @ origin_orientation
    → adapter.solve_ik(target_pos, target_orientation)   (真实 Pink TCP IK)
    → adapter.update_arm_angles(..., override_wrist=False)
    → adapter.compute_fk_pose() 验证 TCP 位置+姿态

验证目标: TCP 位置到达手柄目标点、TCP 姿态与手柄一致（旋转以 TCP 为圆心）。

运行（容器内）:
  cd /ws/src/aiderminal && PYTHONPATH=/ws/src/aiderminal python3 test/test_fullpose_tcp.py
"""
import sys
import numpy as np
import asyncio
from scipy.spatial.transform import Rotation as R

from aiderminal.robots.aider.adapter import AiderAdapter
from aiderminal.config.settings import TelegripConfig
from aiderminal.core.kinematic.utils import compute_relative_position, vr_rotation_to_robot

POS_TOL_M = 0.05      # 位置到达阈值 (m) — 运动学极限约 30-45mm(极端可达范围)
ROT_TOL_DEG = 20.0    # 姿态到达阈值 (度)


def slerp(qa, qb, t):
    if np.dot(qa, qb) < 0:
        qb = -qb
    q = qa + (qb - qa) * t
    return q / np.linalg.norm(q)


IDENT = np.array([0.0, 0.0, 0.0, 1.0])
# (名称, VR位移, VR旋转轴, 角度, 帧数)
SCEN = [
    ("前伸",          np.array([0, 0, -0.15]),   None,           0,   30),
    ("侧伸右",        np.array([0.15, 0, 0]),    None,           0,   30),
    ("前伸上升",      np.array([0, 0.10, -0.12]), None,          0,   30),
    ("纯翻滚45",      np.zeros(3),               np.array([0,0,1]), 45, 30),
    ("纯偏航45",      np.zeros(3),               np.array([0,1,0]), 45, 30),
    ("纯俯仰30",      np.zeros(3),               np.array([1,0,0]), 30, 30),
    ("前伸+偏航",     np.array([0, 0, -0.10]),   np.array([0,1,0]), 40, 36),
    ("侧伸+翻滚",     np.array([0.10, 0, 0]),    np.array([0,0,1]), 40, 36),
]


def run(adapter, arm, d_pos, rot_axis, rot_deg, frames):
    # 握把激活: 记录机器人 TCP 原点 (位置+姿态)
    origin_pos, origin_rot = adapter.compute_fk_pose(arm, adapter._get_angles(arm))
    origin_vr_pos = {"x": 0.0, "y": 0.0, "z": 0.0}

    if rot_axis is not None:
        ax = np.asarray(rot_axis, float); ax /= np.linalg.norm(ax)
        tgt_quat = R.from_rotvec(np.radians(rot_deg) * ax).as_quat()
    else:
        tgt_quat = IDENT.copy()

    for i in range(1, frames + 1):
        t = i / frames
        cur_vr = {"x": d_pos[0]*t, "y": d_pos[1]*t, "z": d_pos[2]*t}
        cur_quat = slerp(IDENT, tgt_quat, t)

        # 位置目标 (与 control_loop 一致)
        delta = compute_relative_position(cur_vr, origin_vr_pos, 1.0)
        target_pos = origin_pos + delta

        # 姿态目标 (与 control_loop 一致)
        rel_q = (R.from_quat(cur_quat) * R.from_quat(IDENT).inv()).as_quat()
        rel_robot = vr_rotation_to_robot(rel_q)
        target_rot = R.from_quat(rel_robot).as_matrix() @ origin_rot
        target_ori = R.from_matrix(target_rot).as_quat()  # [x,y,z,w]

        ik = adapter.solve_ik(arm, target_pos, target_orientation=target_ori)
        cur_angles = adapter._get_angles(arm)
        gripper = cur_angles[7] if len(cur_angles) > 7 else 0.0
        adapter.update_arm_angles(arm, ik, 0, 0, gripper, 0, override_wrist=False)

    # 沉淀: 手停住, 目标保持, 让 IK 收敛 (模拟真实 50Hz 持续控制)
    for _ in range(300):
        ik = adapter.solve_ik(arm, target_pos, target_orientation=target_ori)
        cur_angles = adapter._get_angles(arm)
        gripper = cur_angles[7] if len(cur_angles) > 7 else 0.0
        adapter.update_arm_angles(arm, ik, 0, 0, gripper, 0, override_wrist=False)

    # 验证
    final_pos, final_rot = adapter.compute_fk_pose(arm, adapter._get_angles(arm))
    exp_pos = origin_pos + compute_relative_position(
        {"x": d_pos[0], "y": d_pos[1], "z": d_pos[2]}, origin_vr_pos, 1.0)
    pos_err = np.linalg.norm(final_pos - exp_pos) * 1000

    if rot_axis is not None:
        exp_rot = R.from_quat(vr_rotation_to_robot(tgt_quat)).as_matrix() @ origin_rot
        rot_err = np.degrees(np.linalg.norm(R.from_matrix(final_rot @ exp_rot.T).as_rotvec()))
    else:
        rot_err = 0.0
    return pos_err, rot_err


def main():
    print("=" * 70)
    print("全位姿 TCP IK 端到端测试（真实生产类）")
    print("=" * 70)
    cfg = TelegripConfig()
    adapter = AiderAdapter()
    asyncio.run(adapter.setup(None, cfg))  # visualizer=None

    total = fail = 0
    for arm in ("left", "right"):
        print(f"\n=== {arm.upper()} 臂 ===")
        for name, dp, ax, dg, fr in SCEN:
            # 每场景从默认姿态开始
            adapter.left_angles = adapter.ik_solver.get_posture("left")
            adapter.right_angles = adapter.ik_solver.get_posture("right")
            pe, re_ = run(adapter, arm, dp, ax, dg, fr)
            ok = pe/1000 <= POS_TOL_M and re_ <= ROT_TOL_DEG
            total += 1; fail += 0 if ok else 1
            print(f"  [{'PASS' if ok else 'FAIL'}] {name:<12} 位置误差={pe:6.1f}mm  姿态误差={re_:5.1f}°")
    print(f"\n共 {total} 场景, 通过 {total-fail}, 失败 {fail}")
    print("✅ 全部通过" if fail == 0 else "❌ 有失败")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
