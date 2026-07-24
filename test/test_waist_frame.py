"""腰部参考系测试: 复刻 control_loop 全位姿 + 腰部增量变换。

验证: 到位后 waist/lift 动态变化, TCP 是否跟随腰部保持在臂可达范围内。
对比 无腰部参考(base_link固定) vs 有腰部参考(跟随腰部)。

运行（容器内）:
  cd /ws/src/aiderminal && PYTHONPATH=/ws/src/aiderminal python3 test/test_waist_frame.py
"""
import numpy as np
import asyncio
from scipy.spatial.transform import Rotation as R
from aiderminal.robots.aider.adapter import AiderAdapter
from aiderminal.config.settings import TelegripConfig
from aiderminal.core.kinematic.utils import compute_relative_position

OVR = {"x": 0.0, "y": 0.0, "z": 0.0}


def reach(adapter, arm, target_base, ori_base, use_waist, grip_wpos, grip_wrot, steps=100):
    for _ in range(steps):
        tp, to = target_base, ori_base
        if use_waist:
            wpos, wrot = adapter.compute_waist_pose()
            Rd = wrot @ grip_wrot.T
            tp = Rd @ (target_base - grip_wpos) + wpos
            if to is not None:
                to = R.from_matrix(Rd @ R.from_quat(to).as_matrix()).as_quat()
        ik = adapter.solve_ik(arm, tp, target_orientation=to)
        ca = adapter._get_angles(arm); g = ca[7] if len(ca) > 7 else 0.0
        adapter.update_arm_angles(arm, ik, 0, 0, g, 0, override_wrist=False)


def run(adapter, arm, d_pos_vr, waist_sched, use_waist):
    adapter.waist_angle = 0.0; adapter.lift_height_mm = 0.0
    origin_pos, origin_rot = adapter.compute_fk_pose(arm, adapter._get_angles(arm))
    grip_wpos, grip_wrot = adapter.compute_waist_pose()
    delta = compute_relative_position({"x": d_pos_vr[0], "y": d_pos_vr[1], "z": d_pos_vr[2]}, OVR, 1.0)
    target_base = origin_pos + delta
    ori_base = R.from_matrix(origin_rot).as_quat()
    # 到位
    reach(adapter, arm, target_base, ori_base, use_waist, grip_wpos, grip_wrot, 150)
    # 动态 waist: 测 TCP 对"当前应到目标"的跟踪误差
    track_errs = []
    for wd in waist_sched:
        adapter.waist_angle = np.radians(wd)
        reach(adapter, arm, target_base, ori_base, use_waist, grip_wpos, grip_wrot, 80)
        p, _ = adapter.compute_fk_pose(arm, adapter._get_angles(arm))
        # 当前应到目标 (与该模式下的参考系一致)
        if use_waist:
            wpos, wrot = adapter.compute_waist_pose()
            Rd = wrot @ grip_wrot.T
            cur_target = Rd @ (target_base - grip_wpos) + wpos
        else:
            cur_target = target_base
        track_errs.append(np.linalg.norm(p - cur_target) * 1000)
    return track_errs


def main():
    cfg = TelegripConfig(); adapter = AiderAdapter(); asyncio.run(adapter.setup(None, cfg))
    sched = [30, -20]
    for use_waist in (False, True):
        print(f"\n=== {'腰部参考(跟随)' if use_waist else 'base_link固定(无参考)'} ===")
        for arm in ("left", "right"):
            adapter.left_angles = adapter.ik_solver.get_posture("left")
            adapter.right_angles = adapter.ik_solver.get_posture("right")
            d = run(adapter, arm, np.array([0, 0, -0.12]), sched, use_waist)
            print(f"  {arm}: waist=+30 跟踪误差={d[0]:5.1f}mm   waist=-20 跟踪误差={d[1]:5.1f}mm")


if __name__ == "__main__":
    main()
