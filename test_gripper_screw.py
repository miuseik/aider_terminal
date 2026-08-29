#!/usr/bin/env python3
"""
丝杆夹爪 (17号) 多圈测试：从当前位置 正方向 5cm → 反方向 5cm

硬件：M5 粗牙螺丝丝杆，导程 0.8mm/转
  5cm = 50mm = 62.5 圈 = 392.7 rad

为什么必须用 CSP 而不是 MOTION：
  - MOTION 帧位置字段是 16bit，被固件限制 ±2 圈（±12.57 rad）。
    发 392.7 rad 会被回绕成 ~2.48 rad，电机只会原地小幅摆动（已实测验证）。
  - CSP 模式目标 LOC_REF (0x7016) 是 float32 直接写入，无 16bit 回绕，
    可以表达多圈位置（±2000 rad 覆盖 ±500 圈）。

RUN_MODE 切换要点（实测结论）：
  - 必须在 disable 状态写 RUN_MODE，enable 状态下写会被固件拒绝。
  - 本固件按 float32 解析写入的 value，用 write_parameter (float) 写。
  - 切换后回读 RUN_MODE 验证，失败会自动换编码重试并打印诊断。

运行（容器内 /app 下）：
  python3 test_gripper_screw.py
"""
import sys
import time
import math

sys.path.insert(0, "/app")

from src.drivers.actuator.robStride.robstride_dynamics.can_driver import RobstrideCanDriver
from src.drivers.actuator.robStride.robstride_dynamics.protocol import (
    ParamIndex, RunMode, MotorType, MotorParams, MOTOR_PARAMS,
)

# ---------- 参数 ----------
MOTOR_ID = 17
CAN_NAME = "can1"
LEAD_MM = 0.8            # M5 粗牙导程 (mm/转)
DIST_MM = 50.0           # 单段行程 5cm
DIST_RAD = DIST_MM / LEAD_MM * 2 * math.pi   # = 392.7 rad = 62.5 圈
SPEED_LIMIT = 20.0       # rad/s（约 20 秒走完一段）
ARRIVE_TOL_RAD = 1.0     # 到位判定 ~0.13mm
TIMEOUT_S = 120          # 单段最长等待
STALL_CHECK_S = 10       # 位置连续多少秒不动视为卡死

# CSP 位置范围放大：默认 ±12.57 rad 只够 ±2 圈，覆盖 ±500 圈
MOTOR_PARAMS[MotorType.RS05] = MotorParams(
    p_min=-2000.0, p_max=2000.0,
    v_min=-33.0, v_max=33.0,
    t_min=-17.0, t_max=17.0,
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
)


def read_param(can, idx, label):
    r = can.read_parameter(MOTOR_ID, idx, timeout=0.6)
    if r is None:
        print(f"  [{label}] 读取超时 (无应答)", flush=True)
        return None
    print(f"  [{label}] = {r.value:.4f}", flush=True)
    return r.value


def mech_pos(can):
    r = can.read_parameter(MOTOR_ID, ParamIndex.MECH_POS, timeout=0.6)
    return r.value if r is not None else None


def switch_mode(can, mode: RunMode, label: str) -> bool:
    """disable 状态下写 RUN_MODE，float/int 两种编码依次尝试，回读验证。"""
    print(f"-- 切换 {label} (disable 后写 RUN_MODE) --", flush=True)
    can.disable_motor(MOTOR_ID)
    time.sleep(0.3)

    for enc_name, writer in (("float", can.write_parameter),
                             ("int", can.write_parameter_int)):
        writer(MOTOR_ID, ParamIndex.RUN_MODE,
               float(mode) if enc_name == "float" else int(mode))
        time.sleep(0.3)
        v = read_param(can, ParamIndex.RUN_MODE, f"RUN_MODE({enc_name}编码)")
        if v is not None and abs(v - float(mode)) < 0.5:
            print(f"  ✓ {label} 切换成功 (RUN_MODE={v:.0f})", flush=True)
            return True
    print(f"  ✗ {label} 切换失败，两次编码都未生效", flush=True)
    return False


def csp_move(can, target, label):
    """CSP 写绝对位置并轮询 MECH_POS 直到到达或超时。"""
    p0 = mech_pos(can)
    print(f"\n→ {label}: target={target:+.2f} rad "
          f"({(target - p0) / (2 * math.pi):+.1f} 圈)，当前 pos={p0:.3f}", flush=True)

    can.set_position_csp(MOTOR_ID, target)
    t0 = time.time()
    last_p, last_t = p0, time.time()
    while time.time() - t0 < TIMEOUT_S:
        p = mech_pos(can)
        if p is None:
            time.sleep(0.3)
            continue
        print(f"    t={time.time() - t0:5.1f}s  pos={p:9.3f} rad  "
              f"位移={(p - p0) / (2 * math.pi):+7.1f} 圈", flush=True)
        if abs(p - target) < ARRIVE_TOL_RAD:
            print(f"  ✓ {label} 到达 target={target:+.2f} 实际={p:.3f} "
                  f"耗时={time.time() - t0:.1f}s", flush=True)
            return True
        # 卡死检测：位置连续 N 秒无变化
        if abs(p - last_p) < 0.05:
            if time.time() - last_t > STALL_CHECK_S:
                print(f"  ✗ {label} 位置 {STALL_CHECK_S}s 无变化，疑似模式未生效/堵转",
                      flush=True)
                v = read_param(can, ParamIndex.MECH_VEL, "MECH_VEL")
                if v is not None:
                    print(f"     (MECH_VEL={v:.3f} rad/s)", flush=True)
                return False
        else:
            last_p, last_t = p, time.time()
        time.sleep(0.3)
    print(f"  ✗ {label} 超时 ({TIMEOUT_S}s)，未到达 target={target:.2f}", flush=True)
    return False


def main():
    print(f"=== 丝杆夹爪测试 ID={MOTOR_ID} ===", flush=True)
    print(f"导程 {LEAD_MM}mm/转 | 单段 {DIST_MM}mm = {DIST_RAD:.1f} rad "
          f"= {DIST_RAD / 2 / math.pi:.1f} 圈 | 速度上限 {SPEED_LIMIT} rad/s", flush=True)
    print("3 秒后开始动作（可 Ctrl-C 取消）...", flush=True)
    time.sleep(3)

    can = RobstrideCanDriver(can_name=CAN_NAME)
    if not can.connect():
        print(f"✗ CAN {CAN_NAME} 连接失败", flush=True)
        return
    print(f"✓ CAN {CAN_NAME} 已连接", flush=True)

    # 清一次故障，避免历史 FAULT 阻塞
    can.disable_motor(MOTOR_ID, clear_fault=True)
    time.sleep(0.3)

    p0 = mech_pos(can)
    if p0 is None:
        print("✗ 读不到 MECH_POS，电机无应答，检查供电/CAN 接线", flush=True)
        can.disconnect()
        return
    print(f"✓ 当前机械位置 MECH_POS={p0:.4f} rad "
          f"(= {p0 / 2 / math.pi:.2f} 圈，从上次上电零点起)", flush=True)

    if not switch_mode(can, RunMode.POSITION_CSP, "CSP 模式"):
        can.disconnect()
        return

    can.enable_motor(MOTOR_ID)
    time.sleep(0.4)
    can.set_velocity_limit(MOTOR_ID, SPEED_LIMIT)
    time.sleep(0.3)

    # 1) 正方向 5cm
    ok1 = csp_move(can, p0 + DIST_RAD, "正5cm")
    # 2) 反方向 5cm（回起点）
    ok2 = csp_move(can, p0, "反5cm")

    can.disable_motor(MOTOR_ID)
    can.disconnect()
    print("\n=== 结果 ===", flush=True)
    print(f"  正5cm: {'✓' if ok1 else '✗'}   反5cm: {'✓' if ok2 else '✗'}", flush=True)
    print("若两者都失败：看 RUN_MODE 回读值。读回 ~2.8e-45 说明编码错；"
          "读回 0 说明被拒绝；电机完全不动则断电重启再试。", flush=True)


if __name__ == "__main__":
    main()
