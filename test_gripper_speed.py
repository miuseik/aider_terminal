#!/usr/bin/env python3
"""17号爪机诊断：回读 LOC_REF/模式 + 速度模式实测能否驱动。

回答三个问题：
1. CSP 写入的 LOC_REF=393.47 固件是否接受（回读）
2. RUN_MODE 切换是否成功
3. VELOCITY 模式 SPD_REF 能否让电机转
"""
import sys
import time
import math
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
sys.path.insert(0, "/app")

from src.drivers.actuator.robStride.robstride_driver import RobStrideOfficialDriver
from src.drivers.actuator.robStride.robstride_dynamics.protocol import (
    ParamIndex, RunMode, MotorType, MotorParams, MOTOR_PARAMS,
)

MOTOR_ID = 17
CAN_IF = "can1"
LEAD_MM = 0.8


def rd(can, idx, label, tries=5):
    for _ in range(tries):
        r = can.read_parameter(MOTOR_ID, idx, timeout=0.6)
        if r and r.success:
            print(f"  {label} = {r.value:.4f}", flush=True)
            return r.value
        time.sleep(0.1)
    print(f"  {label} = 读取失败", flush=True)
    return None


def main():
    d = RobStrideOfficialDriver(can_interface=CAN_IF)
    if not d.connect():
        print("CAN1 CONNECT FAILED", flush=True)
        return
    can = d._can
    can.motor_type_map[MOTOR_ID] = MotorType.RS05
    MOTOR_PARAMS[MotorType.RS05] = MotorParams(
        p_min=-2000.0, p_max=2000.0, v_min=-33.0, v_max=33.0,
        t_min=-17.0, t_max=17.0, kp_min=0.0, kp_max=500.0,
        kd_min=0.0, kd_max=5.0,
    )

    print("=== 17号爪机 诊断 ===", flush=True)
    print("-- 1. 回读当前状态 --", flush=True)
    rd(can, ParamIndex.RUN_MODE, "RUN_MODE")
    rd(can, ParamIndex.LOC_REF, "LOC_REF")
    p0 = rd(can, ParamIndex.MECH_POS, "MECH_POS(start)")
    if p0 is None:
        d.disconnect()
        return

    print("-- 2. 切 VELOCITY 模式 (float 编码写 RUN_MODE) --", flush=True)
    can.enable_motor(MOTOR_ID)
    time.sleep(0.3)
    can.write_parameter(MOTOR_ID, ParamIndex.RUN_MODE, float(RunMode.VELOCITY))
    time.sleep(0.3)
    can.enable_motor(MOTOR_ID)
    time.sleep(0.3)
    rd(can, ParamIndex.RUN_MODE, "RUN_MODE(切后)")

    print("-- 3. 速度模式 SPD_REF=15 rad/s --", flush=True)
    can.write_parameter(MOTOR_ID, ParamIndex.SPD_REF, 15.0)
    for i in range(7):
        time.sleep(0.6)
        p = rd(can, ParamIndex.MECH_POS, f"MECH_POS(t={i + 1})")
        if p is not None:
            print(f"    位移 {(p - p0) / 2 / math.pi * LEAD_MM * 1000:.1f} mm "
                  f"(圈 {(p - p0) / 2 / math.pi:.2f})", flush=True)

    print("-- 4. 停止 --", flush=True)
    can.write_parameter(MOTOR_ID, ParamIndex.SPD_REF, 0.0)
    time.sleep(0.5)
    p1 = rd(can, ParamIndex.MECH_POS, "MECH_POS(stop)")
    if p1 is not None:
        print(f"  净位移 {(p1 - p0) / 2 / math.pi * LEAD_MM * 1000:.1f} mm", flush=True)

    can.disable_motor(MOTOR_ID)
    d.disconnect()
    print("=== 诊断完成 ===", flush=True)


main()
