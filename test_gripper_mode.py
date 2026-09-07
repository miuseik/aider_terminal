#!/usr/bin/env python3
"""17号爪机：RUN_MODE 切换组合实验。

找出哪种写法能让固件切换运行模式：
  A. enable 状态 + int 编码写
  B. enable 状态 + float 编码写
  C. disable 状态 + int 编码写 + enable
  D. disable 状态 + float 编码写 + enable
每种都回读 RUN_MODE 验证。
"""
import sys
import time
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
sys.path.insert(0, "/app")

from src.drivers.actuator.robStride.robstride_driver import RobStrideOfficialDriver
from src.drivers.actuator.robStride.robstride_dynamics.protocol import (
    ParamIndex, RunMode, MotorType, MotorParams, MOTOR_PARAMS,
)

MOTOR_ID = 17
CAN_IF = "can1"
TARGET_MODE = RunMode.VELOCITY  # 2


def read_mode(can, label):
    r = can.read_parameter(MOTOR_ID, ParamIndex.RUN_MODE, timeout=0.5)
    v = r.value if r and r.success else "FAIL"
    print(f"  {label} RUN_MODE = {v}", flush=True)
    return r.value if r and r.success else None


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

    print("=== 17号 RUN_MODE 切换实验 ===", flush=True)
    read_mode(can, "初始")

    # A. enable 状态 + int 编码
    print("-- A. enable + int 编码 --", flush=True)
    can.disable_motor(MOTOR_ID, clear_fault=True)
    time.sleep(0.2)
    can.enable_motor(MOTOR_ID)
    time.sleep(0.3)
    can.write_parameter_int(MOTOR_ID, ParamIndex.RUN_MODE, int(TARGET_MODE))
    time.sleep(0.3)
    read_mode(can, "A后")

    # B. enable 状态 + float 编码
    print("-- B. enable + float 编码 --", flush=True)
    can.write_parameter(MOTOR_ID, ParamIndex.RUN_MODE, float(TARGET_MODE))
    time.sleep(0.3)
    read_mode(can, "B后")

    # C. disable 状态 + int 编码写 + enable
    print("-- C. disable + int 编码 + enable --", flush=True)
    can.disable_motor(MOTOR_ID, clear_fault=True)
    time.sleep(0.2)
    can.write_parameter_int(MOTOR_ID, ParamIndex.RUN_MODE, int(TARGET_MODE))
    time.sleep(0.2)
    can.enable_motor(MOTOR_ID)
    time.sleep(0.3)
    read_mode(can, "C后")

    # D. disable 状态 + float 编码写 + enable
    print("-- D. disable + float 编码 + enable --", flush=True)
    can.disable_motor(MOTOR_ID, clear_fault=True)
    time.sleep(0.2)
    can.write_parameter(MOTOR_ID, ParamIndex.RUN_MODE, float(TARGET_MODE))
    time.sleep(0.2)
    can.enable_motor(MOTOR_ID)
    time.sleep(0.3)
    read_mode(can, "D后")

    print("-- 尝试 SPD_REF=10 看是否转动（当前模式） --", flush=True)
    can.write_parameter(MOTOR_ID, ParamIndex.SPD_REF, 10.0)
    for i in range(3):
        time.sleep(0.5)
        read_mode(can, f"t={i + 1}")

    can.disable_motor(MOTOR_ID, clear_fault=True)
    d.disconnect()
    print("=== 实验完成 ===", flush=True)


main()
