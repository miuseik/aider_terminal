#!/usr/bin/env python3
"""Standalone probe: driver-level ping against live CAN hardware.
Bypasses the running app (PID 1). Only reads params — no motion."""
import sys, logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

sys.path.insert(0, "/app")
from src.drivers.actuator.robStride.robstride_driver import RobStrideOfficialDriver
from src.drivers.actuator.robStride.robstride_dynamics.protocol import ParamIndex, RunMode

def probe(can_if, ids):
    print(f"\n===== probe {can_if} ids={ids} =====", flush=True)
    d = RobStrideOfficialDriver(can_interface=can_if)
    if not d.connect():
        print(f"{can_if}: CONNECT FAILED", flush=True)
        return
    print(f"{can_if}: connected ok", flush=True)
    for mid in ids:
        ok = d.ping(mid, timeout=0.4)
        print(f"{can_if}: ping motor {mid} -> {ok}", flush=True)
        r = d._can.read_parameter(mid, ParamIndex.RUN_MODE, timeout=0.4)
        if r is not None:
            print(f"{can_if}: motor {mid} RUN_MODE result success={r.success} value={r.value}", flush=True)
        else:
            print(f"{can_if}: motor {mid} RUN_MODE -> NO RESPONSE", flush=True)
    d.disconnect()
    print(f"{can_if}: done", flush=True)

probe("can0", [20, 21])
probe("can1", [8, 10])
print("PROBE COMPLETE", flush=True)
