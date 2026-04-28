#!/usr/bin/env python3
"""飞特舵机超快测试 - 只连接和读取"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1

print(f"连接 {port}, ID={servo_id}...")
driver = ST3215Driver(port, baudrate=1000000)

if driver.connect():
    print("✅ 连接成功")
    
    # Ping测试
    if driver.ping(servo_id):
        print(f"✅ Ping 成功，舵机ID={servo_id} 在线")
    else:
        print(f"❌ Ping 失败，舵机可能不在线或ID不对")
    
    # 读取位置
    pos = driver.get_position(servo_id)
    print(f"📍 当前位置: {pos}")
    
    # 读取状态
    status = driver.get_status(servo_id)
    print(f"📊 状态: {status}")
    
    driver.disconnect()
else:
    print("❌ 连接失败")
