#!/usr/bin/env python3
"""获取幻尔舵机真实ID"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.Hiwonder.lx16a_driver import LX16ADriver

driver = LX16ADriver('/dev/ttyACM0', 115200)

if driver.connect():
    print("✅ 连接成功")
    
    # 用广播地址读取ID
    print("\n🔍 使用广播地址 (0xFE) 读取ID...")
    response = driver._send_command(0xFE, driver.CMD_READ_ID if hasattr(driver, 'CMD_READ_ID') else 14, 
                                   read_response=True, response_length=6)
    
    if response and len(response) >= 6:
        actual_id = response[5]
        print(f"✅ 舵机真实ID: {actual_id}")
        
        # 用真实ID测试
        print(f"\n📍 用 ID={actual_id} 读取位置...")
        pos = driver.get_position(actual_id)
        if pos is not None:
            print(f"  位置: {pos:.1f}°")
        
        # 测试移动
        print(f"\n🔄 移动到 200...")
        driver.set_position(actual_id, 48, 1000)  # 48度 ≈ 脉冲200
        import time
        time.sleep(1.2)
        
        pos = driver.get_position(actual_id)
        if pos is not None:
            print(f"  新位置: {pos:.1f}°")
    else:
        print("❌ 未收到响应")
    
    driver.disconnect()
else:
    print("❌ 连接失败")
