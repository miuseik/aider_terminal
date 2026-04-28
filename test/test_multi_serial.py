#!/usr/bin/env python3
"""飞特 ST3215 多串口测试 - 左臂、右臂、底盘独立控制"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver


def test_multi_serial():
    """测试三个独立串口"""
    print(f"\n{'='*60}")
    print("🤖 多串口独立控制测试")
    print(f"{'='*60}\n")
    
    # 配置三个串口
    configs = [
        {"name": "左臂", "port": "/dev/ttyACM0", "servo_id": 21},
        {"name": "右臂", "port": "/dev/ttyACM1", "servo_id": 22},
        {"name": "底盘", "port": "/dev/ttyACM2", "servo_id": 32},
    ]
    
    drivers = []
    
    # 1. 初始化并连接所有串口
    print("📡 正在连接三个串口...")
    for config in configs:
        driver = ST3215Driver(config["port"], baudrate=1000000)
        if driver.connect():
            print(f"✅ {config['name']} ({config['port']}) 连接成功")
            drivers.append((config, driver))
        else:
            print(f"❌ {config['name']} ({config['port']}) 连接失败")
    
    if not drivers:
        print("\n❌ 没有成功连接的串口")
        return
    
    print(f"\n✅ 成功连接 {len(drivers)} 个串口\n")
    
    try:
        # 2. Ping 测试
        print("🔍 Ping 测试...")
        for config, driver in drivers:
            if driver.ping(config["servo_id"]):
                print(f"✅ {config['name']} ID={config['servo_id']} 在线")
            else:
                print(f"⚠️ {config['name']} ID={config['servo_id']} 离线")
        
        # 3. 切换到速度模式
        print("\n📡 切换到速度模式...")
        for config, driver in drivers:
            driver.set_velocity_mode(config["servo_id"])
        time.sleep(0.5)
        
        # 4. 同步启动 - 所有舵机同时转动
        print("\n🔄 同步启动 (speed=300) - 持续3秒...")
        for config, driver in drivers:
            driver.set_speed(config["servo_id"], 300)
        time.sleep(3)
        
        # 5. 差速测试 - 模拟机器人转向
        print("\n🎯 差速测试（模拟原地旋转）...")
        print("  左臂: +300 (顺时针)")
        print("  右臂: -300 (逆时针)")
        print("  底盘: +500 (前进)")
        
        speeds = {
            "左臂": 300,
            "右臂": -300,
            "底盘": 500,
        }
        
        for config, driver in drivers:
            speed = speeds.get(config["name"], 0)
            driver.set_speed(config["servo_id"], speed)
        
        time.sleep(3)
        
        # 6. 停止所有
        print("\n⏹ 停止所有舵机...")
        for config, driver in drivers:
            driver.stop(config["servo_id"])
        time.sleep(0.5)
        
        # 7. 切换回位置模式
        print("\n📡 切换回位置模式...")
        for config, driver in drivers:
            driver.set_position_mode(config["servo_id"])
        time.sleep(0.5)
        
        # 8. 回到中间位置
        print("🔄 回到 90° 位置...")
        for config, driver in drivers:
            driver.move_to_angle(config["servo_id"], 90, 500)
        time.sleep(0.6)
        
        print("\n✅ 多串口测试完成\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        for config, driver in drivers:
            driver.stop(config["servo_id"])
    finally:
        # 断开所有连接
        print("🔌 断开所有串口...")
        for config, driver in drivers:
            driver.disconnect()
            print(f"  {config['name']} 已断开")
        print()


if __name__ == "__main__":
    test_multi_serial()
