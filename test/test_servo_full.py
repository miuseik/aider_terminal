#!/usr/bin/env python3
"""飞特 ST3215 舵机完整测试 - 快速/慢速/连续转动"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver


def test_fast_movement(driver, servo_id):
    """快速转动测试"""
    print(f"\n{'='*60}")
    print("⚡ 快速转动测试")
    print(f"{'='*60}")
    
    angles = [0, 90, 180, 90, 0]
    
    for angle in angles:
        print(f"  → {angle}° (快速)")
        driver.move_to_angle(servo_id, angle, 100)  # 100ms 快速移动
        time.sleep(0.15)
        
        pos = driver.get_position(servo_id)
        actual = (pos / 4095.0) * 360 if pos is not None else None
        print(f"    实际: {actual:.1f}°" if actual is not None else "    读取失败")
    
    print("✅ 快速测试完成\n")


def test_slow_movement(driver, servo_id):
    """慢速转动测试"""
    print(f"\n{'='*60}")
    print("🐌 慢速转动测试")
    print(f"{'='*60}")
    
    angles = [0, 45, 90, 135, 180, 135, 90, 45, 0]
    
    for angle in angles:
        print(f"  → {angle}° (慢速)")
        driver.move_to_angle(servo_id, angle, 1000)  # 1000ms 慢速移动
        time.sleep(1.1)
        
        pos = driver.get_position(servo_id)
        actual = (pos / 4095.0) * 360 if pos is not None else None
        print(f"    实际: {actual:.1f}°" if actual is not None else "    读取失败")
    
    print("✅ 慢速测试完成\n")


def test_continuous_rotation(driver, servo_id):
    """连续摆动测试"""
    print(f"\n{'='*60}")
    print("🔄 连续摆动测试 (10次)")
    print(f"{'='*60}")
    
    for i in range(10):
        # 0° → 180°
        driver.move_to_angle(servo_id, 0, 200)
        time.sleep(0.25)
        
        # 180° → 0°
        driver.move_to_angle(servo_id, 180, 200)
        time.sleep(0.25)
        
        if (i + 1) % 5 == 0:
            print(f"  已完成 {i + 1}/10 次摆动")
    
    print("✅ 连续测试完成\n")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 21
    
    print(f"\n{'='*60}")
    print("飞特 ST3215 舵机完整测试")
    print(f"{'='*60}")
    print(f"串口: {port}")
    print(f"舵机ID: {servo_id}")
    print(f"{'='*60}\n")
    
    # 连接
    print("📡 正在连接...")
    driver = ST3215Driver(port, baudrate=1000000)
    
    if not driver.connect():
        print("❌ 连接失败！请检查：")
        print("   1. 串口是否正确")
        print("   2. 舵机是否上电（7-12V）")
        print("   3. USB连接是否正常")
        return
    
    print("✅ 连接成功\n")
    
    # Ping 测试
    if not driver.ping(servo_id):
        print(f"❌ Ping ID={servo_id} 失败，舵机可能不在线")
        driver.disconnect()
        return
    
    print(f"✅ Ping ID={servo_id} 成功\n")
    
    # 读取初始位置
    initial_pos = driver.get_position(servo_id)
    print(f"📍 初始位置: {initial_pos}")
    if initial_pos is not None:
        initial_angle = (initial_pos / 4095.0) * 360
        print(f"   角度: {initial_angle:.1f}°\n")
    
    try:
        # 1. 快速转动
        test_fast_movement(driver, servo_id)
        
        # 暂停一下
        print("等待 2 秒...")
        time.sleep(2)
        
        # 2. 慢速转动
        test_slow_movement(driver, servo_id)
        
        # 暂停一下
        print("等待 2 秒...")
        time.sleep(2)
        
        # 3. 连续摆动
        test_continuous_rotation(driver, servo_id)
        
        # 回到初始位置
        print("🔄 回到初始位置...")
        driver.move_to_angle(servo_id, 90, 500)
        time.sleep(0.6)
        
        print("\n" + "="*60)
        print("✅ 所有测试完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
    finally:
        driver.disconnect()
        print("🔌 已断开连接\n")


if __name__ == "__main__":
    main()
