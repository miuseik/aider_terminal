#!/usr/bin/env python3
"""飞特 ST3215 舵机速度模式测试 - 像车轮一样连续旋转"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver


def test_velocity_mode(driver, servo_id):
    """速度模式测试"""
    print(f"\n{'='*60}")
    print("🚗 速度模式测试（连续旋转）")
    print(f"{'='*60}\n")
    
    # 1. 切换到速度模式
    print("📡 切换到速度模式...")
    if not driver.set_velocity_mode(servo_id):
        print("❌ 切换失败")
        return
    
    time.sleep(0.5)
    
    # 2. 低速顺时针 - 多转几圈
    print("\n🔄 低速顺时针 (speed=200) - 持续10秒...")
    driver.set_speed(servo_id, 200)
    time.sleep(10)
    print("  停止")
    driver.stop(servo_id)
    time.sleep(0.5)
    
    # 3. 中速顺时针 - 多转几圈
    print("\n🔄 中速顺时针 (speed=500) - 持续8秒...")
    driver.set_speed(servo_id, 500)
    time.sleep(8)
    print("  停止")
    driver.stop(servo_id)
    time.sleep(0.5)
    
    # 4. 高速顺时针 - 快速旋转
    print("\n🔄 高速顺时针 (speed=800) - 持续5秒...")
    driver.set_speed(servo_id, 800)
    time.sleep(5)
    print("  停止")
    driver.stop(servo_id)
    time.sleep(0.5)
    
    # 5. 低速逆时针 - 多转几圈
    print("\n🔄 低速逆时针 (speed=-200) - 持续10秒...")
    driver.set_speed(servo_id, -200)
    time.sleep(10)
    print("  停止")
    driver.stop(servo_id)
    time.sleep(0.5)
    
    # 6. 中速逆时针 - 多转几圈
    print("\n🔄 中速逆时针 (speed=-500) - 持续8秒...")
    driver.set_speed(servo_id, -500)
    time.sleep(8)
    print("  停止")
    driver.stop(servo_id)
    time.sleep(0.5)
    
    # 7. 加速测试
    print("\n🚀 加速测试 (0 → 800 → 0)...")
    for speed in range(0, 801, 100):
        driver.set_speed(servo_id, speed)
        print(f"  速度: {speed}")
        time.sleep(0.3)
    
    for speed in range(800, -1, -100):
        driver.set_speed(servo_id, speed)
        print(f"  速度: {speed}")
        time.sleep(0.3)
    
    driver.stop(servo_id)
    
    # 8. 切换回位置模式
    print("\n📡 切换回位置模式...")
    driver.set_position_mode(servo_id)
    time.sleep(0.5)
    
    # 回到中间位置
    print("🔄 回到 90° 位置...")
    driver.move_to_angle(servo_id, 90, 500)
    time.sleep(0.6)
    
    print("\n✅ 速度模式测试完成\n")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 21
    
    print(f"\n{'='*60}")
    print("飞特 ST3215 舵机速度模式测试")
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
    
    try:
        # 运行速度模式测试
        test_velocity_mode(driver, servo_id)
        
        print("="*60)
        print("✅ 所有测试完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        driver.stop(servo_id)
    finally:
        driver.disconnect()
        print("🔌 已断开连接\n")


if __name__ == "__main__":
    main()
