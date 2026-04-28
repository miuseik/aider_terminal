#!/usr/bin/env python3
"""幻尔 Hiwonder LX-16A 舵机测试"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.Hiwonder.lx16a_driver import LX16ADriver, ServoMode


def test_lx16a(port='/dev/ttyUSB0', servo_id=1):
    """测试幻尔 LX-16A 舵机"""
    print(f"\n{'='*60}")
    print("🤖 幻尔 Hiwonder LX-16A 舵机测试")
    print(f"{'='*60}")
    print(f"串口: {port}")
    print(f"舵机ID: {servo_id}")
    print(f"{'='*60}\n")
    
    # 连接
    driver = LX16ADriver(port, baudrate=115200)
    
    if not driver.connect():
        print("❌ 连接失败！请检查：")
        print("   1. 串口是否正确（ls /dev/ttyUSB*）")
        print("   2. 舵机是否上电（7-12V）")
        print("   3. USB连接是否正常")
        return
    
    print("✅ 连接成功\n")
    
    try:
        # 1. Ping 测试
        print("🔍 Ping 测试...")
        if driver.ping(servo_id):
            print(f"✅ ID={servo_id} 在线\n")
        else:
            print(f"❌ ID={servo_id} 离线，尝试扫描...\n")
            # 扫描常见ID
            for sid in range(1, 10):
                if driver.ping(sid):
                    print(f"✅ 找到舵机 ID={sid}")
                    servo_id = sid
                    break
            else:
                print("❌ 未找到任何舵机")
                driver.disconnect()
                return
        
        # 2. 读取状态
        print("\n📊 读取舵机状态...")
        data = driver.get_all_data(servo_id)
        if data:
            print(f"  位置: {data['position']}°")
            print(f"  速度: {data['speed']}")
            print(f"  温度: {data['temperature']}°C")
            print(f"  电压: {data['voltage']}V")
            if data['status']:
                status = data['status']
                print(f"  状态: {'正常' if status['normal'] else '异常'}")
                if status['overheating']:
                    print("    ⚠️ 过热警告")
                if status['overloaded']:
                    print("    ⚠️ 过载警告")
        
        # 3. 位置模式测试
        print("\n" + "="*60)
        print("📍 位置模式测试")
        print("="*60)
        
        angles = [0, 60, 120, 180, 120, 60, 0]
        
        for angle in angles:
            print(f"  → 移动到 {angle}°...")
            driver.move_to_position(servo_id, angle, time_ms=500)
            time.sleep(0.6)
            
            # 读取实际位置
            actual_pos = driver.get_position(servo_id)
            if actual_pos is not None:
                print(f"    实际: {actual_pos:.1f}°")
        
        print("✅ 位置模式测试完成\n")
        
        # 4. 速度模式测试
        print("="*60)
        print("🚗 速度模式测试")
        print("="*60)
        
        # 低速顺时针
        print("\n🔄 低速顺时针 (speed=200) - 持续3秒...")
        driver.rotate_at_speed(servo_id, 200)
        time.sleep(3)
        
        # 停止
        print("  停止")
        driver.rotate_at_speed(servo_id, 0)
        time.sleep(0.5)
        
        # 中速逆时针
        print("\n🔄 中速逆时针 (speed=-400) - 持续3秒...")
        driver.rotate_at_speed(servo_id, -400)
        time.sleep(3)
        
        # 停止
        print("  停止")
        driver.rotate_at_speed(servo_id, 0)
        time.sleep(0.5)
        
        print("✅ 速度模式测试完成\n")
        
        # 5. 回到中间位置
        print("🔄 回到 120° 中间位置...")
        driver.move_to_position(servo_id, 120, time_ms=1000)
        time.sleep(1.1)
        
        # 6. 最终状态
        print("\n📊 最终状态...")
        final_data = driver.get_all_data(servo_id)
        if final_data:
            print(f"  位置: {final_data['position']}°")
            print(f"  温度: {final_data['temperature']}°C")
            print(f"  电压: {final_data['voltage']}V")
        
        print("\n" + "="*60)
        print("✅ 所有测试完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        driver.rotate_at_speed(servo_id, 0)
    finally:
        driver.disconnect()
        print("🔌 已断开连接\n")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    
    test_lx16a(port, servo_id)
