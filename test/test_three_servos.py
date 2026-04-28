#!/usr/bin/env python3
"""飞特 ST3215 三舵机同步测试 - 速度模式"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver


def test_three_servos(port='/dev/ttyACM0', servo_ids=[21, 22, 32]):
    """测试三个舵机"""
    print(f"\n{'='*60}")
    print("🚗 三舵机同步速度模式测试")
    print(f"{'='*60}")
    print(f"串口: {port}")
    print(f"舵机ID: {servo_ids}")
    print(f"{'='*60}\n")
    
    # 创建驱动实例
    driver = ST3215Driver(port, baudrate=1000000)
    
    if not driver.connect():
        print("❌ 连接失败！")
        return
    
    print("✅ 连接成功\n")
    
    # Ping 所有舵机
    online_servos = []
    for sid in servo_ids:
        if driver.ping(sid):
            print(f"✅ ID={sid} 在线")
            online_servos.append(sid)
        else:
            print(f"❌ ID={sid} 离线")
    
    if not online_servos:
        print("\n❌ 没有在线的舵机")
        driver.disconnect()
        return
    
    print(f"\n✅ 找到 {len(online_servos)} 个在线舵机: {online_servos}\n")
    
    try:
        # 1. 切换到速度模式
        print("📡 切换到速度模式...")
        for sid in online_servos:
            driver.set_velocity_mode(sid)
        time.sleep(0.5)
        
        # 2. 低速顺时针 - 所有舵机同步
        print("\n🔄 低速顺时针 (speed=200) - 持续5秒...")
        for sid in online_servos:
            driver.set_speed(sid, 200)
        time.sleep(5)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 3. 中速顺时针
        print("\n🔄 中速顺时针 (speed=500) - 持续5秒...")
        for sid in online_servos:
            driver.set_speed(sid, 500)
        time.sleep(5)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 4. 高速顺时针
        print("\n🔄 高速顺时针 (speed=800) - 持续3秒...")
        for sid in online_servos:
            driver.set_speed(sid, 800)
        time.sleep(3)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 5. 低速逆时针
        print("\n🔄 低速逆时针 (speed=-200) - 持续5秒...")
        for sid in online_servos:
            driver.set_speed(sid, -200)
        time.sleep(5)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 6. 中速逆时针
        print("\n🔄 中速逆时针 (speed=-500) - 持续5秒...")
        for sid in online_servos:
            driver.set_speed(sid, -500)
        time.sleep(5)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 7. 差速测试 - 不同速度
        print("\n🎯 差速测试（模拟转向）...")
        print("  左轮慢 (200), 中轮中 (500), 右轮快 (800)")
        
        speeds = {
            online_servos[0]: 200,   # 第一个舵机慢速
        }
        if len(online_servos) > 1:
            speeds[online_servos[1]] = 500  # 第二个舵机中速
        if len(online_servos) > 2:
            speeds[online_servos[2]] = 800  # 第三个舵机快速
        
        for sid, speed in speeds.items():
            driver.set_speed(sid, speed)
        
        time.sleep(5)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 8. 反向差速
        print("\n🎯 反向差速测试...")
        print("  左轮快 (-800), 中轮中 (-500), 右轮慢 (-200)")
        
        reverse_speeds = {
            online_servos[0]: -800,
        }
        if len(online_servos) > 1:
            reverse_speeds[online_servos[1]] = -500
        if len(online_servos) > 2:
            reverse_speeds[online_servos[2]] = -200
        
        for sid, speed in reverse_speeds.items():
            driver.set_speed(sid, speed)
        
        time.sleep(5)
        print("  停止")
        for sid in online_servos:
            driver.stop(sid)
        time.sleep(0.5)
        
        # 9. 切换回位置模式
        print("\n📡 切换回位置模式...")
        for sid in online_servos:
            driver.set_position_mode(sid)
        time.sleep(0.5)
        
        # 回到中间位置
        print("🔄 回到 90° 位置...")
        for sid in online_servos:
            driver.move_to_angle(sid, 90, 500)
        time.sleep(0.6)
        
        print("\n✅ 三舵机测试完成\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        for sid in online_servos:
            driver.stop(sid)
    finally:
        driver.disconnect()
        print("🔌 已断开连接\n")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    # 解析舵机ID列表
    if len(sys.argv) > 2:
        servo_ids = [int(x) for x in sys.argv[2].split(',')]
    else:
        servo_ids = [21, 22, 32]
    
    test_three_servos(port, servo_ids)
