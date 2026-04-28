#!/usr/bin/env python3
"""飞特 ST3215 舵机长时间运行测试 - 慢速转1分钟"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver


def test_long_rotation(port='/dev/ttyACM0', servo_id=21, duration=60):
    """长时间慢速旋转测试"""
    print(f"\n{'='*60}")
    print("⏱️ 长时间慢速旋转测试")
    print(f"{'='*60}")
    print(f"串口: {port}")
    print(f"舵机ID: {servo_id}")
    print(f"持续时间: {duration} 秒 ({duration/60:.1f} 分钟)")
    print(f"{'='*60}\n")
    
    # 连接
    driver = ST3215Driver(port, baudrate=1000000)
    
    if not driver.connect():
        print("❌ 连接失败！")
        return
    
    print("✅ 连接成功\n")
    
    # Ping 测试
    if not driver.ping(servo_id):
        print(f"❌ Ping ID={servo_id} 失败")
        driver.disconnect()
        return
    
    print(f"✅ Ping ID={servo_id} 成功\n")
    
    try:
        # 切换到速度模式
        print("📡 切换到速度模式...")
        driver.set_velocity_mode(servo_id)
        time.sleep(0.5)
        
        # 开始慢速旋转
        speed = 200  # 低速
        print(f"🔄 开始慢速顺时针旋转 (speed={speed})...")
        print(f"⏱️  将持续 {duration} 秒\n")
        
        driver.set_speed(servo_id, speed)
        
        # 倒计时
        start_time = time.time()
        elapsed = 0
        
        while elapsed < duration:
            remaining = duration - elapsed
            minutes = int(remaining // 60)
            seconds = int(remaining % 60)
            
            # 每5秒打印一次进度
            if int(elapsed) % 5 == 0 or elapsed < 1:
                print(f"  ⏰ 剩余时间: {minutes:02d}:{seconds:02d} ({elapsed:.0f}s / {duration}s)")
            
            time.sleep(1)
            elapsed = time.time() - start_time
        
        # 停止
        print("\n⏹ 停止旋转")
        driver.stop(servo_id)
        time.sleep(0.5)
        
        # 切换回位置模式
        print("\n📡 切换回位置模式...")
        driver.set_position_mode(servo_id)
        time.sleep(0.5)
        
        # 回到中间位置
        print("🔄 回到 90° 位置...")
        driver.move_to_angle(servo_id, 90, 500)
        time.sleep(0.6)
        
        total_time = time.time() - start_time
        print(f"\n✅ 测试完成！总用时: {total_time:.1f} 秒")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断测试")
        driver.stop(servo_id)
    finally:
        driver.disconnect()
        print("🔌 已断开连接\n")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 21
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 60  # 默认60秒
    
    test_long_rotation(port, servo_id, duration)
