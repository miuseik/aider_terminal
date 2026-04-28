#!/usr/bin/env python3
"""飞特 ST3215 舵机快速测试脚本"""

import sys
import time
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.feetech.st3215_driver import ST3215Driver


def test_servo(port='/dev/ttyACM0', servo_id=1, baudrate=1000000):
    """测试单个舵机"""
    print(f"\n{'='*60}")
    print(f"飞特 ST3215 舵机测试")
    print(f"{'='*60}")
    print(f"串口: {port}")
    print(f"舵机ID: {servo_id}")
    print(f"波特率: {baudrate}")
    print(f"{'='*60}\n")
    
    # 1. 连接舵机
    print("📡 正在连接舵机...")
    driver = ST3215Driver(port, baudrate=baudrate)
    
    if not driver.connect():
        print("❌ 连接失败！请检查：")
        print("   1. 串口是否正确（ls /dev/ttyACM*）")
        print("   2. 舵机是否上电")
        print("   3. 波特率是否匹配（默认 1000000）")
        return False
    
    print("✅ 连接成功\n")
    
    try:
        # 2. 读取当前角度
        print("📖 读取当前角度...")
        current_pos = driver.get_position(servo_id)
        if current_pos is not None:
            print(f"   当前位置: {current_pos}°\n")
        else:
            print("   ⚠️ 读取失败\n")
        
        # 3. 测试移动到不同角度
        test_angles = [0, 45, 90, -45, 0]
        
        for angle in test_angles:
            print(f"🔄 移动到 {angle}°...")
            success = driver.write_position(servo_id, angle)
            
            if success:
                time.sleep(0.5)  # 等待移动完成
                actual_pos = driver.get_position(servo_id)
                print(f"   ✅ 目标: {angle}°, 实际: {actual_pos}°")
            else:
                print(f"   ❌ 移动失败")
                break
            
            time.sleep(0.3)
        
        # 4. 读取其他信息
        print("\n📊 读取舵机信息...")
        voltage = driver.get_voltage(servo_id)
        temperature = driver.get_temperature(servo_id)
        load = driver.get_load(servo_id)
        
        if voltage is not None:
            print(f"   电压: {voltage}V")
        if temperature is not None:
            print(f"   温度: {temperature}°C")
        if load is not None:
            print(f"   负载: {load}%")
        
        print("\n✅ 测试完成！")
        return True
        
    except Exception as e:
        print(f"\n❌ 测试出错: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 5. 断开连接
        print("\n🔌 断开连接...")
        driver.disconnect()
        print("✅ 已断开\n")


if __name__ == '__main__':
    # 可以从命令行参数获取配置
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    
    test_servo(port=port, servo_id=servo_id)
