"""
快速测试飞特 ST3215 舵机
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from drivers.feetech.st3215_driver import ST3215Driver, ServoMode


def test_servo():
    """测试 ST3215 舵机控制"""
    print("=" * 60)
    print("🔧 飞特 ST3215 舵机快速测试")
    print("=" * 60)
    
    # ⚠️ 修改为你的串口号
    PORT = "COM10"      # Windows: COM3, COM4... | Linux: /dev/ttyUSB0...
    SERVO_ID = 1        # 舵机ID
    BAUDRATE = 1000000  # ST3215 默认波特率 1M
    
    print(f"\n📡 连接串口: {PORT}")
    print(f"🎯 目标舵机: ID={SERVO_ID}")
    print(f"⚡ 波特率: {BAUDRATE}")
    
    # 创建驱动实例
    driver = ST3215Driver(port=PORT, baudrate=BAUDRATE)
    
    # 连接
    if not driver.connect():
        print("❌ 连接失败，请检查:")
        print("   1. 串口号是否正确")
        print("   2. 舵机是否通电（ST3215需要7.4V-12V）")
        print("   3. 接线是否正确")
        return
    
    print("✅ 连接成功！")
    
    try:
        # 1. Ping 测试 - 扫描 ID 1-10
        print("\n📡 扫描在线舵机...")
        found_ids = []
        for test_id in range(1, 11):
            comm_result, dxl_error, data = driver.packet_handler.read1ByteTxRx(
                driver.port_handler,
                test_id,
                driver.ADDR_ID
            )
            if comm_result == 0 and dxl_error == 0:
                print(f"   ✅ 发现舵机 ID={test_id}")
                found_ids.append(test_id)
            time.sleep(0.02)
        
        if not found_ids:
            print("   ❌ 未找到任何舵机")
            print("\n请检查:")
            print("   1. 舵机是否通电")
            print("   2. TX/RX 是否接反（尝试交换）")
            print("   3. 波特率是否正确（ST3215默认1000000）")
            return
        
        # 使用第一个找到的舵机
        SERVO_ID = found_ids[0]
        print(f"\n🎯 使用舵机 ID={SERVO_ID}")
        
        # 2. 读取当前位置
        print("\n📖 读取当前位置...")
        current_pos = driver.get_position(SERVO_ID)
        if current_pos is not None:
            print(f"   当前位置: {current_pos:.1f}°")
        else:
            print("   位置读取失败")
            current_pos = 180
        
        # 3. 设置为位置模式
        print("\n⚙️ 设置为位置模式...")
        driver.set_mode(SERVO_ID, ServoMode.POSITION)
        time.sleep(0.3)
        
        # 4. 移动到90度
        print("\n🔄 移动到90度...")
        driver.set_position(SERVO_ID, angle=90.0, time_ms=1000)
        time.sleep(1.5)
        
        new_pos = driver.get_position(SERVO_ID)
        if new_pos is not None:
            print(f"   新位置: {new_pos:.1f}°")
        
        # 5. 移动到270度
        print("\n🔄 移动到270度...")
        driver.set_position(SERVO_ID, angle=270.0, time_ms=1500)
        time.sleep(2)
        
        new_pos = driver.get_position(SERVO_ID)
        if new_pos is not None:
            print(f"   新位置: {new_pos:.1f}°")
        
        # 6. 回到初始位置
        print(f"\n🔄 回到初始位置 ({current_pos:.1f}°)...")
        driver.set_position(SERVO_ID, angle=current_pos, time_ms=1500)
        time.sleep(2)
        
        # 7. 读取状态信息
        print("\n📊 读取舵机状态...")
        temp = driver.get_temperature(SERVO_ID)
        voltage = driver.get_voltage(SERVO_ID)
        
        if temp is not None:
            print(f"   温度: {temp}°C")
        if voltage is not None:
            print(f"   电压: {voltage:.1f}V")
        
        print("\n✅ 测试完成！")
        
    except Exception as e:
        print(f"\n❌ 测试出错: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 断开连接
        driver.disconnect()
        print("\n🔌 已断开连接")


if __name__ == "__main__":
    test_servo()
