"""
快速测试舵机 - 让舵机动起来
"""
import sys
import time
from pathlib import Path

# 添加当前目录到路径
sys.path.insert(0, str(Path(__file__).parent))

from drivers.Hiwonder.lx16a_driver import LX16ADriver


def test_servo():
    """测试舵机控制"""
    print("=" * 60)
    print("🔧 舵机快速测试")
    print("=" * 60)
    
    # ⚠️ 修改为你的串口号
    PORT = "COM10"  # Windows: COM3, COM4... | Linux: /dev/ttyUSB0, /dev/ttyACM0...
    SERVO_ID = 26   # 舵机ID
    
    print(f"\n📡 连接串口: {PORT}")
    print(f"🎯 目标舵机: ID={SERVO_ID}")
    
    # 创建驱动实例
    driver = LX16ADriver(port=PORT, baudrate=115200)
    
    # 连接
    if not driver.connect():
        print("❌ 连接失败，请检查:")
        print("   1. 串口号是否正确")
        print("   2. 舵机是否通电")
        print("   3. 接线是否正确")
        return
    
    print("✅ 连接成功！")
    
    try:
        # 1. Ping 测试
        print("\n📡 测试通信...")
        if driver.ping(SERVO_ID):
            print(f"✅ 舵机 ID={SERVO_ID} 在线")
        else:
            print(f"❌ 舵机 ID={SERVO_ID} 无响应")
            print("   提示: 尝试扫描其他ID...")
            for test_id in range(1, 11):
                if driver.ping(test_id):
                    print(f"   ✅ 发现舵机 ID={test_id}")
            return
        
        # 2. 使能力矩
        print("\n⚡ 使能力矩...")
        driver.enable_torque(SERVO_ID)
        time.sleep(0.5)
        
        # 3. 读取当前位置
        print("\n📖 读取当前位置...")
        current_pos = driver.get_position(SERVO_ID)
        if current_pos is not None:
            print(f"   当前位置: {current_pos:.1f}°")
        else:
            print("   位置读取失败")
            current_pos = 0
        
        # 4. 移动到90度
        print("\n🔄 移动到90度...")
        driver.move_to_position(SERVO_ID, angle=90.0, time_ms=1000)
        time.sleep(1.5)
        
        new_pos = driver.get_position(SERVO_ID)
        if new_pos is not None:
            print(f"   新位置: {new_pos:.1f}°")
        
        # 5. 移动到180度
        print("\n🔄 移动到180度...")
        driver.move_to_position(SERVO_ID, angle=180.0, time_ms=1500)
        time.sleep(2)
        
        new_pos = driver.get_position(SERVO_ID)
        if new_pos is not None:
            print(f"   新位置: {new_pos:.1f}°")
        
        # 6. 回到初始位置
        print(f"\n🔄 回到初始位置 ({current_pos:.1f}°)...")
        driver.move_to_position(SERVO_ID, angle=current_pos, time_ms=1500)
        time.sleep(2)
        
        # 7. 失能力矩
        print("\n🔒 失能力矩...")
        driver.disable_torque(SERVO_ID)
        
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
