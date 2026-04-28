"""
舵机诊断工具 - 全面检测连接问题
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from drivers.Hiwonder.lx16a_driver import LX16ADriver


def diagnose():
    """全面诊断"""
    print("=" * 60)
    print("🔧 LX-16A 舵机诊断工具")
    print("=" * 60)
    
    PORT = "COM9"
    
    print(f"\n📡 测试串口: {PORT}")
    
    # 尝试不同波特率
    baudrates = [115200, 9600, 57600, 1000000]
    
    for baudrate in baudrates:
        print(f"\n{'='*60}")
        print(f"🔍 测试波特率: {baudrate}")
        print(f"{'='*60}")
        
        driver = LX16ADriver(port=PORT, baudrate=baudrate)
        
        if not driver.connect():
            print(f"❌ 无法打开串口")
            continue
        
        print(f"✅ 串口打开成功")
        
        # 扫描 ID 1-10
        found_ids = []
        for servo_id in range(1, 11):
            if driver.ping(servo_id):
                print(f"   ✅ 发现舵机 ID={servo_id}")
                found_ids.append(servo_id)
            time.sleep(0.05)
        
        driver.disconnect()
        
        if found_ids:
            print(f"\n🎉 找到 {len(found_ids)} 个舵机: {found_ids}")
            print(f"\n💡 请在 test_servo_quick.py 中设置:")
            print(f"   PORT = \"{PORT}\"")
            print(f"   SERVO_ID = {found_ids[0]}  # 或其他ID")
            return
    
    print("\n❌ 未找到任何舵机")
    print("\n请检查:")
    print("   1. 舵机是否通电（通常需要7.4V-12V）")
    print("   2. USB转串口线是否正确连接")
    print("   3. TX/RX 是否接反（尝试交换）")
    print("   4. 舵机ID是否在1-10范围内")
    print("   5. 舵机是否损坏")


if __name__ == "__main__":
    diagnose()
