"""
幻尔 LX-16A 全面诊断工具
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from drivers.Hiwonder.lx16a_driver import LX16ADriver


def diagnose_lx16a():
    """全面诊断 LX-16A 连接"""
    print("=" * 70)
    print("🔧 幻尔 LX-16A 全面诊断工具")
    print("=" * 70)
    
    PORT = "COM10"
    
    # 尝试所有常见波特率
    baudrates = [
        115200,   # LX-16A 常见默认
        9600,
        57600,
        38400,
        1000000,
    ]
    
    print(f"\n📡 测试串口: {PORT}")
    print(f"🔍 将尝试 {len(baudrates)} 种波特率\n")
    
    for baudrate in baudrates:
        print(f"{'='*70}")
        print(f"⚡ 测试波特率: {baudrate:,}")
        print(f"{'='*70}")
        
        driver = LX16ADriver(port=PORT, baudrate=baudrate)
        
        if not driver.connect():
            print(f"❌ 无法打开端口")
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
            print(f"\n💡 配置信息:")
            print(f"   PORT = \"{PORT}\"")
            print(f"   BAUDRATE = {baudrate}")
            print(f"   SERVO_ID = {found_ids[0]}")
            print(f"\n请在 test_servo_quick.py 中更新这些参数")
            return True
    
    print("\n" + "=" * 70)
    print("❌ 未找到任何舵机")
    print("=" * 70)
    print("\n可能的原因:")
    print("   1. ⚡ 舵机没有外接电源（LX-16A需要7.4V-12V）")
    print("   2. 🔌 TX/RX 接线错误（尝试交换两根线）")
    print("   3. 🔗 GND 没有共地")
    print("   4. 🆔 舵机ID不在1-10范围内")
    print("   5. 💥 舵机或USB转串口损坏")
    print("\n建议操作:")
    print("   1. 确认舵机已接通7.4V-12V电源")
    print("   2. 交换 TX 和 RX 两根线")
    print("   3. 确保 GND 已连接")
    print("   4. 使用幻尔官方调试软件测试")
    print("=" * 70)
    
    return False


if __name__ == "__main__":
    diagnose_lx16a()
