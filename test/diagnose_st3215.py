"""
飞特 ST3215 全面诊断工具
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import scservo_sdk


def full_diagnosis():
    """全面诊断 ST3215 连接"""
    print("=" * 70)
    print("🔧 飞特 ST3215 全面诊断工具")
    print("=" * 70)
    
    PORT = "/dev/ttyACM0"
    
    # 尝试所有常见波特率
    baudrates = [
        1000000,  # ST3215 默认
        115200,   # 常见备用
        57600,
        38400,
        19200,
        9600,
    ]
    
    print(f"\n📡 测试串口: {PORT}")
    print(f"🔍 将尝试 {len(baudrates)} 种波特率\n")
    
    for baudrate in baudrates:
        print(f"{'='*70}")
        print(f"⚡ 测试波特率: {baudrate:,}")
        print(f"{'='*70}")
        
        try:
            # 创建端口处理器
            port_handler = scservo_sdk.PortHandler(PORT)
            packet_handler = scservo_sdk.PacketHandler(0)
            
            # 打开端口
            if not port_handler.openPort():
                print(f"❌ 无法打开端口")
                continue
            
            # 设置波特率
            if not port_handler.setBaudRate(baudrate):
                print(f"❌ 无法设置波特率")
                port_handler.closePort()
                continue
            
            print(f"✅ 串口打开成功")
            
            # 扫描 ID 1-10
            found_ids = []
            for servo_id in range(1, 11):
                # 尝试读取 ID 寄存器
                dxl_model_number, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(
                    port_handler,
                    servo_id,
                    3  # Model Number 地址
                )
                
                if dxl_comm_result == scservo_sdk.COMM_SUCCESS and dxl_error == 0:
                    print(f"   ✅ 发现舵机 ID={servo_id}, 型号={dxl_model_number}")
                    found_ids.append(servo_id)
                
                time.sleep(0.02)
            
            port_handler.closePort()
            
            if found_ids:
                print(f"\n🎉 找到 {len(found_ids)} 个舵机: {found_ids}")
                print(f"\n💡 配置信息:")
                print(f"   PORT = \"{PORT}\"")
                print(f"   BAUDRATE = {baudrate}")
                print(f"   SERVO_ID = {found_ids[0]}")
                print(f"\n请在 test_st3215_quick.py 中更新这些参数")
                return True
        
        except Exception as e:
            print(f"❌ 错误: {e}")
            continue
    
    print("\n" + "=" * 70)
    print("❌ 未找到任何舵机")
    print("=" * 70)
    print("\n可能的原因:")
    print("   1. ⚡ 舵机没有外接电源（ST3215需要7.4V-12V）")
    print("   2. 🔌 TX/RX 接线错误（尝试交换两根线）")
    print("   3. 🔗 GND 没有共地")
    print("   4. 🆔 舵机ID不在1-10范围内")
    print("   5. 💥 舵机或USB转串口损坏")
    print("\n建议操作:")
    print("   1. 确认舵机已接通7.4V-12V电源")
    print("   2. 交换 TX 和 RX 两根线")
    print("   3. 确保 GND 已连接")
    print("   4. 使用飞特官方调试软件测试")
    print("=" * 70)
    
    return False


if __name__ == "__main__":
    full_diagnosis()
