"""
扫描所有可用的串口
"""
import serial.tools.list_ports


def scan_ports():
    """扫描并列出所有可用串口"""
    print("=" * 60)
    print("🔍 扫描可用串口")
    print("=" * 60)
    
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("\n❌ 未找到任何串口")
        print("\n请检查:")
        print("   1. USB 转串口线是否连接")
        print("   2. 驱动是否已安装")
        print("   3. 舵机是否通电")
        return []
    
    print(f"\n✅ 发现 {len(ports)} 个串口:\n")
    
    available_ports = []
    for i, port in enumerate(ports, 1):
        print(f"{i}. {port.device}")
        print(f"   描述: {port.description}")
        print(f"   硬件ID: {port.hwid}")
        print()
        available_ports.append(port.device)
    
    print("=" * 60)
    print("💡 提示: 通常 USB 转串口的描述包含 'USB-SERIAL' 或 'CH340'")
    print("=" * 60)
    
    return available_ports


if __name__ == "__main__":
    ports = scan_ports()
    
    if ports:
        print(f"\n📝 请在 test_servo_quick.py 中修改 PORT 为以下之一:")
        for port in ports:
            print(f"   PORT = \"{port}\"")
