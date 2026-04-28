#!/usr/bin/env python3
"""测试 Jetson Nano SDK（去除 GPIO 依赖）"""

import sys
import time
from pathlib import Path

# 添加 SDK 路径
sdk_path = Path(__file__).parent.parent / "drivers" / "Hiwonder" / "06 Jetson Nano版本" / "程序文件" / "所需库文件"
sys.path.insert(0, str(sdk_path))

# 修改 SDK，移除 GPIO 依赖
from sdk import hiwonder_servo_controller
servo_ctrl = hiwonder_servo_controller

# 覆盖 GPIO 函数为空操作
def dummy_func():
    pass

servo_ctrl.port_as_write = dummy_func
servo_ctrl.port_as_read = dummy_func
servo_ctrl.port_init = dummy_func

# 创建控制器
print("连接 /dev/ttyACM0 @ 115200...")
controller = servo_ctrl.HiwonderServoController('/dev/ttyACM0', 115200)
print("✅ 连接成功\n")

try:
    # 1. 读取 ID
    print("🔍 读取舵机ID...")
    servo_id = controller.get_servo_id()
    if servo_id:
        print(f"✅ 舵机ID: {servo_id}\n")
    else:
        print("❌ 未找到舵机\n")
        sys.exit(1)
    
    # 2. 读取位置
    print("📍 读取当前位置...")
    pos = controller.get_servo_position(servo_id)
    if pos is not None:
        angle = (pos / 1000.0) * 240
        print(f"  脉冲: {pos}, 角度: {angle:.1f}°\n")
    
    # 3. 移动到 200
    print("🔄 移动到 200 (约48°)...")
    controller.set_servo_position(servo_id, 200, 1000)
    time.sleep(1.2)
    
    pos = controller.get_servo_position(servo_id)
    if pos is not None:
        angle = (pos / 1000.0) * 240
        print(f"  ✅ 新位置: {pos} ({angle:.1f}°)\n")
    
    # 4. 移动到 800
    print("🔄 移动到 800 (约192°)...")
    controller.set_servo_position(servo_id, 800, 1000)
    time.sleep(1.2)
    
    pos = controller.get_servo_position(servo_id)
    if pos is not None:
        angle = (pos / 1000.0) * 240
        print(f"  ✅ 新位置: {pos} ({angle:.1f}°)\n")
    
    # 5. 回到 500
    print("🔄 回到 500 (120°)...")
    controller.set_servo_position(servo_id, 500, 1000)
    time.sleep(1.2)
    
    pos = controller.get_servo_position(servo_id)
    if pos is not None:
        angle = (pos / 1000.0) * 240
        print(f"  ✅ 最终位置: {pos} ({angle:.1f}°)\n")
    
    print("✅ 测试完成！")

except Exception as e:
    print(f"❌ 错误: {e}")
    import traceback
    traceback.print_exc()

finally:
    controller.close()
    print("🔌 已断开")
