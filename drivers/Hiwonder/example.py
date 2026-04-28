"""
Hiwonder 幻尔 LX-16A 驱动使用示例
演示4个核心功能的使用
"""

import sys
import time
from lx16a_driver import LX16ADriver, ServoMode


def example_set_id():
    """示例1: 设置舵机ID"""
    print("\n=== 示例1: 设置舵机ID ===")
    
    driver = LX16ADriver(port='COM3', baudrate=115200)
    
    if not driver.connect():
        print("连接失败")
        return
    
    # 将ID为1的舵机改为ID为2
    success = driver.set_id(old_id=1, new_id=2)
    
    if success:
        print("✅ ID设置成功")
    else:
        print("❌ ID设置失败")
    
    driver.disconnect()


def example_set_mode():
    """示例2: 设置工作模式"""
    print("\n=== 示例2: 设置工作模式 ===")
    
    driver = LX16ADriver(port='COM3', baudrate=115200)
    
    if not driver.connect():
        print("连接失败")
        return
    
    servo_id = 1
    
    # 设置为位置模式
    driver.set_mode(servo_id, ServoMode.POSITION)
    print("已设置为位置模式")
    
    time.sleep(0.5)
    
    # 设置为速度模式
    driver.set_mode(servo_id, ServoMode.SPEED)
    print("已设置为速度模式")
    
    driver.disconnect()


def example_control_position():
    """示例3: 控制角度（位置模式）"""
    print("\n=== 示例3: 控制角度 ===")
    
    driver = LX16ADriver(port='COM3', baudrate=115200)
    
    if not driver.connect():
        print("连接失败")
        return
    
    servo_id = 1
    
    # 使能力矩
    driver.enable_torque(servo_id)
    
    # 移动到90度，用时1秒
    driver.move_to_position(servo_id, angle=90.0, time_ms=1000)
    print("正在移动到90度...")
    time.sleep(1.5)
    
    # 移动到180度，用时2秒
    driver.move_to_position(servo_id, angle=180.0, time_ms=2000)
    print("正在移动到180度...")
    time.sleep(2.5)
    
    # 读取当前位置
    current_angle = driver.get_position(servo_id)
    if current_angle is not None:
        print(f"当前位置: {current_angle:.1f}°")
    
    driver.disable_torque(servo_id)
    driver.disconnect()


def example_control_speed():
    """示例4: 控制转速（速度模式）"""
    print("\n=== 示例4: 控制转速 ===")
    
    driver = LX16ADriver(port='COM3', baudrate=115200)
    
    if not driver.connect():
        print("连接失败")
        return
    
    servo_id = 1
    
    # 使能力矩
    driver.enable_torque(servo_id)
    
    # 以500的速度正转2秒
    driver.rotate_at_speed(servo_id, speed=500)
    print("正转中...")
    time.sleep(2)
    
    # 停止
    driver.rotate_at_speed(servo_id, speed=0)
    print("停止")
    time.sleep(0.5)
    
    # 以300的速度反转2秒
    driver.rotate_at_speed(servo_id, speed=-300)
    print("反转中...")
    time.sleep(2)
    
    # 停止
    driver.rotate_at_speed(servo_id, speed=0)
    print("停止")
    
    driver.disable_torque(servo_id)
    driver.disconnect()


def example_read_data():
    """示例5: 读取舵机数据"""
    print("\n=== 示例5: 读取舵机数据 ===")
    
    driver = LX16ADriver(port='COM3', baudrate=115200)
    
    if not driver.connect():
        print("连接失败")
        return
    
    servo_id = 1
    
    # 读取所有数据
    data = driver.get_all_data(servo_id)
    
    if data:
        print(f"\n舵机ID: {data['id']}")
        print(f"位置: {data['position']:.1f}°" if data['position'] else "位置: 读取失败")
        print(f"速度: {data['speed']}" if data['speed'] else "速度: 读取失败")
        print(f"温度: {data['temperature']:.1f}°C" if data['temperature'] else "温度: 读取失败")
        print(f"电压: {data['voltage']:.2f}V" if data['voltage'] else "电压: 读取失败")
        
        if data['status']:
            status = data['status']
            print(f"状态: {'正常' if status['normal'] else '异常'}")
            if not status['normal']:
                if status['overheating']:
                    print("  ⚠️ 过热警告")
                if status['overloaded']:
                    print("  ⚠️ 过载警告")
    
    driver.disconnect()


def example_ping():
    """示例6: 扫描在线舵机"""
    print("\n=== 示例6: 扫描在线舵机 ===")
    
    driver = LX16ADriver(port='COM3', baudrate=115200)
    
    if not driver.connect():
        print("连接失败")
        return
    
    print("扫描ID 1-10的舵机...")
    online_ids = []
    
    for servo_id in range(1, 11):
        if driver.ping(servo_id):
            online_ids.append(servo_id)
            print(f"  ✅ 发现舵机 ID={servo_id}")
        time.sleep(0.05)
    
    print(f"\n共发现 {len(online_ids)} 个舵机: {online_ids}")
    
    driver.disconnect()


if __name__ == "__main__":
    print("=" * 60)
    print("Hiwonder 幻尔 LX-16A 驱动示例")
    print("=" * 60)
    
    # 根据需要注释/取消注释要运行的示例
    example_set_id()
    example_set_mode()
    example_control_position()
    example_control_speed()
    example_read_data()
    example_ping()
    
    print("\n" + "=" * 60)
    print("所有示例执行完毕")
    print("=" * 60)
