"""
Robstride 灵足电机驱动使用示例
演示4个核心功能的使用

注意：此示例需要在 Linux 系统上运行，并配置好 SocketCAN
"""

import sys
import time


def check_sdk_available():
    """检查SDK是否可用"""
    try:
        from robstride_driver import RobstrideDriver
        return True
    except ImportError as e:
        print(f"❌ SDK 不可用: {e}")
        print("\n请先安装 el_a3_sdk:")
        print("  cd /path/to/EDULITE_A3/el_a3_sdk")
        print("  pip install -e .")
        return False


def example_set_id():
    """示例1: 设置电机ID"""
    print("\n=== 示例1: 设置电机ID ===")
    
    from robstride_driver import RobstrideDriver
    
    driver = RobstrideDriver(can_name="can0")
    
    if not driver.connect():
        print("连接失败")
        return
    
    # 将ID为1的电机改为ID为2
    success = driver.set_id(old_id=1, new_id=2)
    
    if success:
        print("✅ ID设置成功")
    else:
        print("❌ ID设置失败")
    
    driver.disconnect()


def example_set_mode():
    """示例2: 设置工作模式"""
    print("\n=== 示例2: 设置工作模式 ===")
    
    from robstride_driver import RobstrideDriver, RunMode
    
    driver = RobstrideDriver(can_name="can0")
    
    if not driver.connect():
        print("连接失败")
        return
    
    motor_id = 1
    
    # 设置为位置模式
    driver.set_mode(motor_id, RunMode.POSITION_PP)
    print("已设置为位置模式 (PP)")
    
    time.sleep(0.5)
    
    # 设置为速度模式
    driver.set_mode(motor_id, RunMode.VELOCITY)
    print("已设置为速度模式")
    
    driver.disconnect()


def example_control_position():
    """示例3: 控制角度（位置模式）"""
    print("\n=== 示例3: 控制角度 ===")
    
    from robstride_driver import RobstrideDriver, RunMode
    
    driver = RobstrideDriver(can_name="can0")
    
    if not driver.connect():
        print("连接失败")
        return
    
    motor_id = 1
    
    # 使能电机
    driver.enable_motor(motor_id)
    
    # 移动到90度 (π/2 rad)
    import math
    driver.move_to_position(motor_id, position=math.pi/2)
    print("正在移动到 90° (π/2 rad)...")
    time.sleep(1.5)
    
    # 移动到180度 (π rad)
    driver.move_to_position(motor_id, position=math.pi)
    print("正在移动到 180° (π rad)...")
    time.sleep(1.5)
    
    # 读取当前位置
    current_pos = driver.get_position(motor_id)
    if current_pos is not None:
        print(f"当前位置: {current_pos:.3f} rad ({math.degrees(current_pos):.1f}°)")
    
    # 失能电机
    driver.disable_motor(motor_id)
    driver.disconnect()


def example_control_speed():
    """示例4: 控制转速（速度模式）"""
    print("\n=== 示例4: 控制转速 ===")
    
    from robstride_driver import RobstrideDriver
    
    driver = RobstrideDriver(can_name="can0")
    
    if not driver.connect():
        print("连接失败")
        return
    
    motor_id = 1
    
    # 使能电机
    driver.enable_motor(motor_id)
    
    # 以1 rad/s的速度正转2秒
    driver.rotate_at_speed(motor_id, speed=1.0)
    print("正转中 (1 rad/s)...")
    time.sleep(2)
    
    # 停止
    driver.rotate_at_speed(motor_id, speed=0.0)
    print("停止")
    time.sleep(0.5)
    
    # 以0.5 rad/s的速度反转2秒
    driver.rotate_at_speed(motor_id, speed=-0.5)
    print("反转中 (-0.5 rad/s)...")
    time.sleep(2)
    
    # 停止
    driver.rotate_at_speed(motor_id, speed=0.0)
    print("停止")
    
    # 失能电机
    driver.disable_motor(motor_id)
    driver.disconnect()


def example_read_data():
    """示例5: 读取电机数据"""
    print("\n=== 示例5: 读取电机数据 ===")
    
    from robstride_driver import RobstrideDriver
    
    driver = RobstrideDriver(can_name="can0")
    
    if not driver.connect():
        print("连接失败")
        return
    
    motor_id = 1
    
    # 等待反馈
    time.sleep(0.5)
    
    # 读取所有数据
    data = driver.get_observation(motor_id)
    
    if data:
        import math
        print(f"\n电机ID: {data['motor_id']}")
        print(f"位置: {data['position']:.3f} rad ({math.degrees(data['position']):.1f}°)")
        print(f"速度: {data['velocity']:.3f} rad/s")
        print(f"力矩: {data['torque']:.3f} Nm")
        print(f"温度: {data['temperature']:.1f} °C")
    else:
        print("未收到电机反馈，请检查:")
        print("  1. CAN接口是否正确配置")
        print("  2. 电机是否已使能")
        print("  3. 电机ID是否正确")
    
    driver.disconnect()


def example_ping():
    """示例6: 扫描在线电机"""
    print("\n=== 示例6: 扫描在线电机 ===")
    
    from robstride_driver import RobstrideDriver
    
    driver = RobstrideDriver(can_name="can0")
    
    if not driver.connect():
        print("连接失败")
        return
    
    print("扫描ID 1-7的电机...")
    online_ids = []
    
    for motor_id in range(1, 8):
        if driver.ping(motor_id):
            online_ids.append(motor_id)
            print(f"  ✅ 发现电机 ID={motor_id}")
        time.sleep(0.05)
    
    print(f"\n共发现 {len(online_ids)} 个电机: {online_ids}")
    
    driver.disconnect()


if __name__ == "__main__":
    print("=" * 60)
    print("Robstride 灵足电机驱动示例")
    print("=" * 60)
    
    # 检查SDK
    if not check_sdk_available():
        sys.exit(1)
    
    print("\n⚠️  注意: 此示例需要在 Linux 系统上运行")
    print("   并确保 CAN 接口已配置: sudo ip link set can0 up type can bitrate 1000000")
    
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
