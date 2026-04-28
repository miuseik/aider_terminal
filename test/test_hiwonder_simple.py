#!/usr/bin/env python3
"""简单测试幻尔舵机移动"""

import serial
import time

def checksum(buf):
    s = sum(buf)
    s = s - 0x55 - 0x55
    s = ~s
    return s & 0xff

def move_servo(ser, servo_id, pulse, time_ms):
    """移动舵机到指定位置"""
    buf = bytearray([0x55, 0x55])
    buf.append(servo_id)
    buf.append(7)  # 长度（ID + CMD + 4字节数据）
    buf.append(1)  # CMD_MOVE_TIME_WRITE
    buf.extend([pulse & 0xff, (pulse >> 8) & 0xff])  # 脉冲值
    buf.extend([time_ms & 0xff, (time_ms >> 8) & 0xff])  # 时间
    buf.append(checksum(buf))
    
    print(f"发送: {[hex(b) for b in buf]}")
    ser.write(buf)

def read_pos(ser, servo_id):
    """读取位置"""
    buf = bytearray([0x55, 0x55])
    buf.append(servo_id)
    buf.append(3)
    buf.append(28)  # CMD_POS_READ
    buf.append(checksum(buf))
    
    ser.flushInput()
    ser.write(buf)
    time.sleep(0.01)
    
    count = ser.inWaiting()
    if count > 0:
        recv = ser.read(count)
        if len(recv) >= 7 and recv[0] == 0x55 and recv[1] == 0x55:
            pulse = recv[5] | (recv[6] << 8)
            angle = (pulse / 1000.0) * 240
            return pulse, angle
    return None, None

# 连接
print("连接 /dev/ttyACM0 @ 115200...")
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)

try:
    servo_id = 1
    
    # 读取初始位置
    print("\n📍 初始位置:")
    pulse, angle = read_pos(ser, servo_id)
    if pulse is not None:
        print(f"  脉冲: {pulse}, 角度: {angle:.1f}°")
    
    # 移动到 200
    print("\n🔄 移动到 200 (约48°)...")
    move_servo(ser, servo_id, 200, 1000)
    time.sleep(1.2)
    
    pulse, angle = read_pos(ser, servo_id)
    if pulse is not None:
        print(f"  ✅ 脉冲: {pulse}, 角度: {angle:.1f}°")
    
    # 移动到 800
    print("\n🔄 移动到 800 (约192°)...")
    move_servo(ser, servo_id, 800, 1000)
    time.sleep(1.2)
    
    pulse, angle = read_pos(ser, servo_id)
    if pulse is not None:
        print(f"  ✅ 脉冲: {pulse}, 角度: {angle:.1f}°")
    
    # 回到 500
    print("\n🔄 回到 500 (120°)...")
    move_servo(ser, servo_id, 500, 1000)
    time.sleep(1.2)
    
    pulse, angle = read_pos(ser, servo_id)
    if pulse is not None:
        print(f"  ✅ 脉冲: {pulse}, 角度: {angle:.1f}°")
    
    print("\n✅ 测试完成！")

finally:
    ser.close()
