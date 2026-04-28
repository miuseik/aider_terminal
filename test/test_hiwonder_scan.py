#!/usr/bin/env python3
"""扫描幻尔舵机ID和波特率"""

import serial
import time

CMD_ID_READ = 14

def checksum(buf):
    s = sum(buf)
    s = s - 0x55 - 0x55
    s = ~s
    return s & 0xff

def scan_id(ser, baudrate):
    """扫描所有可能的ID"""
    print(f"\n{'='*60}")
    print(f"扫描波特率: {baudrate}")
    print(f"{'='*60}")
    
    for servo_id in range(1, 254):
        # 构建读取ID命令
        buf = bytearray([0x55, 0x55])
        buf.append(servo_id)
        buf.append(3)  # 长度
        buf.append(CMD_ID_READ)
        buf.append(checksum(buf))
        
        ser.write(buf)
        time.sleep(0.005)
        
        count = ser.inWaiting()
        if count > 0:
            recv_data = ser.read(count)
            if len(recv_data) >= 6 and recv_data[0] == 0x55 and recv_data[1] == 0x55:
                print(f"✅ 找到舵机! ID={servo_id}, 响应: {[hex(b) for b in recv_data]}")
                return servo_id
    
    print("❌ 未找到舵机")
    return None

# 尝试不同波特率
baudrates = [115200, 9600, 57600, 38400, 19200, 1000000]

for baudrate in baudrates:
    try:
        print(f"\n尝试波特率 {baudrate}...")
        ser = serial.Serial('/dev/ttyACM0', baudrate, timeout=0.1)
        time.sleep(0.2)
        
        found_id = scan_id(ser, baudrate)
        ser.close()
        
        if found_id is not None:
            print(f"\n🎉 成功! 波特率={baudrate}, ID={found_id}")
            break
    except Exception as e:
        print(f"错误: {e}")
