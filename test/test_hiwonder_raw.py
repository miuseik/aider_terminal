#!/usr/bin/env python3
"""测试幻尔舵机原始通信"""

import serial
import time

# 协议常量
FRAME_HEADER = 0x55
CMD_MOVE_TIME_WRITE = 1
CMD_POS_READ = 28
CMD_TEMP_READ = 26
CMD_VIN_READ = 27

def checksum(buf):
    """计算校验和"""
    s = sum(buf)
    s = s - 0x55 - 0x55  # 去掉两个帧头
    s = ~s  # 取反
    return s & 0xff

def build_cmd(servo_id, cmd, dat1=None, dat2=None):
    """构建命令帧"""
    buf = bytearray([0x55, 0x55])  # 帧头
    buf.append(servo_id)
    
    # 指令长度
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)
    
    buf.append(cmd)  # 指令
    
    # 数据
    if dat1 is not None and dat2 is not None:
        buf.extend([dat1 & 0xff, (dat1 >> 8) & 0xff])
        buf.extend([dat2 & 0xff, (dat2 >> 8) & 0xff])
    elif dat1 is not None:
        buf.append(dat1 & 0xff)
    
    buf.append(checksum(buf))  # 校验和
    return buf

def read_response(ser, expected_cmd):
    """读取响应"""
    ser.flushInput()
    time.sleep(0.005)
    count = ser.inWaiting()
    
    if count > 0:
        recv_data = ser.read(count)
        print(f"收到 {len(recv_data)} 字节: {[hex(b) for b in recv_data]}")
        
        if len(recv_data) >= 5 and recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == expected_cmd:
            data_len = recv_data[3]
            print(f"  长度字段: {data_len}")
            
            if data_len == 4:  # 单字节数据（温度、ID等）
                return recv_data[5]
            elif data_len == 5:  # 双字节数据（位置、电压等）
                pos = recv_data[5] | (recv_data[6] << 8)
                # 转换为有符号整数
                if pos > 32767:
                    pos -= 65536
                return pos
        else:
            print(f"  ❌ 响应格式错误")
    
    return None

# 连接串口
print("连接 /dev/ttyACM0 @ 115200...")
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)

try:
    servo_id = 1
    
    # 1. 读取位置
    print("\n📍 读取位置...")
    cmd = build_cmd(servo_id, CMD_POS_READ)
    print(f"发送: {[hex(b) for b in cmd]}")
    ser.write(cmd)
    time.sleep(0.01)
    
    pos = read_response(ser, CMD_POS_READ)
    if pos is not None:
        angle = (pos / 1000.0) * 240
        print(f"✅ 位置: {pos} (角度: {angle:.1f}°)")
    
    # 2. 读取温度
    print("\n🌡️  读取温度...")
    cmd = build_cmd(servo_id, CMD_TEMP_READ)
    ser.write(cmd)
    time.sleep(0.01)
    
    temp = read_response(ser, CMD_TEMP_READ)
    if temp is not None:
        print(f"✅ 温度: {temp}°C")
    
    # 3. 读取电压
    print("\n⚡ 读取电压...")
    cmd = build_cmd(servo_id, CMD_VIN_READ)
    ser.write(cmd)
    time.sleep(0.01)
    
    vin = read_response(ser, CMD_VIN_READ)
    if vin is not None:
        voltage = vin / 100.0
        print(f"✅ 电压: {vin} ({voltage:.2f}V)")
    
    # 4. 移动到 500 位置（中位）
    print("\n🔄 移动到位置 500 (用时1000ms)...")
    cmd = build_cmd(servo_id, CMD_MOVE_TIME_WRITE, 500, 1000)
    print(f"发送: {[hex(b) for b in cmd]}")
    ser.write(cmd)
    time.sleep(1.1)
    
    # 5. 再次读取位置
    print("\n📍 读取新位置...")
    cmd = build_cmd(servo_id, CMD_POS_READ)
    ser.write(cmd)
    time.sleep(0.01)
    
    pos = read_response(ser, CMD_POS_READ)
    if pos is not None:
        angle = (pos / 1000.0) * 240
        print(f"✅ 位置: {pos} (角度: {angle:.1f}°)")
    
    print("\n✅ 测试完成！")

except Exception as e:
    print(f"❌ 错误: {e}")
finally:
    ser.close()
    print("🔌 已断开")
