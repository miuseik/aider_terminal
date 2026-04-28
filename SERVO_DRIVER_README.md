# 总线舵机驱动使用说明

## 📋 概述

本项目支持两种总线舵机：
- **LX-16A** (幻尔科技)
- **ST3215** (飞特科技)

## 🔧 安装依赖

### ST3215 舵机

```bash
pip install feetech-servo-sdk
```

### LX-16A 舵机

LX-16A 驱动已内置，无需额外安装。只需确保安装了 `pyserial`：

```bash
pip install pyserial
```

## ⚙️ 配置

编辑 `config.yaml` 文件：

```yaml
robot:
  left_arm:
    port: COM3              # Windows 串口号，Linux 为 /dev/ttyUSB0
    servo_type: st3215      # lx16a 或 st3215
    baudrate: 1000000       # ST3215: 1000000, LX-16A: 115200
  
  right_arm:
    port: COM4
    servo_type: st3215
    baudrate: 1000000
```

## 🧪 测试舵机

运行测试脚本验证舵机连接：

```bash
# 测试 ST3215
python test_servo.py st3215 COM3

# 测试 LX-16A
python test_servo.py lx16a COM3
```

## 📖 API 使用示例

### 创建舵机驱动

```python
from drivers.bus_servo_driver import create_servo_driver, ServoType

# 创建 LX-16A 驱动
driver = create_servo_driver(
    servo_type=ServoType.LX16A,
    port="COM3",
    baudrate=115200
)

# 创建 ST3215 驱动
driver = create_servo_driver(
    servo_type=ServoType.ST3215,
    port="COM3",
    baudrate=1000000
)
```

### 连接舵机

```python
if driver.connect():
    print("✅ 连接成功")
else:
    print("❌ 连接失败")
```

### 发送控制指令

```python
# 关节角度字典
action = {
    'shoulder_pan.pos': 45.0,     # 肩关节旋转 45度
    'shoulder_lift.pos': 30.0,    # 肩关节抬升 30度
    'elbow_flex.pos': 60.0,       # 肘关节弯曲 60度
    'wrist_flex.pos': 0.0,        # 腕关节 0度
    'wrist_roll.pos': 0.0,        # 腕关节旋转 0度
    'gripper.pos': 20.0           # 夹爪开合 20度
}

# 发送指令（50ms内完成运动）
driver.send_action(action, time_ms=50)
```

### 读取当前位置

```python
observation = driver.get_observation()
print(observation)
# 输出: {'shoulder_pan.pos': 44.5, 'shoulder_lift.pos': 29.8, ...}
```

### 断开连接

```python
driver.disconnect()
```

## 🔍 故障排查

### 问题1：串口无法打开

**Windows:**
- 检查设备管理器中的 COM 端口号
- 确保没有其他程序占用该端口
- 尝试以管理员权限运行

**Linux:**
- 检查权限：`ls -l /dev/ttyUSB*`
- 添加用户到 dialout 组：`sudo usermod -a -G dialout $USER`
- 重新登录使权限生效

### 问题2：舵机不响应

1. 检查波特率是否正确
   - LX-16A: 115200
   - ST3215: 1000000

2. 检查舵机 ID 是否正确（默认 1-6）

3. 检查电源是否充足（建议 7.4V 以上）

4. 使用官方调试软件测试舵机

### 问题3：通信超时

- 检查串口线连接
- 尝试降低波特率
- 增加超时时间

## 📊 技术细节

### LX-16A 协议

- **帧头**: 0x55 0x55
- **位置范围**: 0-1000 (对应 0-240°)
- **校验和**: 取反求和
- **通信方式**: 半双工串口

### ST3215 协议

- **SDK**: feetech-servo-sdk
- **位置范围**: 0-4095 (对应 0-360°)
- **通信方式**: 半双工串口
- **波特率**: 最高 1Mbps

## 🎯 下一步

现在您已经完成了舵机驱动的集成，可以：

1. 启动遥操作系统
2. 通过 Web 界面控制机器人
3. 调整 PID 参数优化性能
4. 添加更多功能（如力反馈、轨迹规划等）

祝使用愉快！🚀
