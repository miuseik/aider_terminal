# 驱动模块总览

## 目录结构

```
drivers/
├── Hiwonder/               # 幻尔科技舵机驱动
│   ├── __init__.py
│   ├── lx16a_driver.py     # LX-16A 驱动实现
│   └── example.py          # 使用示例
│
├── Feetech/                # 飞特科技舵机驱动
│   ├── __init__.py
│   ├── st3215_driver.py    # ST3215 驱动实现
│   └── example.py          # 使用示例
│
├── robstride/              # 灵足时代电机驱动
│   ├── __init__.py
│   ├── robstride_driver.py # Robstride 驱动封装
│   └── example.py          # 使用示例
│
├── camera/                 # 相机驱动
│   ├── __init__.py
│   ├── camera_driver.py    # 相机驱动实现
│   └── example.py          # 使用示例
│
└── docs/                   # 文档目录
    ├── DRIVERS_OVERVIEW.md            # 本文档
    ├── SERVO_DETECTOR_README.md       # 舵机探测工具说明
    ├── Hiwonder_README.md             # Hiwonder 驱动说明
    ├── Feetech_README.md              # Feetech 驱动说明
    ├── Robstride_README.md            # Robstride 驱动说明
    ├── Robstride_INSTALLATION.md      # Robstride 安装指南
    ├── Robstride_HOW_TO_ADD_NEW_MOTORS.md  # Robstride 添加新电机指南
    └── Camera_README.md               # Camera 驱动说明
```

## 核心功能

两个驱动都实现了相同的4个核心功能接口：

### 1. 设置ID
- `set_id(old_id, new_id)` - 修改舵机ID

### 2. 设置模式
- `set_mode(servo_id, mode)` - 设置工作模式
  - `ServoMode.POSITION` - 位置模式（角度控制）
  - `ServoMode.SPEED` - 速度模式（转速控制）

### 3. 控制转速或角度

#### 位置模式
- `set_position(servo_id, angle, time_ms)` - 设置目标角度
- `move_to_position(servo_id, angle, time_ms)` - 便捷方法（自动切换模式）

#### 速度模式
- `set_speed(servo_id, speed)` - 设置旋转速度
- `rotate_at_speed(servo_id, speed)` - 便捷方法（自动切换模式）

### 4. 读取舵机数据
- `get_position(servo_id)` - 读取当前位置
- `get_speed(servo_id)` - 读取当前速度
- `get_temperature(servo_id)` - 读取温度
- `get_voltage(servo_id)` - 读取电压
- `get_status(servo_id)` - 读取状态
- `get_all_data(servo_id)` - 读取所有数据

## 技术对比

| 特性 | Hiwonder LX-16A | Feetech ST3215 | Robstride |
|------|------------------|----------------|-----------|
| **通信方式** | UART串口 | UART串口 | CAN总线 |
| **平台支持** | Windows/Linux | Windows/Linux | Linux only |
| **位置范围** | 0-1000 (0-240°) | 0-4095 (0-360°) | ±2.79/±1.57 rad |
| **速度范围** | -1000 ~ 1000 | -1023 ~ 1023 | ±33/±50 rad/s |
| **力矩反馈** | ❌ | ❌ | ✅ |
| **默认波特率** | 115200 | 1000000 (1M) | 1000000 (CAN) |
| **依赖库** | pyserial（内置） | feetech-servo-sdk | el_a3_sdk |

## 快速开始

### Hiwonder LX-16A

```python
from drivers.Hiwonder import LX16ADriver, ServoMode

# 创建驱动
driver = LX16ADriver(port='COM3', baudrate=115200)
driver.connect()

# 设置位置模式并移动到90度
driver.move_to_position(servo_id=1, angle=90.0, time_ms=1000)

# 读取位置
angle = driver.get_position(servo_id=1)
print(f"当前位置: {angle:.1f}°")

driver.disconnect()
```

### Feetech ST3215

```python
from drivers.Feetech import ST3215Driver, ServoMode

# 创建驱动
driver = ST3215Driver(port='COM3', baudrate=1000000)
driver.connect()

# 设置位置模式并移动到90度
driver.move_to_position(servo_id=1, angle=90.0)

# 读取位置
angle = driver.get_position(servo_id=1)
print(f"当前位置: {angle:.1f}°")

driver.disconnect()
```

### Robstride 灵足电机

```python
from drivers.robstride import RobstrideDriver, RunMode
import math

# 创建驱动 (Linux only)
driver = RobstrideDriver(can_name="can0")
driver.connect()

# 使能电机
driver.enable_motor(motor_id=1)

# 设置位置模式并移动到90度
driver.move_to_position(motor_id=1, position=math.pi/2)

# 读取位置
data = driver.get_observation(motor_id=1)
print(f"当前位置: {data['position']:.3f} rad")
print(f"当前力矩: {data['torque']:.3f} Nm")

driver.disable_motor(motor_id=1)
driver.disconnect()
```

### Camera 相机

```python
from drivers.camera import OpenCVCameraDriver

# 配置摄像头
config = {
    'index_or_path': '/dev/video0',
    'width': 1280,
    'height': 480,
    'fps': 30,
    'fourcc': 'MJPG'
}

# 创建并连接
camera = OpenCVCameraDriver(config)
camera.connect()

# 读取帧
frame = camera.read()        # 同步读取
frame = camera.read_latest() # 异步读取

# 断开连接
camera.disconnect()
```

## 运行示例

### Hiwonder 示例
```bash
cd drivers/Hiwonder
python example.py
```

### Feetech 示例
```bash
cd drivers/Feetech
python example.py
```

### Robstride 示例
```bash
cd drivers/robstride
python example.py
```

### Camera 示例
```bash
cd drivers/camera
python example.py
```

## 辅助功能

所有驱动都提供以下辅助方法：
- `enable_motor(servo_id)` / `enable_torque(servo_id)` - 使能力矩
- `disable_motor(servo_id)` / `disable_torque(servo_id)` - 失能力矩
- `ping(servo_id)` - 测试舵机是否在线
- `connect()` - 连接通信接口
- `disconnect()` - 断开连接

## 注意事项

1. **串口号**: Windows使用 `COM3`, Linux使用 `/dev/ttyUSB0`
2. **CAN接口**: Robstride需要Linux SocketCAN，使用前需配置: `sudo ip link set can0 up type can bitrate 1000000`
3. **波特率**: 确保与舵机实际波特率一致
4. **舵机ID**: 每个舵机必须有唯一的ID
5. **力矩控制**: 操作前建议先使能力矩，完成后失能
6. **错误处理**: 所有方法都有返回值，建议检查是否成功
7. **单位差异**: Robstride使用弧度制，Hiwonder/Feetech使用角度制
