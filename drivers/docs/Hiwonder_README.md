# Hiwonder 幻尔总线舵机驱动

## 概述

LX-16A 总线舵机的Python驱动，基于官方通信协议实现。

**品牌说明**：Hiwonder（幻尔科技）是 LX-16A 舵机的制造商。

## 核心功能

### 1. 设置ID
```python
from lx16a_driver import LX16ADriver

driver = LX16ADriver(port='COM3', baudrate=115200)
driver.connect()

# 将ID为1的舵机改为ID为2
driver.set_id(old_id=1, new_id=2)
```

### 2. 设置模式
```python
from lx16a_driver import LX16ADriver, ServoMode

# 位置模式 - 用于精确角度控制
driver.set_mode(servo_id=1, mode=ServoMode.POSITION)

# 速度模式 - 用于连续旋转
driver.set_mode(servo_id=1, mode=ServoMode.SPEED)
```

### 3. 控制转速或角度

#### 位置模式 - 控制角度
```python
# 移动到90度，用时1秒
driver.move_to_position(servo_id=1, angle=90.0, time_ms=1000)

# 或直接使用底层方法
driver.set_position(servo_id=1, angle=180.0, time_ms=2000)
```

#### 速度模式 - 控制转速
```python
# 以500的速度正转
driver.rotate_at_speed(servo_id=1, speed=500)

# 以300的速度反转
driver.rotate_at_speed(servo_id=1, speed=-300)

# 停止
driver.rotate_at_speed(servo_id=1, speed=0)
```

### 4. 读取舵机数据
```python
# 读取位置
angle = driver.get_position(servo_id=1)

# 读取速度
speed = driver.get_speed(servo_id=1)

# 读取温度
temp = driver.get_temperature(servo_id=1)

# 读取电压
voltage = driver.get_voltage(servo_id=1)

# 读取状态
status = driver.get_status(servo_id=1)

# 读取所有数据
data = driver.get_all_data(servo_id=1)
print(f"位置: {data['position']}°")
print(f"温度: {data['temperature']}°C")
print(f"电压: {data['voltage']}V")
```

## 技术规格

- **位置范围**: 0-1000 (对应0-240度)
- **速度范围**: -1000 ~ 1000
- **默认波特率**: 115200
- **ID范围**: 1-253

## 辅助功能

```python
# 使能力矩
driver.enable_torque(servo_id=1)

# 失能力矩
driver.disable_torque(servo_id=1)

# 测试舵机是否在线
if driver.ping(servo_id=1):
    print("舵机在线")
```

## 完整示例

查看 `example.py` 文件获取完整的使用示例。

运行示例：
```bash
cd drivers/LewanSoul
python example.py
```
