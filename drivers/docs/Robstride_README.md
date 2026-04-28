# Robstride 灵足总线电机驱动

## 概述

灵足科技 Robstride 电机的Python驱动，基于官方 `el_a3_sdk` 封装。

**品牌说明**：Robstride（灵足时代）是高性能关节电机的制造商。

**官方仓库**: https://gitee.com/robstride/EDULITE_A3

## 系统要求

- **操作系统**: Linux (需要 SocketCAN 支持)
- **CAN适配器**: CANdle / gs_usb 兼容设备
- **Python依赖**: el_a3_sdk

## 安装

### 1. 配置 CAN 接口

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 2. 安装 el_a3_sdk

```bash
cd /path/to/EDULITE_A3/el_a3_sdk
pip install -e .
```

## 核心功能

### 1. 设置ID
```python
from drivers.robstride import RobstrideDriver

driver = RobstrideDriver(can_name="can0")
driver.connect()

# 将ID为1的电机改为ID为2
driver.set_id(old_id=1, new_id=2)
```

### 2. 设置模式
```python
from drivers.robstride import RobstrideDriver, RunMode

# 位置模式 (PP - 梯形规划)
driver.set_mode(motor_id=1, mode=RunMode.POSITION_PP)

# 位置模式 (CSP - 连续位置)
driver.set_mode(motor_id=1, mode=RunMode.POSITION_CSP)

# 速度模式
driver.set_mode(motor_id=1, mode=RunMode.VELOCITY)

# 电流模式
driver.set_mode(motor_id=1, mode=RunMode.CURRENT)

# 运控模式 (PD + 前馈)
driver.set_mode(motor_id=1, mode=RunMode.MOTION_CONTROL)
```

### 3. 控制转速或角度

#### 位置模式 - 控制角度
```python
import math

# 移动到90度 (π/2 rad)
driver.move_to_position(motor_id=1, position=math.pi/2)

# 或使用底层方法
driver.set_position(motor_id=1, position=1.57, mode=RunMode.POSITION_PP)
```

#### 速度模式 - 控制转速
```python
# 以1 rad/s的速度旋转
driver.rotate_at_speed(motor_id=1, speed=1.0)

# 停止
driver.rotate_at_speed(motor_id=1, speed=0.0)
```

### 4. 读取电机数据
```python
# 读取完整状态
data = driver.get_observation(motor_id=1)
print(f"位置: {data['position']} rad")
print(f"速度: {data['velocity']} rad/s")
print(f"力矩: {data['torque']} Nm")
print(f"温度: {data['temperature']} °C")

# 或读取单个数据
position = driver.get_position(motor_id=1)
speed = driver.get_speed(motor_id=1)
torque = driver.get_torque(motor_id=1)
temp = driver.get_temperature(motor_id=1)
```

## 技术规格

| 电机型号 | 位置范围 | 速度范围 | 力矩范围 |
|---------|---------|---------|---------|
| **RS00** | ±2.79 rad (±160°) | ±33 rad/s | ±14 Nm |
| **EL05** | ±1.57 rad (±90°) | ±50 rad/s | ±6 Nm |
| **RS05** | ±1.57 rad (±90°) | ±50 rad/s | ±5.5 Nm |

- **通信方式**: CAN 2.0 扩展帧，29位 ID，1Mbps
- **默认波特率**: 1000000 (1M)
- **ID范围**: 1-253

## 辅助功能

```python
# 使能电机
driver.enable_motor(motor_id=1)

# 失能电机
driver.disable_motor(motor_id=1)

# 设置零位
driver.set_zero_position(motor_id=1)

# 测试电机是否在线
if driver.ping(motor_id=1):
    print("电机在线")

# 查询固件版本
version = driver.query_version(motor_id=1)
print(f"版本: {version}")
```

## 完整示例

查看 `example.py` 文件获取完整的使用示例。

运行示例：
```bash
cd drivers/robstride
python example.py
```

## 添加新电机型号

如果需要支持更多灵足电机型号（如 RS01, RS03, RS04, RS06 等），请查看：
[HOT_TO_ADD_NEW_MOTORS.md](HOW_TO_ADD_NEW_MOTORS.md)

## 与 Hiwonder/Feetech 的区别

| 特性 | Hiwonder LX-16A | Feetech ST3215 | Robstride |
|------|----------------|----------------|-----------|
| **通信方式** | UART串口 | UART串口 | CAN总线 |
| **平台支持** | Windows/Linux | Windows/Linux | Linux only |
| **位置单位** | 度 (0-240°) | 度 (0-360°) | 弧度 (rad) |
| **速度单位** | -1000~1000 | -1023~1023 | rad/s |
| **额外反馈** | 电压 | 电压 | 力矩 |

## 注意事项

1. **Linux专用** - 此驱动只能在 Linux 系统上运行（需要 SocketCAN）
2. **CAN配置** - 使用前必须配置 CAN 接口
3. **单位差异** - Robstride 使用弧度制，而 Hiwonder/Feetech 使用角度制
4. **力矩反馈** - Robstride 提供力矩反馈，这是其他两款没有的
5. **高级功能** - 官方SDK支持更多高级功能（重力补偿、轨迹规划等），可直接使用
