# MotorController 使用指南

## 概述

`MotorController` 是电机控制的核心类，支持三种品牌的舵机/电机：
- **Hiwonder LX-16A** - 幻尔科技总线舵机
- **Feetech ST3215** - 飞特科技总线舵机
- **Robstride** - 灵足时代 CAN 总线电机

## 初始化方式

### 方式一：无配置初始化（仅业务层）

```python
from controller.motor_controller import MotorController

# 不传入配置，只用于业务逻辑（如角度控制）
controller = MotorController(robot_interface=robot_interface)
```

**适用场景**：
- 通过 `robot_interface` 间接控制电机
- 不需要直接访问底层驱动

### 方式二：带配置初始化（直连驱动）

```python
from controller.motor_controller import MotorController

# LX-16A 舵机
config = {
    'servo_type': 'lx16a',
    'port': 'COM3',           # Windows
    'baudrate': 115200
}
controller = MotorController(config=config)

# ST3215 舵机
config = {
    'servo_type': 'st3215',
    'port': '/dev/ttyUSB0',   # Linux
    'baudrate': 1000000
}
controller = MotorController(config=config)

# Robstride 电机
config = {
    'servo_type': 'robstride',
    'port': 'can0',
    'baudrate': 1000000
}
controller = MotorController(config=config)
```

**适用场景**：
- 需要直接调用底层驱动 API
- 独立测试舵机功能
- 设置 ID、模式等硬件操作

## 主要功能

### 1. 控制单个电机角度

```python
success = controller.control_motor(
    arm='left',
    motor_name='shoulder_pan',
    angle=45.0
)
```

**参数说明**：
- `arm`: `'left'` 或 `'right'`
- `motor_name`: 电机名称（见下方列表）
- `angle`: 目标角度（度）

**电机名称列表**：
| 名称 | 说明 |
|------|------|
| `shoulder_pan` | 肩部旋转 |
| `shoulder_lift` | 肩部升降 |
| `elbow_flex` | 肘部弯曲 |
| `wrist_flex` | 腕部弯曲 |
| `wrist_roll` | 腕部旋转 |
| `gripper` | 夹爪 |

### 2. 读取传感器数据

```python
data = controller.read_sensor_data(
    arm='left',
    motor_name='shoulder_pan'
)

if data:
    print(f"位置: {data['position']}°")
    print(f"转速: {data['velocity']} rpm")
    print(f"电流: {data['current']} A")
    print(f"温度: {data['temperature']} °C")
```

**返回值**：
```python
{
    'position': float,      # 角度（度）
    'velocity': float,      # 转速（rpm）
    'current': float,       # 电流（A）
    'temperature': float    # 温度（°C）
}
```

**读取优先级**：
1. 优先从底层驱动读取（如果已初始化）
2. 降级到 `robot_interface` 读取

### 3. 设置电机 ID（纯硬件操作）

```python
success = controller.set_motor_id(
    port='COM3',
    servo_type='lx16a',
    old_id=1,
    new_id=10,
    baudrate=115200
)
```

**特点**：
- ✅ 与业务无关，不需要 `robot_interface`
- ✅ 临时打开串口，操作完自动关闭
- ✅ 支持所有品牌舵机

### 4. 校准电机零点

```python
success = controller.calibrate_motor(
    arm='left',
    motor_name='shoulder_pan',
    target_zero=0.0
)
```

**注意**：当前为临时方案，仅记录日志和更新角度值。真正的零点校准需要在电机固件层面实现。

### 5. 发送机械臂指令

```python
angles = {
    'shoulder_pan': 30.0,
    'shoulder_lift': -45.0,
    'elbow_flex': 60.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0
}

success = controller.send_arm_command(
    arm='left',
    angles=angles
)
```

## 完整示例

### 示例 1：LX-16A 舵机控制

```python
from controller.motor_controller import MotorController

# 初始化控制器
config = {
    'servo_type': 'lx16a',
    'port': 'COM3',
    'baudrate': 115200
}
controller = MotorController(config=config)

# 检查驱动是否连接
if controller.driver:
    print("✅ 驱动已连接")
    
    # 设置舵机 ID
    controller.set_motor_id('COM3', 'lx16a', 1, 10)
    
    # 读取传感器数据
    data = controller.read_sensor_data('left', 'shoulder_pan')
    if data:
        print(f"当前位置: {data['position']}°")
else:
    print("❌ 驱动未连接")
```

### 示例 2：ST3215 舵机批量配置

```python
from controller.motor_controller import MotorController

# 配置多个舵机的 ID
servos = [
    {'old_id': 1, 'new_id': 1},
    {'old_id': 2, 'new_id': 2},
    {'old_id': 3, 'new_id': 3},
]

for servo in servos:
    success = controller.set_motor_id(
        port='/dev/ttyUSB0',
        servo_type='st3215',
        old_id=servo['old_id'],
        new_id=servo['new_id'],
        baudrate=1000000
    )
    if success:
        print(f"✅ ID {servo['old_id']} → {servo['new_id']} 设置成功")
    else:
        print(f"❌ ID {servo['old_id']} → {servo['new_id']} 设置失败")
```

### 示例 3：Robstride 电机控制

```python
from controller.motor_controller import MotorController

# 初始化 Robstride 电机驱动
config = {
    'servo_type': 'robstride',
    'port': 'can0',
    'baudrate': 1000000
}
controller = MotorController(config=config)

# 读取电机状态
data = controller.read_sensor_data('left', 'shoulder_pan')
if data:
    print(f"位置: {data['position']}°")
    print(f"电流: {data['current']} A")
    print(f"温度: {data['temperature']} °C")
```

## 错误处理

所有方法都返回 `bool` 或 `Optional[Dict]`，建议进行错误检查：

```python
# 检查返回值
success = controller.control_motor('left', 'shoulder_pan', 45.0)
if not success:
    print("❌ 控制失败，请检查连接状态")

# 检查传感器数据
data = controller.read_sensor_data('left', 'shoulder_pan')
if data is None:
    print("❌ 无法读取传感器数据")
else:
    print(f"位置: {data['position']}°")
```

## 日志级别

控制器使用 Python 标准 logging 模块，可以通过以下方式调整日志级别：

```python
import logging

# 设置为 DEBUG 级别，查看详细日志
logging.getLogger('controller.motor_controller').setLevel(logging.DEBUG)

# 设置为 WARNING 级别，只显示警告和错误
logging.getLogger('controller.motor_controller').setLevel(logging.WARNING)
```

## 注意事项

1. **串口占用**：设置 ID 时会临时打开串口，操作完成后自动关闭
2. **ID 唯一性**：同一总线上的每个舵机必须有唯一的 ID
3. **波特率匹配**：确保配置的波特率与舵机实际波特率一致
4. **连接状态**：使用前确保机器人已连接（如果使用 `robot_interface`）
5. **异常处理**：所有方法都有完善的异常处理，不会抛出未捕获的异常

## 扩展新品牌

要添加新的舵机/电机品牌：

1. 在 `drivers/` 下创建新目录（如 `drivers/newbrand/`）
2. 实现驱动类，提供以下接口：
   - `connect() -> bool`
   - `disconnect()`
   - `send_action(action: Dict[str, float], time_ms: int)`
   - `get_observation() -> Dict[str, float]`
   - `set_id(old_id: int, new_id: int) -> bool`

3. 在 `_initialize_driver()` 中添加新分支：
```python
elif servo_type == 'newbrand':
    from drivers.newbrand.new_driver import NewDriver
    self.driver = NewDriver(port=port, baudrate=baudrate)
```

4. 在 `ServoType` 枚举中添加新类型（可选）
