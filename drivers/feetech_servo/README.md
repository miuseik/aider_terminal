# Feetech STS3215 舵机驱动

本目录包含从 **Open Duck Mini Runtime** 项目直接复制的 Feetech STS3215 舵机驱动。

## 📦 驱动类型

### 1. RustypotDriver (高性能)
- **来源**: `mini_bdx_runtime/rustypot_position_hwi.py`
- **特点**: 
  - 基于 Rust 实现的高性能 Python 绑定
  - 支持 ~1kHz 控制频率
  - 批量同步写入
  - 适用于实时控制场景
- **依赖**: `pip install rustypot`

### 2. PypotDriver (功能全面)
- **来源**: `scripts/configure_motor.py`
- **特点**:
  - Pollen Robotics 开发的纯 Python 库
  - 功能全面，易于使用
  - 支持配置和调试
  - 适用于非实时场景
- **依赖**: `pip install pypot`

## 🔧 安装依赖

```bash
# 安装 rustypot (推荐用于实时控制)
pip install rustypot

# 或安装 pypot (推荐用于配置和调试)
pip install pypot

# 或两者都安装（自动降级）
pip install rustypot pypot
```

## 💡 使用示例

### 通过 MotorController 使用（推荐）

```python
from controller.motor_controller_new import MotorController

# 创建控制器
controller = MotorController()

# 注册舵机（品牌会自动检测）
controller.register_servo('/dev/ttyACM0', 1, 'feetech', 'joint_1')

# 控制舵机（自动选择 rustypot 或 pypot）
controller.set_servo_angle('/dev/ttyACM0', 1, 90.0)

# 批量控制
controller.set_servos_angles('/dev/ttyACM0', {1: 90, 2: 45})

# 同步控制
controller.sync_write_positions('/dev/ttyACM0', {1: 90, 2: 45})
```

### 直接使用 RustypotDriver

```python
from drivers.feetech_servo import RustypotDriver

# 创建驱动
driver = RustypotDriver(port='/dev/ttyACM0', baudrate=1000000)
driver.connect()

# 控制单个舵机
driver.set_position(1, 90.0)  # 角度（度）

# 批量控制
driver.set_positions({1: 90, 2: 45})

# 读取状态
status = driver.read_status(1)
print(f"位置: {status['position']}°, 速度: {status['velocity']}")

# 断开连接
driver.disconnect()
```

### 直接使用 PypotDriver

```python
from drivers.feetech_servo import PypotDriver

# 创建驱动
driver = PypotDriver(port='/dev/ttyACM0')
driver.connect()

# 控制单个舵机
driver.set_position(1, 90.0)  # 角度（度）

# 批量控制
driver.set_positions({1: 90, 2: 45})

# 配置舵机
driver.configure_servo(1, kp=32, ki=0, kd=0)

# 修改ID
driver.set_id(1, 2)

# 断开连接
driver.disconnect()
```

### 使用 PypotConfigurator 配置舵机

```python
from drivers.feetech_servo import PypotConfigurator

# 创建配置器
config = PypotConfigurator(port='/dev/ttyACM0')

# 配置舵机（扫描、设置PID、修改ID）
config.configure_motor(new_id=5, kp=32, ki=0, kd=0)
```

## 🔄 自动降级策略

`motor_controller_new.py` 会按以下顺序尝试连接：

1. **优先尝试 Rustypot** (高性能)
   - 如果成功 → 使用 RustypotDriver
   - 如果失败 → 继续尝试 Pypot

2. **降级到 Pypot** (功能全面)
   - 如果成功 → 使用 PypotDriver
   - 如果失败 → 返回错误

这样可以确保即使某个库未安装，系统仍能正常工作。

## 📊 对比

| 特性 | Rustypot | Pypot |
|------|----------|-------|
| **性能** | ⚡ 高 (~1kHz) | 🐢 中等 |
| **语言** | Rust + Python | 纯 Python |
| **实时性** | ✅ 优秀 | ⚠️ 一般 |
| **功能完整性** | ⚠️ 基础 | ✅ 全面 |
| **配置能力** | ❌ 有限 | ✅ 强大 |
| **适用场景** | 实时控制 | 配置/调试 |

## 🎯 推荐用法

- **实时控制** → 使用 `RustypotDriver`
- **配置舵机** → 使用 `PypotConfigurator`
- **一般控制** → 两者皆可，自动选择
- **开发调试** → 使用 `PypotDriver`

## 📝 注意事项

1. **单位**: Rustypot 内部使用弧度，但驱动适配器已转换为角度（度）
2. **波特率**: 默认 1000000，可根据需要调整
3. **权限**: 确保有串口访问权限 (`sudo usermod -a -G dialout $USER`)
4. **依赖**: 至少安装其中一个库（rustypot 或 pypot）

## 🔗 原始代码来源

- Open Duck Mini Runtime: `/home/miuseik/www/open_origin/duck/Open_Duck_Mini_Runtime`
  - `mini_bdx_runtime/rustypot_position_hwi.py`
  - `scripts/configure_motor.py`
