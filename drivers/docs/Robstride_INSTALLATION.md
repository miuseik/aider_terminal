# Robstride 灵足电机驱动安装指南

## ⚠️ 重要提示

**Robstride 驱动只能在 Linux 系统上运行**，因为它依赖 SocketCAN。

---

## 📋 前置要求

1. **操作系统**: Linux (Ubuntu 20.04+ 推荐)
2. **CAN适配器**: CANdle / gs_usb 兼容设备
3. **Python**: 3.8+

---

## 🔧 安装步骤

### 1. 配置 CAN 接口

```bash
# 加载 CAN 模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan  # 虚拟CAN（用于测试）

# 配置物理 CAN 接口
sudo ip link set can0 up type can bitrate 1000000

# 验证配置
ip -details link show can0
```

### 2. 安装 el_a3_sdk

```bash
# 进入 SDK 目录
cd C:\www\open_source\EDULITE_A3\el_a3_sdk

# 以开发模式安装（推荐）
pip install -e .

# 或者安装完整功能（含动力学支持）
pip install -e ".[dynamics]"
```

### 3. 验证安装

```python
python -c "from el_a3_sdk import ELA3Interface; print('✅ el_a3_sdk 安装成功')"
```

### 4. 测试驱动

```bash
cd C:\www\lerobot\aider\aider_terminal\drivers\robstride
python example.py
```

---

## 📝 依赖文件说明

由于 `el_a3_sdk` 是本地包（不在 PyPI 上），已在以下文件中添加注释说明：

- ✅ `requirements.txt` - 已添加注释
- ✅ `pyproject.toml` - 已添加注释
- ✅ `requirements-dev.txt` - 已添加注释

**注意**: 这些依赖被注释掉了，因为：
1. 它不是 PyPI 包，无法通过 `pip install` 直接安装
2. 只在 Linux 上可用
3. 需要用户手动从本地路径安装

---

## 🐛 常见问题

### Q1: Windows 上能用吗？
**A**: ❌ 不能。SocketCAN 是 Linux 内核特性，Windows 不支持。

### Q2: macOS 上能用吗？
**A**: ❌ 不能。macOS 也不支持 SocketCAN。

### Q3: 如何在 Windows 上开发？
**A**: 
- 方案1: 使用 WSL2 (Windows Subsystem for Linux)
- 方案2: 使用虚拟机运行 Linux
- 方案3: 只编写代码，在 Linux 机器上测试

### Q4: 没有 CAN 硬件怎么办？
**A**: 可以使用虚拟 CAN 进行测试：
```bash
# 创建虚拟 CAN 接口
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# 在代码中使用
driver = RobstrideDriver(can_name="vcan0")
```

### Q5: 导入错误 "No module named 'el_a3_sdk'"
**A**: 确保已正确安装：
```bash
cd /path/to/EDULITE_A3/el_a3_sdk
pip install -e .
```

---

## 🔄 与其他驱动对比

| 驱动 | 平台 | 安装方式 | 难度 |
|------|------|---------|------|
| **Hiwonder** | Windows/Linux | `pip install` (自动) | ⭐ 简单 |
| **Feetech** | Windows/Linux | `pip install feetech-servo-sdk` | ⭐⭐ 中等 |
| **Robstride** | Linux only | 手动安装本地包 | ⭐⭐⭐ 复杂 |

---

## 📚 相关资源

- **官方仓库**: https://gitee.com/robstride/EDULITE_A3
- **SDK 源码**: `C:\www\open_source\EDULITE_A3\el_a3_sdk`
- **官方文档**: `C:\www\open_source\EDULITE_A3\el_a3_sdk\README_zh.md`
- **驱动封装**: `C:\www\lerobot\aider\aider_terminal\drivers\robstride\`

---

## 💡 快速开始（Linux）

```bash
# 1. 配置 CAN
sudo ip link set can0 up type can bitrate 1000000

# 2. 安装 SDK
cd /path/to/EDULITE_A3/el_a3_sdk
pip install -e .

# 3. 运行示例
cd /path/to/aider_terminal/drivers/robstride
python example.py
```

---

## ⚙️ 在项目中启用 Robstride

编辑配置文件，添加：

```yaml
robot:
  arm:
    enabled: true
    driver_type: robstride  # 使用 Robstride 驱动
    can_name: can0          # CAN 接口名称
    motor_ids: [1, 2, 3, 4, 5, 6, 7]
```

然后在代码中：

```python
from drivers.robstride import RobstrideDriver

driver = RobstrideDriver(can_name="can0")
if driver.connect():
    print("连接成功！")
    # ... 控制逻辑
    driver.disconnect()
```
