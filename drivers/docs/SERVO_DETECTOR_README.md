# 舵机自动探测工具

## 概述

`servo_detector.py` 是一个自动化工具，可以扫描串口并识别连接的舵机品牌和型号。

## 工作原理

探测器通过以下方式识别舵机：

1. **尝试不同波特率** - 依次测试常见波特率（115200, 1000000, 9600, 57600）
2. **发送协议特定的ping命令** - 使用各品牌的通信协议查询舵机
3. **验证响应** - 读取位置数据确认舵机真实存在
4. **返回检测结果** - 包含品牌、型号、端口、波特率、ID等信息

```
┌─────────────┐
│ 插入舵机     │
└──────┬──────┘
       │
       ▼
┌─────────────────┐
│ 扫描指定端口     │
└──────┬──────────┘
       │
       ▼
┌──────────────────────┐
│ 尝试 LewanSoul 协议   │ ← 波特率: 115200, 9600, 57600...
│ Ping ID 1-10         │
└──────┬───────────────┘
       │ 未找到?
       ▼
┌──────────────────────┐
│ 尝试 Feetech 协议     │ ← 波特率: 1000000, 115200...
│ Ping ID 1-10         │
└──────┬───────────────┘
       │ 找到!
       ▼
┌──────────────────────┐
│ 返回舵机信息          │
│ - 品牌: LewanSoul    │
│ - 型号: LX-16A       │
│ - 端口: COM3         │
│ - 波特率: 115200     │
│ - ID: 1              │
└──────────────────────┘
```

## 使用方法

### 1. 探测单个端口

```bash
# Windows
python drivers/servo_detector.py COM3

# Linux
python drivers/servo_detector.py /dev/ttyUSB0
```

**输出示例：**
```
============================================================
舵机自动探测工具
============================================================

正在扫描端口: COM3

INFO: 🔍 开始在 COM3 上探测舵机...
INFO: ✅ 检测到 Hiwonder LX-16A
   端口: COM3
   波特率: 115200
   ID: 1
   当前位置: 120.5°

============================================================
检测结果:
============================================================
品牌: Hiwonder
型号: LX-16A
端口: COM3
波特率: 115200
ID: 1
当前位置: 120.5°

============================================================
建议的配置:
============================================================
robot:
  left_arm:
    enabled: true
    port: COM3
    servo_type: lx16a
    baudrate: 115200
    # 检测到: Hiwonder LX-16A, ID=1
```

### 2. 扫描所有端口

```bash
python drivers/servo_detector.py
```

这会扫描所有常见的串口号（COM1-COM19, /dev/ttyUSB0-3, /dev/ttyACM0-3）。

### 3. 在代码中使用

```python
from drivers.servo_detector import ServoDetector

# 创建探测器
detector = ServoDetector()

# 探测单个端口
result = detector.detect_on_port('COM3')

if result:
    print(f"发现 {result['brand']} {result['model']}")
    print(f"端口: {result['port']}")
    print(f"波特率: {result['baudrate']}")
    print(f"ID: {result['id']}")
    
    # 生成配置
    config = detector.generate_config([result])
    print(config)
else:
    print("未检测到舵机")

# 或扫描所有端口
all_servos = detector.scan_all_ports()
for servo in all_servos:
    print(f"{servo['brand']} {servo['model']} @ {servo['port']}")
```

## 支持的舵机

| 品牌 | 型号 | 协议 | 默认波特率 | 平台 |
|------|------|------|-----------|------|
| Hiwonder (幻尔) | LX-16A | UART串口 | 115200 | Windows/Linux |
| Feetech (飞特) | ST3215 | UART串口 | 1000000 | Windows/Linux |
| Robstride (灵足) | RS00/EL05 | CAN总线 | 1000000 | Linux only |

**注意**: Robstride 电机使用 CAN 总线通信，无法通过此工具探测。需要使用专门的 CAN 扫描工具或直接连接测试。

## 常见问题

### Q1: 为什么检测不到舵机？

**可能原因：**
1. **舵机未供电** - 确保舵机电源正常
2. **串口号错误** - 检查设备管理器确认正确的COM口
3. **波特率不匹配** - 探测器会尝试多种波特率，但如果舵机使用了非标准波特率可能失败
4. **ID不在范围内** - 探测器只扫描ID 1-10，如果舵机ID超出范围需要手动指定

**解决方法：**
```bash
# 1. 查看设备管理器确认端口号
# Windows: 设备管理器 → 端口(COM和LPT)
# Linux: ls /dev/tty*

# 2. 尝试其他波特率
python drivers/servo_detector.py COM3  # 会自动尝试多种波特率

# 3. 如果知道ID，可以直接连接测试
```

### Q2: 如何修改探测的ID范围？

编辑 `servo_detector.py`，修改这两行：

```python
# 在 _try_lx16a 和 _try_st3215 方法中
for servo_id in range(1, 11):  # 改为 range(1, 254) 扫描所有ID
```

⚠️ **注意：** 扫描更多ID会增加探测时间。

### Q3: 可以同时连接多个舵机吗？

可以！探测器会扫描所有ID并报告发现的舵机：

```bash
python drivers/servo_detector.py COM3
```

如果有多个舵机在同一总线上，会显示：
```
✅ 检测到 LewanSoul LX-16A
   ID: 1
   当前位置: 120.5°

✅ 检测到 LewanSoul LX-16A
   ID: 2
   当前位置: 90.0°
```

## 集成到主程序

可以在 `robot_interface.py` 中添加自动探测功能：

```python
from drivers.servo_detector import ServoDetector

def auto_detect_and_connect(self):
    """自动探测并连接舵机"""
    detector = ServoDetector()
    
    # 探测左臂端口
    left_result = detector.detect_on_port(self.config.follower_ports["left"])
    
    if left_result:
        print(f"自动检测到左臂: {left_result['brand']} {left_result['model']}")
        # 更新配置
        self.config.follower_ports["left_servo_type"] = left_result['model'].lower()
        self.config.follower_ports["left_baudrate"] = left_result['baudrate']
    
    # 类似处理右臂...
```

## 技术细节

### 探测流程

1. **Hiwonder LX-16A 探测**
   - 使用帧头 `0x55 0x55`
   - 发送读位置命令 (CMD_READ_POS = 28)
   - 验证响应长度和校验和

2. **Feetech ST3215 探测**
   - 使用 SDK 的 ping 方法
   - 读取模型号寄存器
   - 验证通信成功

### 性能优化

- 每个波特率尝试超时设为 0.5 秒
- 找到第一个舵机后立即返回（可修改为扫描所有）
- 使用短超时避免长时间等待

## 扩展支持新舵机

要添加新品牌的舵机支持：

1. 在 `ServoDetector` 类中添加新方法：
```python
def _try_new_brand(self, port: str, baudrates: list) -> Optional[Dict]:
    """尝试新品牌协议"""
    # 实现探测逻辑
    pass
```

2. 在 `detect_on_port` 中调用：
```python
new_result = self._try_new_brand(port, baudrates)
if new_result:
    return new_result
```

## 总结

舵机自动探测工具可以：
- ✅ 自动识别舵机品牌和型号
- ✅ 自动检测波特率
- ✅ 自动发现舵机ID
- ✅ 生成配置文件片段
- ✅ 支持多舵机同时探测

**推荐使用流程：**
1. 插入舵机
2. 运行 `python drivers/servo_detector.py COM3`
3. 复制生成的配置到 `config.yaml`
4. 启动主程序

这样就不需要手动查找舵机的参数了！🎉
