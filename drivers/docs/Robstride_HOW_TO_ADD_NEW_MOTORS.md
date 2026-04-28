# 如何添加新的灵足电机型号

## 📋 背景

当前 `el_a3_sdk` 只定义了 3 个电机型号（RS00, EL05, RS05），但灵足实际有 9+ 个型号。

如果需要驱动其他型号（如 RS01, RS03, RS04, RS06 等），需要扩展 SDK。

---

## 🔧 添加步骤

### 步骤1：修改协议定义

编辑文件：`C:\www\open_source\EDULITE_A3\el_a3_sdk\el_a3_sdk\protocol.py`

#### 1.1 在 `MotorType` 中添加新型号

```python
class MotorType(IntEnum):
    """电机型号"""
    RS00 = 0   # 关节 1-3: ±14 Nm, ±33 rad/s
    EL05 = 1   # 关节 4-7(配置A): ±6 Nm,  ±50 rad/s
    RS05 = 2   # 关节 4-7(配置B): ±5.5 Nm, ±50 rad/s
    
    # === 新增型号 ===
    RS06 = 3   # 中型关节: ±36 Nm, ±50 rad/s
    RS04 = 4   # 大型关节: ±120 Nm, ±33 rad/s
    RS01 = 5   # 轻载关节: ±17 Nm, ±33 rad/s
    RS02 = 6   # 轻载关节: ±17 Nm, ±33 rad/s
    RS03 = 7   # 重载关节: ±90 Nm, ±50 rad/s
```

#### 1.2 在 `MOTOR_PARAMS` 中添加参数范围

```python
MOTOR_PARAMS = {
    MotorType.RS00: MotorParams(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    MotorType.EL05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-6.0, t_max=6.0),
    MotorType.RS05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-5.5, t_max=5.5),
    
    # === 新增型号参数 ===
    # RS06: ±36 Nm, ±50 rad/s
    MotorType.RS06: MotorParams(v_min=-50.0, v_max=50.0, t_min=-36.0, t_max=36.0),
    
    # RS04: ±120 Nm, ±33 rad/s
    MotorType.RS04: MotorParams(v_min=-33.0, v_max=33.0, t_min=-120.0, t_max=120.0),
    
    # RS01/RS02: ±17 Nm, ±33 rad/s
    MotorType.RS01: MotorParams(v_min=-33.0, v_max=33.0, t_min=-17.0, t_max=17.0),
    MotorType.RS02: MotorParams(v_min=-33.0, v_max=33.0, t_min=-17.0, t_max=17.0),
    
    # RS03: ±90 Nm, ±50 rad/s
    MotorType.RS03: MotorParams(v_min=-50.0, v_max=50.0, t_min=-90.0, t_max=90.0),
}
```

#### 1.3 （可选）更新默认映射

如果您的机械臂使用新型号，更新 `DEFAULT_MOTOR_TYPE_MAP`：

```python
DEFAULT_MOTOR_TYPE_MAP = {
    1: MotorType.RS00,
    2: MotorType.RS00,
    3: MotorType.RS00,
    4: MotorType.EL05,
    5: MotorType.EL05,
    6: MotorType.EL05,
    7: MotorType.EL05,
    # 如果有更多关节，继续添加...
}
```

---

### 步骤2：重新安装 SDK

```bash
cd C:\www\open_source\EDULITE_A3\el_a3_sdk
pip install -e .
```

---

### 步骤3：在代码中使用新型号

```python
from el_a3_sdk.protocol import MotorType, MOTOR_PARAMS

# 创建自定义电机类型映射
custom_motor_map = {
    1: MotorType.RS06,  # 关节1使用RS06
    2: MotorType.RS06,
    3: MotorType.RS04,  # 关节3使用RS04（大力矩）
}

# 初始化驱动时传入
from el_a3_sdk.can_driver import RobstrideCanDriver

driver = RobstrideCanDriver(
    can_name="can0",
    motor_type_map=custom_motor_map  # 使用自定义映射
)
```

---

## 📊 完整电机参数参考表

| 型号 | 峰值扭矩 | 速度范围 | 典型应用 | p_min/p_max | v_min/v_max | t_min/t_max |
|------|---------|---------|---------|-------------|-------------|-------------|
| **RS00** | 14 Nm | ±33 rad/s | 基座关节 | ±12.57 | ±33.0 | ±14.0 |
| **RS01** | 17 Nm | ±33 rad/s | 轻载关节 | ±12.57 | ±33.0 | ±17.0 |
| **RS02** | 17 Nm | ±33 rad/s | 轻载关节 | ±12.57 | ±33.0 | ±17.0 |
| **RS03** | 90 Nm | ±50 rad/s | 重载关节 | ±12.57 | ±50.0 | ±90.0 |
| **RS04** | 120 Nm | ±33 rad/s | 工业级 | ±12.57 | ±33.0 | ±120.0 |
| **RS05** | 5.5 Nm | ±50 rad/s | 微型机器人 | ±12.57 | ±50.0 | ±5.5 |
| **RS06** | 36 Nm | ±50 rad/s | 中小型机器人 | ±12.57 | ±50.0 | ±36.0 |
| **EL05** | 6 Nm | ±50 rad/s | 桌面机械臂 | ±12.57 | ±50.0 | ±6.0 |

**注意**：
- `p_min/p_max`: 位置范围（弧度），通常都是 ±12.57 rad (±720°)
- `v_min/v_max`: 速度范围（rad/s），根据型号不同
- `t_min/t_max`: 力矩范围（Nm），根据型号不同

---

## 🎯 在您的封装驱动中使用

如果您使用了我们创建的薄封装层 `drivers/robstride/robstride_driver.py`，可以这样指定电机类型：

```python
from drivers.robstride import RobstrideDriver
from el_a3_sdk.protocol import MotorType

# 创建驱动，指定自定义电机映射
driver = RobstrideDriver(can_name="can0")

# 连接后设置电机类型映射
driver.driver.motor_type_map = {
    1: MotorType.RS06,
    2: MotorType.RS06,
    3: MotorType.RS04,
}

driver.connect()
```

---

## ⚠️ 注意事项

1. **参数准确性**：上述参数是基于公开资料的估算值，实际使用时请以官方技术文档为准
2. **通信协议一致**：所有 Robstride 电机使用相同的 CAN 通信协议，只是参数范围不同
3. **ID 分配**：确保每个电机有唯一的 CAN ID（1-253）
4. **电源匹配**：不同型号的电压/电流需求可能不同，请确认电源规格
5. **散热要求**：大扭矩电机（如 RS04）可能需要额外散热

---

## 📚 获取准确参数

如需准确的电机参数，建议：

1. **查看官方文档**：https://gitee.com/robstride/EDULITE_A3
2. **联系技术支持**：zhaoyuan@robstride.com
3. **读取电机参数**：使用 `read_parameter()` 方法直接从电机读取实际参数

```python
# 读取电机的实际参数
result = driver.read_parameter(motor_id=1, param_index=0x700B)  # LIMIT_TORQUE
if result:
    print(f"力矩限制: {result.value} Nm")
```

---

## 💡 快速示例：添加 RS06

假设您要在一台 3 自由度机械臂上使用 RS06：

```python
# 1. 修改 protocol.py（如上所述）

# 2. 重新安装 SDK
cd C:\www\open_source\EDULITE_A3\el_a3_sdk
pip install -e .

# 3. 在代码中使用
from el_a3_sdk import ELA3Interface
from el_a3_sdk.protocol import MotorType

arm = ELA3Interface(
    can_name="can0",
    motor_type_map={
        1: MotorType.RS06,
        2: MotorType.RS06,
        3: MotorType.RS06,
    }
)

arm.ConnectPort()
arm.EnableArm()

# 控制机械臂
arm.JointCtrl(0.5, 0.3, -0.2)

arm.DisableArm()
arm.DisconnectPort()
```

---

## ✅ 总结

添加新电机型号的核心步骤：
1. ✅ 在 `MotorType` 中定义新型号
2. ✅ 在 `MOTOR_PARAMS` 中添加参数范围
3. ✅ 重新安装 SDK
4. ✅ 在代码中使用新型号

整个过程只需修改一个文件（`protocol.py`），非常简单！
