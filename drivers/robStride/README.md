# RobStride 电机驱动

## 目录结构

```
robStride/
├── __init__.py                      # 模块入口
├── robstride_official_driver.py     # 封装层（提供简洁 API）
└── robstride_dynamics/              # 官方 SDK（已集成）
    ├── __init__.py
    ├── bus.py                       # CAN 总线通信
    ├── protocol.py                  # 通信协议定义
    └── table.py                     # 电机参数表
```

## 使用方式

```python
from drivers.robStride import RobStrideMotor

# 创建电机实例
motor = RobStrideMotor(motor_id=127, can_interface="can0")

# 连接并控制
motor.connect()
motor.enable_torque(True)
motor.set_position(1.57)  # 90度
state = motor.read_state()
motor.disconnect()
```

## 说明

- **robstride_dynamics/**: 官方 Python SDK，直接集成到项目中，无需 pip install
- **robstride_official_driver.py**: 封装层，提供更简洁的 API，与原有代码兼容
- 所有依赖都在项目内部，部署时只需复制整个项目即可
