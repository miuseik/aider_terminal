# Aider Terminal 系统架构

> 本文档描述系统**现状**与**目标结构（ROS 化）**。
> 目标：子系统以 ROS 的 Node / Topic / Service 模型组织，通信经 `aider_server` 中转。

---

## 1. ROS 概念 ↔ 本项目映射

| ROS 概念 | 本项目等价物 | 现状 |
|---------|-------------|------|
| **Master** (节点/话题注册中心) | `aider_server` (WS 中转) | ⚠️ 仅按 `type` 透传/广播，无显式节点注册表、无话题目录 |
| **Node** (独立生命周期的进程/模块) | 各子系统：`sensors` / `navigation` / `perception` / `interaction` / `inputs` / `control_loop` | ⚠️ 目前 `app.py` 直接 `new` 装配，无统一节点接口 |
| **Topic** (发布/订阅异步数据流) | WS 消息的 `type` 字段（如 `sensor/rgbd`, `sensor/imu`, `slam_pose`） | ✅ 已采用；消息结构对齐 `sensor_msgs` |
| **Message** (消息结构体) | 普通 JSON dict（已对齐 `sensor_msgs/Image`, `sensor_msgs/Imu`） | ✅ 已对齐（无 ROS 运行时） |
| **Service** (请求/响应) | HTTP REST（`/api/servo/limits` 等）+ WS `action=` | ✅ 已有 |
| **Action** (长任务) | 暂用 Service 模拟 | ⚠️ 未区分 |
| **tf** (坐标系变换) | 暂未建；`frame_id` 字符串已在消息里预留 | ⚠️ 仅占位 |
| **launch** (启动编排) | `app.py` `TelegripSystem` + `start-dev.sh` / `start-pro.sh` | ⚠️ 无声明式 launch |

---

## 2. 现状结构（诚实版）

```
aider_terminal/
├── aiderminal/                  # 主包（控制核心）
│   ├── app.py                   # TelegripSystem: 装配入口（直连 new 各组件）
│   ├── comm/websocket/          # WS 客户端（连 aider_server）
│   ├── config/                  # 配置 (settings.py)
│   ├── controller/              # 执行器控制
│   ├── core/                    # control_loop（每帧消费 ControlGoal→IK→adapter）
│   ├── drivers/                 # 设备驱动（opencv_camera_driver 在此）
│   ├── inputs/                  # VR / 键盘 / 外骨骼 → ControlGoal
│   ├── robots/aider/            # 机器人适配 (adapter / IK / 限位)
│   ├── router/  nodes/  reporter/  utils/
├── aider_camera/                # 独立包: AstraCameraDriver (RGB-D, pyorbbecsdk2) ⚠️ 游离在主包外
├── aider_sensors/               # 独立包: SensorManager + HiwonderIMU ⚠️ 游离在主包外
├── test/                        # 测试脚本 (_test_*.py)
├── setup.py  README.md
```

**乱的根源**：
1. 相机/IMU 驱动分裂在 `aiderminal/drivers/`（OpenCV）与独立包 `aider_camera/`、`aider_sensors/`，归属不统一。
2. `sensors` 既是"驱动层"（`imu_hiwonder`）又是"采集+广播层"（`sensor_manager`），与 `drivers/` 边界模糊。
3. `app.py` 直接 `from aider_sensors import ...` 硬接，新增子系统会堆成意大利面。
4. `aider_server` 只做 `type` 透传，缺少 ROS Master 的"节点/话题"元数据，无法做精确订阅与发现。

---

## 3. 目标结构（ROS 化）

### 3.1 节点化（每个子系统 = 一个 Node）
所有子系统实现统一接口：
```python
class Node:
    name: str
    def start(self): ...      # 发布/订阅注册、资源初始化
    def stop(self): ...       # 释放资源
    # 通过 bus.publish(topic, msg) / bus.subscribe(topic, cb)
```

### 3.2 总线即 Master
`aider_server` 升级为带元数据的消息总线：
- **节点注册**：Node `start()` 时向 server 报到 `{node: "sensors", pubs: ["sensor/rgbd","sensor/imu"], subs: [...]}`
- **话题路由**：client 订阅某 `topic` 才收该 `topic`（替代当前全量广播）
- **话题目录**：`GET /api/topics` 可查当前活跃话题与发布者

### 3.3 目标目录（收敛版）
```
aider_terminal/
├── aiderminal/
│   ├── app.py                  # TelegripSystem: 仅做 launch 编排，按清单启停 Node
│   ├── bus/                    # Node 基类 + 本地/WS 总线抽象（pub/sub）
│   ├── nodes/                  # 所有子系统节点
│   │   ├── sensors/            # 采集节点（含驱动 aider_camera/aider_sensors 迁入）
│   │   │   ├── camera_driver.py   # AstraCameraDriver (从 aider_camera 迁来)
│   │   │   ├── imu_driver.py      # HiwonderIMU (从 aider_sensors 迁来)
│   │   │   └── sensor_node.py     # SensorManager → 发布 sensor/rgbd, sensor/imu
│   │   ├── inputs/             # VR/键盘/外骨骼 → 发布 control/goal
│   │   ├── navigation/         # 订阅 sensor/* + odom → 发布 control/goal(底盘)
│   │   ├── perception/         # 订阅 sensor/* → 发布 slam_pose, slam_map
│   │   └── interaction/        # 订阅 sensor/rgbd + 状态 → 调 VLA → 发布对话/动作
│   ├── core/control_loop.py    # 订阅 control/goal → IK → adapter
│   ├── robots/aider/           # 机器人适配 (adapter / IK)
│   └── comm/                   # WS 客户端（连 aider_server master）
```

### 3.4 话题清单（约定）
| Topic | 类型(对齐 sensor_msgs) | 发布者 | 订阅者 |
|-------|----------------------|--------|--------|
| `sensor/rgbd` | Image×2 (bgr8 / 16UC1) | sensors | perception, interaction, 前端 |
| `sensor/imu` | Imu (m/s², rad/s, quat) | sensors | perception, navigation |
| `sensor/scan` | LaserScan (未来) | sensors/lidar | navigation, perception |
| `slam_pose` | PoseStamped / Odometry | perception | navigation, 前端 |
| `slam_map` | OccupancyGrid | perception | 前端 |
| `control/goal` | ControlGoal | inputs / navigation / interaction | control_loop |
| `tf` | 坐标系变换树 | 各 Node | 全部 |

---

## 4. 迁移路线（4 阶段，与功能落地同步）

1. **传感地基（已完成 70%）**：`aider_sensors` 驱动 + 采集 + ROS 对齐广播。
   - 待做：驱动迁入 `aiderminal/nodes/sensors/`，统一 Node 接口。
2. **总线升级**：`aider_server` 加节点注册 + 按 topic 订阅；`bus/` 抽象层。
3. **感知**：`perception` 节点订阅 `sensor/*`，接 RTAB-Map → 发布 `slam_pose`/`slam_map`。
4. **导航 + 交互**：`navigation` 订阅 `sensor/*`+`slam_pose` 做规划 → `control/goal`；`interaction` 接 VLA 服务。

> 注：`aider_slam` (RTAB-Map) 与 `aider_vla` (Prismatic) 为**外部依赖**，不并入 terminal 仓库，
> 仅作为独立进程/服务经 WS/HTTP 消费 `sensor/*` 主题、发布 `slam_*` / 动作意图。

---

## 5. 当前进度标记
- ✅ 相机驱动 `AstraCameraDriver`（aider_camera）
- ✅ IMU 驱动 `HiwonderIMU`（aider_sensors）
- ✅ `SensorManager` 采集 + 广播 `sensor/rgbd` / `sensor/imu`（字段对齐 sensor_msgs）
- ✅ `aider_server` 已透传 `sensor/*`
- ⚠️ 未做：Node 接口统一、驱动归位、总线元数据、perception/navigation/interaction 节点
