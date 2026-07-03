# Aider Terminal

基于 ROS2 的双臂机器人遥操作系统，支持 WebRTC 实时视频推流、PyBullet 物理仿真、Pink 逆运动学求解。

**支持机器人**：Aider（主）、Aloha

---

## 便捷操作
```bash
git add ./;git commit -m "初步重构ros";git push
```

## 快速启动 (ROS2 + Docker)

项目通过 ROS2 节点运行在 Docker 容器中。

### 首次启动

```bash
# 0. 允许 Docker 访问 X11（宿主机执行一次即可）
xhost +local:docker

# 1. 启动 aiderminal 容器（含 X11 图形转发）
docker run -d --name aiderminal \
  -v /home/miuseik/www/aider/aider_terminal:/ws/src/aiderminal:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  --network host \
  --device /dev/video0:/dev/video0 \
  aider_ros:x11 \
  bash -c "tail -f /dev/null"

# 2. 构建 & 运行
docker exec aiderminal bash -c "
  source /opt/ros/jazzy/setup.bash && \
  cd /ws && colcon build --symlink-install --packages-select aiderminal && \
  source install/setup.bash && \
  ros2 run aiderminal terminal_node --ros-args -p robot_type:=aider -p no_robot:=true
"
```

### 后续运行

```bash
docker start aiderminal
docker exec aiderminal bash -c "
  source /opt/ros/jazzy/setup.bash && \
  source /ws/install/setup.bash && \
  ros2 run aiderminal terminal_node --ros-args -p robot_type:=aider -p no_robot:=true
"
```

### 使用 launch 文件

```bash
docker exec aiderminal bash -c "
  source /opt/ros/jazzy/setup.bash && \
  source /ws/install/setup.bash && \
  ros2 launch aiderminal terminal.launch.py robot_type:=aider no_robot:=true
"
```

---

## ROS2 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_type` | string | `aider` | 机器人类型：`aider` / `aloha` |
| `no_robot` | bool | `true` | 禁用硬件连接，仅仿真 |
| `no_sim` | bool | `false` | 禁用 PyBullet 物理仿真 |
| `no_viz` | bool | `false` | 禁用 PyBullet 可视化窗口（无头模式） |
| `no_vr` | bool | `false` | 禁用 VR 输入 |
| `no_keyboard` | bool | `false` | 禁用键盘输入 |
| `env_dev` | bool | `false` | 开发模式：`true` 时 server/api 默认连 localhost |
| `server_host` | string | `""` | WebSocket 服务器地址（留空使用默认） |
| `api_host` | string | `""` | API 服务器地址（留空使用默认） |
| `autoconnect` | bool | `false` | 启动时自动连接机器人 |

### 环境切换

生产环境（默认，连接 `ws.houqicg.com` / `www.houqicg.com`）：

```bash
ros2 run aiderminal terminal_node --ros-args -p robot_type:=aider -p no_robot:=true
```

开发环境（连接 localhost）：

```bash
ros2 run aiderminal terminal_node --ros-args -p robot_type:=aider -p env_dev:=true
```

自定义服务器：

```bash
ros2 run aiderminal terminal_node --ros-args \
  -p robot_type:=aider \
  -p server_host:=my-server.com \
  -p api_host:=my-api.com
```

### 旧版 main.py 参数对照

| 旧 `main.py` 参数 | ROS2 等效参数 |
|---|---|
| `--role-aider` | `-p robot_type:=aider`（默认值） |
| `--role-aloha` | `-p robot_type:=aloha` |
| `--env-dev` | `-p env_dev:=true` |
| `--server-host xxx` | `-p server_host:=xxx` |
| `--api-host xxx` | `-p api_host:=xxx` |
| `--no-robot` | `-p no_robot:=true` |
| `--no-viz` | `-p no_viz:=true` |
| `--no-sim` | `-p no_sim:=true` |
| `--autoconnect` | `-p autoconnect:=true` |

---

## 项目结构

```
aider_terminal/
├── aiderminal/                 # Python 包（所有核心逻辑）
│   ├── comm/                   # 通信：WebSocket 客户端、API 协议
│   ├── config/                 # 配置：config.yaml、settings.py
│   ├── controller/             # 执行器/电机控制器
│   ├── core/                   # 核心：控制循环、机器人接口、运动学
│   │   ├── kinematic/pybullet/ # PyBullet FK + IK
│   │   └── kinematic/pink/     # Pink IK 求解器（Pinocchio）
│   ├── drivers/                # 硬件驱动
│   │   ├── actuator/           #   执行器（Feetech / RobStride）
│   │   ├── audio/              #   麦克风输入
│   │   ├── camera/             #   OpenCV 摄像头
│   │   └── webrtc/             #   WebRTC 视频/音频推流
│   ├── inputs/                 # 输入：VR 手柄、键盘
│   ├── nodes/                  # ROS2 节点
│   ├── robots/                 # 机器人定义（aider / aloha）
│   ├── router/                 # 指令路由
│   └── utils/                  # 工具：网络、舵机检测、网格压缩
├── URDF/                       # 机器人 URDF 模型 + STL 网格
├── launch/                     # ROS2 launch 文件
├── app.py                      # 应用入口（非 ROS）
├── main.py                     # 主入口（非 ROS）
├── package.xml                 # ROS2 包描述
├── setup.py / setup.cfg        # Python 包安装
└── pyproject.toml
```

---

## 架构

```
┌──────────────┐     WebSocket     ┌──────────────┐
│  远程服务器    │◄────────────────►│  Terminal     │
│  (ws.houqicg) │                  │  Node (ROS2)  │
└──────────────┘                  └──────┬───────┘
        │                                │
        │ WebRTC (视频/音频)              │
        ▼                                ▼
┌──────────────┐              ┌─────────────────┐
│   VR 头显     │              │  控制循环        │
│  (WebXR)     │              │  ┌─────────────┐│
└──────────────┘              │  │ PyBullet    ││
                              │  │ 物理仿真    ││
┌──────────────┐              │  ├─────────────┤│
│  键盘输入     │──────────────►│  │ Pink IK     ││
└──────────────┘              │  │ 求解器      ││
                              │  ├─────────────┤│
                              │  │ 机器人硬件   ││
                              │  │ 接口        ││
                              │  └─────────────┘│
                              └─────────────────┘
```

**数据流**：VR/键盘输入 → 控制循环 → IK 求解 → 机器人硬件 + PyBullet 可视化 → WebRTC 回传视频

---

## 支持的机器人

### Aider
- 双臂 8-DOF 机器人
- URDF：`URDF/aider/urdf/aider_pro.SLDASM.urdf`
- IK：PyBullet + Pink (Pinocchio)

### Aloha
- 基于 SO100 机械臂的双臂系统
- URDF：`URDF/aloha/urdf/`

---

## Pink IK 求解器依赖

Pink IK 求解器基于 [pink](https://github.com/stephane-caron/pink) 和 Pinocchio，需要额外依赖。

### Linux (Docker 容器内)

```bash
pip install meshcat_shapes qpsolvers loop_rate_limiters quadprog pin-pink
```

### Windows / Conda

```bash
# pinocchio 需要 conda 安装（预编译 C++ 库）
conda install -c conda-forge pinocchio -y

# 其余依赖
pip install meshcat_shapes qpsolvers loop_rate_limiters quadprog

# pin-pink 跳过自带依赖，用 conda 的 pinocchio
pip install pin-pink --no-deps
```

### 所需包一览

| 包名 | 用途 |
|------|------|
| `pinocchio` | 机器人建模与运动学 |
| `pin-pink` | Pink IK 求解框架 |
| `meshcat_shapes` | 可视化辅助 |
| `qpsolvers` + `quadprog` | 二次规划求解 |
| `loop_rate_limiters` | 循环频率限制 |

---

## 常见问题

### PyBullet 仿真窗口不弹出

1. 确认容器启动时包含 X11 转发：`-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY`
2. 宿主机执行 `xhost +local:docker`
3. 容器内需安装 `x11-utils mesa-utils`

### 摄像头无法打开

摄像头不可用时系统自动降级，WebRTC 以无视频模式运行，不影响其他功能。

### 音频不可用

`sounddevice` 在容器内可能无法直接访问宿主音频设备，系统自动禁用音频推流。

---

## 许可证

MIT
