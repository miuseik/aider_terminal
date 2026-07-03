# Aider Terminal

双臂机器人遥操作系统，支持 PyBullet 物理仿真、Pink 逆运动学、WebRTC 视频推流。

**机器人**：Aider / Aloha

---
## 便捷操作
```bash
git add ./;git commit -m "视频ok";git push
```
## 快速开始

### 方式一：直接跑（无 Docker，无 ROS2）✅ 推荐日常开发

```bash
# 1. 安装
cd ~/www/aider/aider_terminal && pip install -e .

# 2. 启动
main_cli --role-aider
main_cli --role-aloha 
main_cli --role-aider --env-dev              # 开发环境（连 localhost）
```

> `--no-robot` 只跑仿真不连真机，日常够用。

### 方式二：Docker + ROS2（需要仿真窗口 / 真机）

```bash
# 首次：创建容器 + 编译 + 运行
./scripts/docker_run.sh              # Aider
```

```bash

./scripts/docker_run.sh aloha         # Aloha
```

```bash

# 后续：直接运行（跳过编译）
./scripts/docker_start.sh
```

```bash

./scripts/docker_start.sh aloha

```

---

## 参数速查

| 参数 | 默认 | 说明 |
|------|------|------|
| `--role-aider` | — | 使用 Aider 机器人 |
| `--role-aloha` | — | 使用 Aloha 机器人 |
| `--no-robot` | false | 不连真机，仅仿真 |
| `--no-sim` | false | 禁用 PyBullet 仿真 |
| `--no-viz` | false | 无头模式（不弹窗口） |
| `--env-dev` | false | 开发环境（连 localhost） |
| `--server-host` | ws.houqicg.com | WebSocket 服务器 |
| `--api-host` | www.houqicg.com | API 服务器 |
| `--autoconnect` | false | 启动自动连接电机 |
| `--log-level` | warning | debug / info / warning / error |

ROS2 等价的参数名：`robot_type` `no_robot` `no_sim` `no_viz` `env_dev` `server_host` `api_host` `autoconnect` `log_level`

---

## 架构

```
┌──────────────┐     WebSocket     ┌──────────────┐
│  远程服务器    │◄────────────────►│  Terminal     │
│              │                  │  Node         │
└──────┬───────┘                  └──────┬───────┘
       │ WebRTC                         │
       ▼                                ▼
┌──────────────┐              ┌─────────────────┐
│  VR 头显      │              │  控制循环        │
│  (WebXR)     │              │  IK → 仿真 → 硬件 │
└──────────────┘              └─────────────────┘
```

**数据流**：VR/键盘 → 控制循环 → IK 求解 → 机器人硬件 + PyBullet 可视化 → WebRTC 回传

---

## 环境切换

```bash
# 生产（默认，连 ws.houqicg.com）
main_cli --role-aider --no-robot

# 开发（连 localhost）
main_cli --role-aider --env-dev

# 自定义服务器
main_cli --role-aider --server-host 192.168.1.100 --api-host 192.168.1.100
```

---

## 项目结构

```
aider/
├── aider_camera/               # 摄像头驱动（独立 pip 包）
├── aider_terminal/
│   ├── aiderminal/app.py       # CLI 入口
│   ├── aiderminal/config/      # 配置
│   ├── aiderminal/core/        # 控制循环、IK 求解
│   ├── aiderminal/drivers/     # 音频、电机、WebRTC（摄像头已拆）
│   ├── aiderminal/inputs/      # VR / 键盘输入
│   ├── aiderminal/comm/        # WebSocket 通信
│   ├── aiderminal/robots/      # Aider / Aloha 机器人适配
│   ├── aiderminal/nodes/       # ROS2 节点
│   ├── URDF/                   # 机器人模型
│   ├── launch/                 # ROS2 launch
│   ├── setup.py                # 包安装 + 依赖声明
│   └── pyproject.toml          # 构建系统
```

---

## Pink IK 依赖

```bash
# Linux (Docker 内)
pip install meshcat_shapes qpsolvers loop_rate_limiters quadprog pin-pink

# Windows / Conda
conda install -c conda-forge pinocchio -y
pip install meshcat_shapes qpsolvers loop_rate_limiters quadprog
pip install pin-pink --no-deps
```

---

## 常见问题

### PyBullet 窗口不弹

容器内检查 X11 转发：`-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY`，宿主机 `xhost +local:docker`。

### 摄像头 / 音频不可用

系统自动降级，不影响控制循环运行。

### ros2: 未找到命令

ROS2 只在 Docker 容器内可用，宿主机用 `main_cli` 直接跑。

---

MIT
