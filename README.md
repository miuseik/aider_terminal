# Aider Terminal

双臂机器人遥操作系统，支持 PyBullet 物理仿真、Pink 逆运动学、WebRTC 视频推流。

**支持机器人**：Aider（8 轴）、Aloha（8 轴）

---

## 快速开始
```bash
git add ./;git commit -m "暂且算他过了";git push
``` 
### 方式一：无需 Docker

```bash
pip install -e .
```

```bash
main_cli --role-aider --no-robot
```

### 方式二：Docker

#### 安装 Docker

```bash
sudo apt install -y docker.io
```

```bash
sudo usermod -aG docker $USER
```

#### 首次启动（构建镜像 + 启动项目）

```bash
sudo ./scripts/docker_run.sh
```

```bash 
#启动 Aloha：
sudo ./scripts/docker_run.sh aloha
```

#### 日常启动（跳过构建）

```bash
sudo ./scripts/docker_start.sh
```

启动 Aloha：

```bash
sudo ./scripts/docker_start.sh aloha
```

> 脚本会自动删除旧容器并创建新容器，每次都是全新启动。

#### 查看日志

```bash
sudo docker logs -f aiderminal
```

---

## 常用操作

| 你要做什么 | 执行这个 |
|-----------|---------|
| **启动 Aider** | `sudo ./scripts/docker_start.sh` |
| **启动 Aloha** | `sudo ./scripts/docker_start.sh aloha` |
| **从 Aider 切换到 Aloha** | `sudo ./scripts/docker_start.sh aloha` |
| **从 Aloha 切换到 Aider** | `sudo ./scripts/docker_start.sh` |
| **重启项目** | `sudo ./scripts/docker_start.sh` |
| **停止项目** | `sudo docker stop aiderminal` |
| **看运行日志** | `sudo docker logs -f aiderminal` |

切换机器人 = 重新启动一次，脚本自动删除旧容器，不需要手动操作。

---

## 参数说明

### 环境变量（Docker）

| 变量 | 默认 | 说明 |
|------|------|------|
| `ROBOT_TYPE` | aider | 机器人型号：`aider` 或 `aloha` |
| `NO_ROBOT` | true | `true` = 只仿真不连真机，`false` = 连接真机 |

### CLI 参数（直接跑时用）

| 参数 | 默认 | 说明 |
|------|------|------|
| `--role-aider` | — | 使用 Aider 机器人 |
| `--role-aloha` | — | 使用 Aloha 机器人 |
| `--no-robot` | false | 不连真机，仅仿真 |
| `--no-sim` | false | 禁用 PyBullet 仿真 |
| `--no-viz` | false | 无头模式（不弹窗口） |
| `--env-dev` | false | 开发环境（连 localhost） |
| `--server-host` | ws.houqicg.com | WebSocket 服务器地址 |
| `--api-host` | www.houqicg.com | API 服务器地址 |
| `--autoconnect` | false | 启动时自动连接电机 |
| `--log-level` | warning | 日志级别：debug / info / warning / error |

---

## 常见问题

### docker: permission denied

所有 docker 命令前面加 `sudo`。如果不想每次打 `sudo`：

```bash
sudo apt install -y util-linux-extra
```

```bash
newgrp docker
```

### 构建镜像太慢

Dockerfile 已配置清华镜像源。首次构建约 5 分钟，后续跳过。

### 摄像头 / 仿真窗口

没插摄像头不影响运行。需要摄像头或仿真窗口时，在启动脚本里加参数，详见 `scripts/docker_run.sh`。

---

MIT
