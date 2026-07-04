# Aider Terminal

双臂机器人遥操作系统 —— PyBullet 仿真 · Pink IK · WebRTC 推流

支持 **Aider** / **Aloha** 机器人。

---

## 启动

### 新电脑（Ubuntu，从零开始）

```bash
./scripts/setup.sh
```

> 自动装 Docker → 下载 ROS 基础镜像（~500MB）→ 构建项目镜像 → 启动。
> 首次约 3-5 分钟，后续 `docker compose up -d` 秒开。

### 已有 Docker 的电脑

```bash
# 一条命令：构建（首次）+ 启动
docker compose up --build -d
```

### 日常使用

```bash
docker compose up -d       # 启动
```
```bash
docker compose restart     # 改代码后重启
```
```bash
docker compose down        # 停止
```
```bash
docker compose logs -f     # 查看实时日志
```
```bash
docker compose logs     # 查看日志
```

> **改代码不需要重新构建镜像** —— 源码通过 volume 实时挂载，
> 重启容器时自动 `colcon build --symlink-install`，几秒生效。

### 切换机器人型号

```bash
ROBOT_TYPE=aloha docker compose up -d
```
```bash
ROBOT_TYPE=aider docker compose up -d
```

---

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_TYPE` | `aider` | 机器人型号：`aider` / `aloha` |
| `NO_ROBOT` | `true` | `true`=纯仿真，`false`=连接真机 |
| `NO_SIM` | `false` | 禁用 PyBullet 仿真 |
| `NO_VIZ` | `false` | 无头模式（不弹窗口） |

示例：
```bash
NO_ROBOT=false docker compose up -d   # 连真机
NO_VIZ=true docker compose up -d     # 服务器无头运行
```

---

## 常见问题

### 仿真窗口弹不出来

```bash
# 宿主机上执行
xhost +local:docker
```

### Permission denied (docker)

```bash
sudo usermod -aG docker $USER
# 退出终端重新登录
```

### 启动失败

```bash
docker compose logs --tail=50
```

### 彻底重建

```bash
docker compose down
docker compose build --no-cache
docker compose up -d
```

---

## 项目结构

```
aider_terminal/
├── aiderminal/              # Python 源码
│   ├── core/                #   控制循环、IK、FK
│   ├── robots/              #   机器人适配器
│   ├── drivers/             #   WebRTC、摄像头
│   └── config/              #   全局配置
├── URDF/                    # 机器人 URDF 模型
├── launch/                  # ROS 2 launch 文件
├── scripts/                 # 辅助脚本
│   ├── setup.sh             #   新机器一条龙
│   ├── docker_entrypoint.sh #   容器入口
│   └── docker_start.sh      #   快捷启动
├── docker-compose.yml
├── Dockerfile
└── README.md
```
---
## 便捷操作（勿删）
```bash
git add ./;git commit -m "ik主动避让";git push
``` 
```bash
git pull
``` 
---

MIT
