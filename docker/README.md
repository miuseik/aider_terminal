# 部署 (docker/)

容器化部署编排。**Dockerfile 保留在仓库根目录**（ROS 构建需要根目录作为 build context）。

## 文件

| 文件 | 用途 |
|------|------|
| `docker-compose.dev.yml` | 开发环境覆盖层（挂载源码、开启 GUI/仿真） |
| `docker-compose.prod.yml` | 生产环境覆盖层（关闭仿真窗口、重启策略） |
| `entrypoint.sh` | 容器入口脚本（ROS 环境加载 + 启动节点） |
| `env.example` | 环境变量示例，复制为 `.env` 后按需修改 |

## 用法

基础编排文件在仓库根目录 `docker-compose.yml`，环境差异用覆盖层叠加：

```bash
# 开发（含仿真 GUI）
docker compose -f docker-compose.yml -f docker/docker-compose.dev.yml up -d

# 生产
docker compose -f docker-compose.yml -f docker/docker-compose.prod.yml up -d
```

## 挂载点

| 主机路径 | 容器路径 | 说明 |
|----------|----------|------|
| `./` | `/ws/src/aiderminal` | 源码（读写，colcon 需要） |
| `./data/logs` | `/ws/log` | 日志持久化 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 转发（仿真窗口） |
| `/dev` | `/dev` | 设备直通（CAN、串口、摄像头） |

## 环境变量

主要变量（详见 `env.example`）：

- `ROBOT_TYPE` — 机器人类型：`aider` / `aloha`（决定加载哪套 URDF 与适配器）
- `NO_SIM` — `true` 时禁用 PyBullet 仿真可视化（真机部署）
- `NO_VIZ` — `true` 时禁用可视化窗口
- `ENV` — `dev` / `pro`
- `DISPLAY` — X11 显示号
