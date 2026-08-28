# 新家 (src/) — ROS 2 快速启动

> 与老业务 `aiderminal` 完全隔离：老业务用容器 `aiderminal`，新家用 `aider_ros2`。
> 启动方式与老系统完全一致，三个命令覆盖所有场景。

## 三个命令

| 场景 | 命令 |
|------|------|
| 新电脑，没有 Docker | `./scripts/setup_ros.sh` |
| 有 Docker，第一次 | `docker compose -f docker-compose.ros.yml up --build -d` |
| 之后每次启动 | `./start-ros.sh` |

对照老系统：

| 老系统 | 新家 |
|--------|------|
| `./scripts/setup.sh` | `./scripts/setup_ros.sh` |
| `docker compose up --build -d` | `docker compose -f docker-compose.ros.yml up --build -d` |
| `./start-pro.sh` | `./start-ros.sh` |

## 依赖说明（大小限制的取舍）

| 依赖 | 来源 | 大小 | 说明 |
|------|------|------|------|
| Docker | Gitee 离线包 / 系统自带 | ~100MB | 新机器自动装，老机器复用 |
| `ubuntu:noble` 基础镜像 | Gitee 离线包 | ~70MB | 卡在 Gitee 100MB 单文件限制内 |
| ROS 2 Jazzy | 清华 apt 源在线装 | 首次 ~2GB | 仅首次，之后用本机缓存 |
| Gazebo + PyBullet + RViz | 清华 apt 源在线装 | 首次 ~5GB | **仅开发机装**，树莓派跳过 |

**设计原则**：基础镜像离线（不卡 Docker Hub），系统包在线（走国内源）。
镜像构建后缓存在本机，后续启动无需重新构建。
不把完整镜像塞进 Gitee（免费版单文件 100MB 限制，装不下 6.9GB）。

## 部署环境自动区分

`setup_ros.sh` 按 CPU 架构自动选择（`ARCH=$(uname -m)`）：

| 环境 | 架构 | 结果 | 镜像 |
|------|------|------|------|
| 开发机 | `x86_64` | `sim` | ~6.9GB（含 Gazebo + PyBullet + RViz） |
| 树莓派 5 / Jetson | `aarch64` | `runtime` | ~1.5GB（仅 ROS2 + ros2_control） |

强制指定：`DEPLOY_TARGET=runtime ./scripts/setup_ros.sh`

## 常用操作

```bash
# 进入容器
docker exec -it aider_ros2 bash

# 容器内常用（先 source）
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash

# 重新构建 workspace（改了 src/ 代码后）
docker exec aider_ros2 bash -c "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install"

# 查看日志
docker compose -f docker-compose.ros.yml logs -f

# 停止
docker compose -f docker-compose.ros.yml down
```

## 演示

```bash
# RViz 看机器人（关节摆动）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_bringup view_aider.launch.py'

# 真实陀螺仪驱动（转动 IMU，机器人跟随）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_bringup imu_live.launch.py'

# PyBullet 仿真
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_sim pybullet.launch.py use_gui:=true with_wave:=true'
```

## 注意

- **ROS_DOMAIN_ID=42**：新家与老系统隔离，ROS 节点互不干扰
- **串口直通**：容器通过 `devices:` 直通 `/dev/ttyUSB0`（IMU）；换设备设 `IMU_PORT=/dev/ttyUSB1`
- **X11**：RViz/Gazebo 需本机 X server，`start-ros.sh` 已自动 `xhost +local:docker`
- **重建镜像**（改 Dockerfile 后）：`docker rmi aider_ros2:latest` 再启动
- **不要频繁改 Dockerfile**：任何改动都会让后面所有 layer 缓存失效，导致 ROS2 重装
