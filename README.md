# Aider Terminal

双 臂机器人遥操 作系统 —— PyBullet 仿真 · Pink IK · WebRTC 推流

> 本项目包含两套系统：
> - **老业务**（`aiderminal/`）—— 遥操作主系统，WebSocket 架构，仍在运行
> - **新家**（`src/`）—— 标准 ROS 2 workspace（28 个功能包），正在迁移中
>
> 两者**完全隔离**（不同容器/镜像/ROS 域），可同时运行，互不影响。

---

## 命令速查

### 老业务（遥操作系统）

| 场景 | 命令 |
|------|------|
| 新电脑，没有 Docker | `./scripts/setup.sh` |
| 有 Docker，第一次 | `docker compose up --build -d` |
| 之后每次启动（生产） | `./start-pro.sh` |
| 之后每次启动（本地） | `./start-dev.sh` |
| 启动 Aloha（生产） | `ROBOT_TYPE=aloha ./start-pro.sh` |
| 启动 Aloha（本地） | `ROBOT_TYPE=aloha ENV=dev docker compose up -d` |
| 改代码后重启 | `docker compose restart` |
| 停止 | `docker compose down` |
| 查看日志 | `docker compose logs -f` |
| 重建镜像 | `docker compose down && docker compose build --no-cache && docker compose up -d` |

### 新家（ROS 2 workspace）

| 场景 | 命令 |
|------|------|
| 新电脑，没有 Docker | `./scripts/setup_ros.sh` |
| 有 Docker，第一次 | `docker compose -f docker-compose.ros.yml up --build -d` |
| 之后每次启动 | `./start-ros.sh` |
| 进入容器 | `docker exec -it aider_ros2 bash` |
| 停止 | `docker compose -f docker-compose.ros.yml down` |
| 查看日志 | `docker compose -f docker-compose.ros.yml logs -f` |
```bash
./start-dev.sh
```
```bash
docker compose restart
```
```bash
./start-pro.sh
```
```bash
docker compose logs -f
```
```bash
docker compose down
```
---

## 环境变量

### 老业务

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_TYPE` | `aider` | `aider` / `aloha` |
| `NO_SIM` | `false` | 禁用仿真 |
| `NO_VIZ` | `false` | 无头模式 |

用法：`ROBOT_TYPE=aloha ./start-pro.sh`

### 新家

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DEPLOY_TARGET` | 自动判断 | `sim`（开发机）/ `runtime`（树莓派） |
| `IMU_PORT` | `/dev/ttyUSB0` | IMU 串口设备，直通进容器 |
| `DISPLAY` | `:0` | X11 显示号（RViz/Gazebo 需要） |

用法：

```bash
DEPLOY_TARGET=runtime ./scripts/setup_ros.sh   # 强制精简镜像
IMU_PORT=/dev/ttyUSB1 ./start-ros.sh           # 换串口设备
```

---

## 新家（ROS 2 workspace）完整说明

### 是什么

`src/` 是一个标准 ROS 2 workspace，28 个功能包，与老业务 `aiderminal/`
**完全隔离**：

| | 老业务 | 新家 |
|---|---|---|
| 容器 | `aiderminal` | `aider_ros2` |
| 镜像 | `aider_terminal-aiderminal` | `aider_ros2` |
| compose | `docker-compose.yml` | `docker-compose.ros.yml` |
| 挂载 | `.` → `/ws/src/aiderminal` | `./src` → `/ws/src` |
| ROS 域 | 默认 0 | **42**（避免节点互相发现） |

隔离设计目的：老业务仍在给别人演示，新家开发不能影响它。

### 怎么启动

```bash
# 新电脑（自动装 Docker + 基础镜像 + 构建 + 启动）
./scripts/setup_ros.sh

# 有 Docker，首次
docker compose -f docker-compose.ros.yml up --build -d

# 之后每次
./start-ros.sh

# 进入容器
docker exec -it aider_ros2 bash
```

容器内先 source 再用：

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
```

（容器内 workspace 已自动构建好，改了 `src/` 代码后重跑
`colcon build --symlink-install` 即可）

### 部署环境自动区分

`setup_ros.sh` 按 CPU 架构判断（`ARCH=$(uname -m)`）：

| 环境 | 架构 | 结果 | 镜像大小 | 内容 |
|------|------|------|---------|------|
| 开发机 | `x86_64` | `sim` | ~6.9GB | ROS2 + Gazebo + PyBullet + RViz |
| 树莓派 5 / Jetson | `aarch64` | `runtime` | ~1.5GB | 仅 ROS2 + ros2_control |

树莓派上不装 Gazebo（aarch64 跑不动 Ogre-Next 渲染，且没必要）。
强制指定：`DEPLOY_TARGET=runtime ./scripts/setup_ros.sh`

### 依赖策略（大小限制的取舍）

| 依赖 | 来源 | 大小 |
|------|------|------|
| Docker | Gitee 离线包 / 系统自带 | ~100MB |
| `ubuntu:noble` 基础镜像 | Gitee 离线包 | ~70MB |
| ROS 2 Jazzy | 清华 apt 源在线装 | 首次 ~2GB |
| Gazebo + PyBullet + RViz | 清华 apt 源在线装 | 首次 ~5GB（仅开发机） |

原则：**基础镜像离线**（不卡 Docker Hub），**系统包在线**（走国内源）。
不把完整镜像塞进 Gitee（免费版单文件 100MB 限制，装不下 6.9GB）。

### 演示

```bash
# 1. RViz 看机器人（关节摆动）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_bringup view_aider.launch.py'

# 2. 真实陀螺仪驱动（转动 IMU，机器人跟随）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_bringup imu_live.launch.py'

# 3. PyBullet 仿真（老系统仿真器移植）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_sim pybullet.launch.py use_gui:=true with_wave:=true'

# 4. Gazebo 仿真（用于强化学习）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_sim gazebo.launch.py'
```

### 目录结构

```
src/                          # 标准 ROS 2 workspace（28 包）
├── robot_description/        # URDF/xacro/meshes（aider + 多机型）
├── robot_sensors/            # 传感器（IMU 驱动 + 发布节点）
├── robot_control/            # 控制（关节摆动演示）
├── robot_sim/                # 仿真（PyBullet + Gazebo）
├── robot_bringup/            # launch 编排
├── robot_msgs/               # 自定义消息（9 msg + 4 srv + 3 action）
├── robot_hardware/           # ros2_control 插件
├── docs/ config/ scripts/    # 文档、配置、脚本
└── robot_*/                  # 其余功能包

data/                         # 数据（bags/maps/models/... 内容不入库）
docker/                       # 部署（Dockerfile.newros、entrypoint、compose 覆盖层）
```

### 新家踩过的坑（改前必读）

1. **BuildKit 必须禁用** —— 它会强制向 `registry-1.docker.io` 校验
   manifest，国内网络必超时。用 `DOCKER_BUILDKIT=0 --pull=false`，
   直接用本地 `ubuntu:noble`
2. **多阶段 `--target` 不生效** —— 传统 builder 下无效。用
   `ARG DEPLOY_TARGET` + `RUN` 内 shell `if` 替代
3. **不要频繁改 Dockerfile** —— 任何改动都让后面所有 layer 缓存失效，
   272 个 ROS2 包重装（曾因此白装三遍）
4. **`Command()` 输出必须包 `ParameterValue(value_type=str)`** ——
   否则 URDF 的 `<?xml` 被当 YAML 解析报错
5. **xacro 不支持字符串比较** —— `<xacro:if>` 只接受布尔表达式。
   多机型用路径拼接：`urdf/$(arg robot_type)/$(arg robot_type).urdf.xacro`
6. **ament_python 包要在 `setup.py` 的 `data_files` 声明 launch/rviz 目录**
   —— 否则 `ros2 launch` 找不到
7. **URDF 不设限位（lower==upper）时 PyBullet 把关节当 fixed** ——
   必须用限位表 patch 后再加载
8. **容器内 `pkill` 杀不净 ros2 launch 起的节点** —— 要干净环境需
   `docker rm -f` 重建容器

---

## 项目结构

```
aider_terminal/
├── aiderminal/        # Python 源码（老业务）
├── src/               # ROS 2 workspace（新家）
├── data/              # 数据目录（内容不入库）
├── docker/            # 新家部署文件
├── URDF/              # 机器人 URDF 模型（老业务用，勿删）
├── launch/            # 老业务 ROS 2 launch 文件
├── scripts/           # 辅助脚本
├── start-dev.sh       # 老业务启动（localhost）
├── start-pro.sh       # 老业务启动（生产）
├── start-ros.sh       # 新家启动
├── docker-compose.yml          # 老业务
├── docker-compose.ros.yml      # 新家
├── Dockerfile                  # 老业务
└── docker/Dockerfile.newros    # 新家
```

---

## 常见问题

```bash
xhost +local:docker                    # 仿真窗口弹不出来
sudo usermod -aG docker $USER          # docker 权限不够（执行后重新登录）
```

---

## 便捷操作（勿删）

```bash
git add ./;git commit -m "hub";git push
```

```bash
git pull
```

```bash
ssh gaoda@192.168.0.113

cd www/aider_terminal
密码：gaoda
```

```bash
ssh gaoda@192.168.0.113
#cd /www/aider/aider_terminal ;sudo git pull ; ./start-pro.sh
cd /www/aider/aider_terminal ;sudo git pull ; docker compose restart
密码：gaoda123
```

---

MIT
