# 新家 (src/) — ROS 2 快速启动

> 给新同事的一键启动指南。与老业务 `aiderminal` 完全隔离，
> 使用独立容器 `aider_ros2` / 镜像 `aider_ros2:latest`。

## 依赖说明（大小限制的取舍）

| 依赖 | 来源 | 大小 | 说明 |
|------|------|------|------|
| Docker | 离线包 Gitee 仓库 / 系统自带 | ~100MB | 新机器自动装，老机器复用 |
| `ubuntu:noble` 基础镜像 | 离线包 Gitee 仓库 | ~70MB | 卡在 Gitee 100MB 单文件限制内 |
| ROS 2 Jazzy + RViz | 清华 apt 源在线装 | 首次 ~2GB | 仅首次构建需联网，之后秒级 |
| Gazebo + PyBullet | 清华 apt 源在线装 | 首次 ~3GB | 仅首次，之后秒级 |

**设计原则**：基础镜像离线（不卡 Docker Hub），系统包在线（走国内源）。
镜像构建产物 `aider_ros2:latest` 在本机缓存，**后续启动无需重新构建**。
不把完整镜像塞进 Gitee（免费版单文件 100MB 限制，装不下 5GB+ 镜像）。

## 一、新机器部署

```bash
# 克隆项目
git clone <你的仓库> aider_terminal
cd aider_terminal

# 一键部署（自动装 Docker + 基础镜像 + 构建 + 启动）
./scripts/setup_ros.sh
```

`setup_ros.sh` 会自动处理：
1. Docker 未装/过旧 → 从 Gitee 离线包安装
2. `ubuntu:noble` 缺失 → 从离线包加载
3. 构建 `aider_ros2:latest`（含 ROS2 + RViz + Gazebo + PyBullet）
4. 启动容器 `aider_ros2`

## 二、日常启动

```bash
./start-ros.sh            # 启动容器（若已存在，秒级）
./start-ros.sh shell      # 进入容器 bash
./start-ros.sh build      # 改代码后重新构建 workspace
./start-ros.sh run <cmd>  # 执行任意 ros2 命令（如 ros2 pkg list）
./start-ros.sh stop       # 停止容器
./start-ros.sh clean      # 停止并清空构建缓存（彻底重来）
```

## 三、常用演示

```bash
# 1. 查看 Aider 机器人（RViz + 关节摆动）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_bringup view_aider.launch.py'

# 2. 真实陀螺仪实时驱动（转动 IMU，机器人跟随）
docker exec -d aider_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export DISPLAY=:0 && ros2 launch robot_bringup imu_live.launch.py'

# 3. PyBullet 仿真（老系统仿真器移植）
ros2 launch robot_sim pybullet.launch.py with_wave:=true

# 4. Gazebo 仿真（用于强化学习）
ros2 launch robot_sim gazebo.launch.py headless:=true
```

## 四、架构速览

```
src/                           # 标准 ROS 2 workspace (28 包)
├── robot_description/         # URDF/xacro/meshes（模型）
├── robot_sensors/             # 传感器（IMU 驱动 + 发布节点）
├── robot_control/             # 控制（关节摆动演示）
├── robot_sim/                 # 仿真（PyBullet + Gazebo）
├── robot_bringup/             # launch 编排
├── robot_msgs/                # 自定义消息
└── robot_hardware/ ...        # 其余包（ros2_control 插件等）
```

## 五、注意

- **ROS_DOMAIN_ID=42**：新家与老系统隔离，两者 ROS 节点互不干扰
- **串口直通**：容器通过 `devices:` 直通 `/dev/ttyUSB0`（IMU）
- **X11 显示**：RViz/Gazebo 需要本机 X server，`start-ros.sh` 已自动 `xhost +local:docker`
- **重建镜像**（改了 Dockerfile 后）：`docker rmi aider_ros2:latest && ./start-ros.sh up`
- **彻底清理**：`./start-ros.sh clean` 会删除构建缓存卷
