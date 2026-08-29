# 新家 ROS 2 工作区目录结构（树状）

> 本文件描述 `aider_terminal/src/` 这个 **标准 ROS 2 工作区**的完整包结构蓝图。
> 构建系统：`colcon`；发行版：`Jazzy` (LTS)。
> 约定：`src/<pkg>/` 每个目录是一个独立 ROS 2 功能包；`install/ build/ log/` 由 `colcon build` 自动生成，不提交。
>
> 与 `aider_terminal` 老业务 (`aiderminal/`) 的映射见文末「与你 `aider_terminal` 的映射」表。

```
robot_framework_ws/                         # 工作空间根 (colcon workspace)
├── src/                                    # 所有源码功能包
│   │
│   ├── 📦 robot_bringup/                   # ① 启动装配包 (唯一入口, 组合所有 launch)
│   │   ├── launch/
│   │   │   ├── robot.launch.py             #   总启动: 描述+硬件+控制+感知+导航
│   │   │   ├── description.launch.py       #   URDF/机器人模型载入 + robot_state_publisher
│   │   │   ├── hardware.launch.py          #   硬件接口 + 驱动
│   │   │   ├── sensors.launch.py          #   相机/IMU/雷达/LiDAR
│   │   │   ├── control.launch.py          #   控制器管理器 (ros2_control)
│   │   │   ├── perception.launch.py       #   SLAM/建图/定位
│   │   │   ├── navigation.launch.py       #   导航栈
│   │   │   ├── manipulation.launch.py     #   机械臂 (可选)
│   │   │   ├── agent.launch.py            #   Agent 大脑 + MCP Servers
│   │   │   └── interaction.launch.py      #   HMI/语音/VLA
│   │   ├── config/
│   │   │   ├── robot_controllers.yaml      #   ros2_control 控制器参数
│   │   │   ├── nav2_params.yaml            #   导航参数
│   │   │   ├── slam_params.yaml            #   SLAM 参数
│   │   │   ├── localization.yaml           #   定位 (robot_localization) 参数
│   │   │   └── twist_mux.yaml              #   指令多路复用参数
│   │   ├── urdf/                           #   机器人描述 (XACRO 宏, 支持多形态)
│   │   │   ├── robot.urdf.xacro            #   总装 (include 各肢体)
│   │   │   ├── biped/                      #   双足宏 (腿+躯干+头)
│   │   │   ├── wheeled/                    #   轮式宏 (底盘+轮+关节)
│   │   │   ├── quadruped/                  #   机器狗宏 (4腿+躯干)
│   │   │   ├── sensors/                    #   传感器挂载 (camera/imu/lidar)
│   │   │   └── actuators/                  #   执行器挂载 (舵机/电机)
│   │   ├── rviz/
│   │   │   └── robot.rviz                   #   3D 可视化配置
│   │   ├── maps/                           #   已建地图 (.pgm/.yaml)
│   │   ├── worlds/                         #   Gazebo 仿真世界 (.sdf/.world)
│   │   └── package.xml
│   │
│   ├── 📦 robot_description/              # ② 模型与 tf 真源 (URDF+Mesh)
│   │   ├── urdf/
│   │   ├── meshes/                         #   3D 模型 (.stl/.dae/.obj)
│   │   │   ├── biped/
│   │   │   ├── wheeled/
│   │   │   └── quadruped/
│   │   ├── launch/
│   │   │   └── description.launch.py       #   robot_state_publisher
│   │   └── package.xml
│   │
│   ├── 📦 robot_msgs/                     # ③ 自定义消息/服务/动作 (独立包, 防循环依赖)
│   │   ├── msg/
│   │   │   ├── RobotState.msg             #   模式/电量/故障/状态
│   │   │   ├── ControlGoal.msg            #   高层控制目标
│   │   │   ├── JointCommand.msg           #   关节指令
│   │   │   ├── GaitState.msg              #   步态状态 (腿式)
│   │   │   ├── IMUArray.msg               #   多 IMU
│   │   │   └── ExoInput.msg              #   外骨骼输入
│   │   ├── srv/
│   │   │   ├── SetMode.srv                #   切换控制模式
│   │   │   ├── Calibrate.srv             #   标定触发
│   │   │   └── GetRobotState.srv
│   │   ├── action/
│   │   │   ├── Navigate.action            #   导航目标+反馈+结果
│   │   │   ├── Grasp.action               #   抓取
│   │   │   └── Trajectory.action          #   轨迹执行
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── 📦 robot_hardware/                 # ④ 硬件接口层 (ros2_control + 驱动)
│   │   ├── hardware_interface/            #   System/Actuator 插件
│   │   │   ├── biped_system.cpp           #   双足硬件接口
│   │   │   ├── wheeled_system.cpp         #   轮式硬件接口
│   │   │   ├── quadruped_system.cpp       #   机器狗硬件接口
│   │   │   └── plugin_description.xml
│   │   ├── drivers/                       #   底层 SDK/协议 驱动 (独立, 零 ROS 依赖)
│   │   │   ├── camera/
│   │   │   │   ├── astra_driver/          #   奥比中光 RGB-D (C/C++ SDK)
│   │   │   │   └── opencv_driver/         #   单目 OpenCV
│   │   │   ├── imu/
│   │   │   │   └── hiwonder_imu/          #   幻尔十轴 AHRS (串口)
│   │   │   ├── lidar/
│   │   │   │   └── rplidar_driver/        #   雷达
│   │   │   ├── actuator/                  #   执行器
│   │   │   │   ├── robstride/             #   轮毂/关节电机 (CAN)
│   │   │   │   ├── feetech/               #   舵机 (串口)
│   │   │   │   └── dynamixel/             #    dynamixel 总线
│   │   │   └── exo/                       #   外骨骼 ESP32 (WiFi/串口)
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_sensors/                  # ⑤ 感知采集 (生产 sensor_msgs)
│   │   ├── nodes/
│   │   │   ├── camera_node.cpp/.py        #   → sensor/rgbd (Image×2)
│   │   │   ├── imu_node.cpp/.py           #   → sensor/imu (Imu)
│   │   │   ├── lidar_node.cpp/.py         #   → sensor/scan (LaserScan)
│   │   │   ├── pointcloud_node.cpp/.py    #   → sensor/pointcloud (PointCloud2)
│   │   │   └── encoder_node.cpp/.py       #   → joint_states
│   │   ├── config/
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_perception/               # ⑥ 感知理解与建图
│   │   ├── nodes/
│   │   │   ├── slam_node.cpp/.py          #   订阅sensor/* → 发布 map/pose
│   │   │   ├── localization_node.cpp/.py  #   robot_localization (EKF/UKF)
│   │   │   ├── detection_node.cpp/.py     #   目标检测 (YOLO/视觉)
│   │   │   ├── segmentation_node.cpp/.py  #   语义分割
│   │   │   └── depth_proc_node.cpp/.py    #   深度处理/点云滤波
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_navigation/               # ⑦ 导航与运动规划
│   │   ├── nodes/
│   │   │   ├── planner_node.cpp/.py       #   全局规划 (A*/RRT*)
│   │   │   ├── controller_node.cpp/.py    #   局部控制 (MPC/pure pursuit)
│   │   │   ├── behavior_tree/             #   行为树 XML (Nav2 BT)
│   │   │   │   └── navigate_bt.xml
│   │   │   ├── costmap_node.cpp/.py       #   代价地图
│   │   │   └── gait_node.cpp/.py          #   步态生成 (腿式专用)
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_manipulation/             # ⑧ 机械臂操作 (MoveIt 2, 可选)
│   │   ├── config/
│   │   │   ├── kinematics.yaml            #   IK 求解器
│   │   │   ├── joint_limits.yaml
│   │   │   └── controllers.yaml
│   │   ├── launch/
│   │   │   └── moveit.launch.py
│   │   ├── meshes/
│   │   └── package.xml
│   │
│   ├── 📦 robot_inputs/                   # ⑨ 控制输入 (指挥源)
│   │   ├── nodes/
│   │   │   ├── vr_input_node.cpp/.py       #   VR 手柄 → ControlGoal
│   │   │   ├── keyboard_node.cpp/.py       #   键盘
│   │   │   ├── exo_input_node.cpp/.py      #   外骨骼 → 关节角
│   │   │   ├── joy_node.cpp/.py            #   游戏手柄
│   │   │   └── ai_input_node.cpp/.py       #   AI/VLA 指令
│   │   ├── config/
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_interaction/              # ⑩ 人机交互 (HMI/语音 + VLA 执行)
│   │   ├── nodes/
│   │   │   ├── web_bridge_node.cpp/.py     #   WS↔ROS 桥 (rosbridge 风格)
│   │   │   ├── voice_node.cpp/.py          #   语音识别/合成
│   │   │   ├── vla_node.cpp/.py            #   VLA 执行器: 视觉+语言 → 动作 (被 Agent 经 MCP 调用)
│   │   │   └── hmi_node.cpp/.py           #   屏幕/表情
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_agent/                    # ⑪ 自主决策大脑 (VLA+AGENT+MCP 中枢)
│   │   ├── nodes/
│   │   │   ├── agent_node.cpp/.py          #   总控: 目标分解+任务调度+记忆+纠错监控
│   │   │   ├── memory_node.cpp/.py          #   长期/场景记忆 (RAG, 用户偏好/历史)
│   │   │   ├── planner_node.cpp/.py         #   任务规划 (LLM/PDDL/行为树)
│   │   │   └── mcp_client_node.cpp/.py      #   MCP 客户端: 统一调用各 MCP Server
│   │   ├── mcp_servers/                    #   各异构工具的 MCP 封装 (可独立部署)
│   │   │   ├── mcp_vla/                    #   包 aider_vla (视觉-语言-动作)
│   │   │   │   ├── server.py               #   MCP Server: tools(VLA推理/抓取/识别)
│   │   │   │   └── package.xml
│   │   │   ├── mcp_slam/                   #   包 aider_slam (建图/定位)
│   │   │   │   ├── server.py               #   MCP Server: tools(获取pose/地图/重定位)
│   │   │   │   └── package.xml
│   │   │   ├── mcp_navigate/              #   包 terminal core (导航/规划)
│   │   │   │   ├── server.py               #   MCP Server: tools(导航到/规划路径)
│   │   │   │   └── package.xml
│   │   │   ├── mcp_hardware/              #   包 robots/aider/adapter
│   │   │   │   ├── server.py               #   MCP Server: tools(设关节角/读状态/急停)
│   │   │   │   └── package.xml
│   │   │   └── mcp_exo/                   #   包 ESP32 外骨骼
│   │   │       ├── server.py               #   MCP Server: tools(设外骨骼关节)
│   │   │       └── package.xml
│   │   ├── config/
│   │   │   ├── agent_prompts.yaml          #   Agent 系统提示/角色
│   │   │   └── mcp_routing.yaml            #   工具路由/权限/限流
│   │   ├── launch/
│   │   │   └── agent.launch.py             #   启动 agent + 各 mcp_server
│   │   └── package.xml
│   │
│   ├── 📦 robot_control/                  # ⑫ 控制仲裁与模式管理
│   │   ├── nodes/
│   │   │   ├── twist_mux_node.cpp/.py      #   多源指令仲裁/优先级
│   │   │   ├── mode_manager_node.cpp/.py   #   控制模式切换 (pure_vr/exo_mixed...)
│   │   │   └── safety_node.cpp/.py         #   安全监控/急停
│   │   ├── config/
│   │   ├── launch/
│   │   └── package.xml
│   │
│   ├── 📦 robot_sim/                       # ⑬ 仿真 (PyBullet + Gazebo)
│   │   ├── worlds/
│   │   ├── models/
│   │   ├── launch/
│   │   │   ├── pybullet.launch.py          #   PyBullet 仿真 (已就绪, 老系统移植)
│   │   │   ├── gazebo.launch.py            #   Gazebo 仿真 (RL 用, 空壳待接)
│   │   │   └── sim.launch.py               #   仿真替代 hardware (占位)
│   │   └── package.xml
│   │
│   ├── 📦 robot_tests/                     # ⑭ 测试 (unittest/launch_testing)
│   │   ├── test/
│   │   │   ├── test_sensors.py
│   │   │   ├── test_control.py
│   │   │   ├── test_agent.py               #   Agent 任务分解/工具调用测试
│   │   │   └── test_launch.py
│   │   └── package.xml
│   │
│   └── 📦 robot_utils/                     # ⑮ 公共工具库 (无节点)
│       ├── include/robot_utils/
│       │   ├── math_utils.hpp              #   坐标变换/四元数
│       │   ├── tf_utils.hpp                #   tf2 辅助
│       │   └── ros_utils.hpp               #   QoS/参数辅助
│       ├── src/
│       └── package.xml
│
├── install/                                # colcon build 产物 (自动生成, 不提交)
├── build/                                  # 编译中间文件 (自动生成)
├── log/                                    # 运行日志 (自动生成)
├── .colcon.meta                            # 并行构建/忽略配置
├── .repos                                  # 多仓库依赖 (vcs import)
└── README.md                               # 框架说明 + 构建指南
```

---

## 各包职责速查表

| 包 | ROS 角色 | 输入 | 输出 | 多形态差异 |
|----|----------|------|------|-----------|
| `robot_bringup` | 装配入口 | — | launch 组合 | 切换 urdf/launch 即换形态 |
| `robot_description` | tf 真源 | XACRO | `/tf` `/robot_description` | biped/wheeled/quadruped 三套宏 |
| `robot_msgs` | 接口定义 | — | `.msg/.srv/.action` | 通用 + GaitState(腿式) |
| `robot_hardware` | 硬件接口 | SDK 原始 | `/joint_states` `/command` | System 插件按形态分 |
| `robot_sensors` | 感知采集 | 硬件 | `sensor/*` | 共用 |
| `robot_perception` | 理解建图 | `sensor/*` | `map` `pose` `detection` | 共用 |
| `robot_navigation` | 规划控制 | `pose`+`map`+`goal` | `twist`/`gait` | 轮式用MPC, 腿式用gait |
| `robot_manipulation` | 操作 | `sensor/*`+`goal` | 关节轨迹 | 可选 |
| `robot_inputs` | 指令源 | 人/AI | `ControlGoal` | 共用 |
| `robot_interaction` | HMI+VLA执行 | `sensor/*`+状态 | 对话/视觉动作 | 共用 (VLA 经 MCP 被 Agent 调) |
| `robot_agent` | 自主决策中枢 | 高层目标/记忆 | 任务分解→MCP工具调用 | 共用 (VLA+AGENT+MCP 核心) |
| `robot_control` | 仲裁 | 多源指令 | 最终 `/command` | 模式管理 |
| `robot_sim` | 仿真 | — | 虚拟硬件 | 替代 hardware 包 (PyBullet 已就绪) |
| `robot_tests` | 验证 | — | 测试报告 | 共用 |
| `robot_utils` | 工具 | — | 库函数 | 共用 |

### VLA + AGENT + MCP 三层关系

```
用户目标 / 语音
   │
   ▼
robot_agent/agent_node          ← AGENT: 分解任务 + 调度 + 记忆 + 纠错
   ├─ memory_node               (长期/场景记忆, RAG)
   ├─ planner_node              (任务规划: LLM/PDDL/行为树)
   └─ mcp_client_node ──(MCP)──▶ mcp_vla      → aider_vla   (VLA 执行: 视觉-语言-动作)
                               ──▶ mcp_slam     → aider_slam  (建图/定位)
                               ──▶ mcp_navigate → terminal core (导航/规划)
                               ──▶ mcp_hardware → adapter     (关节/状态/急停)
                               ──▶ mcp_exo      → ESP32       (外骨骼)
                                        │
                                        ▼
                              机器人动作 / 状态反馈 → Agent 监控纠错
```

- **VLA** = 执行肌肉（视觉+语言→动作），作为 MCP 工具被 Agent 调用，而非独立决策。
- **AGENT** = 任务大脑（多任务分解/并行/记忆/纠错），是必需调度中枢。
- **MCP** = 统一接口层，让 Agent 干净对接一堆异构私有工具（aider_vla/aider_slam/terminal/ESP32），可插拔、可观测。

> 多任务结论：端到端 VLA 自身应付不了多任务并发/长程分解；AGENT 必上，MCP 在工具异构场景下必上。

---

## 多形态（双足/轮式/机器狗）如何共用

1. **URDF 宏切换**：`robot.urdf.xacro` 用参数 `<xacro:arg name="type" default="quadruped"/>` 选择 `biped/` `wheeled/` `quadruped/` 子宏。
2. **硬件接口插件切换**：`robot_hardware/hardware_interface/` 下三个 System 插件，launch 里按形态 `type` 加载对应插件。
3. **导航差异**：轮式走 `controller_node`(MPC/pure pursuit)；腿式额外走 `gait_node`(步态生成)，`twist`→`gait` 转换。
4. **其余包（sensors/perception/inputs/interaction/control）完全共用**，不感知形态。

---

## 与你 `aider_terminal` 的映射

| ROS 2 标准包 | 你的项目 | 备注 |
|--------------|----------|------|
| `robot_hardware/drivers/*` | `aider_camera/` `aider_sensors/`(驱动) | 已独立 ✅ |
| `robot_sensors` | `aider_sensors/SensorManager` | 应统一编排相机+IMU |
| `robot_inputs` | `aiderminal/inputs/` | ✅ |
| `robot_perception` | `aider_slam`(外部) | 经 WS 消费 sensor/* |
| `robot_navigation` | `aiderminal/core/` + 规划 | 自写 |
| `robot_control` | `aiderminal/router/` + 模式管理 | ✅ |
| `robot_interaction` | `aider_vla`(外部) + HMI | 经 WS/HTTP (VLA 作为 MCP 工具) |
| `robot_agent` | (待建) Agent 中枢 + `mcp_*` Servers | 包 aider_vla/aider_slam/terminal/ESP32 |
| `aider_server` | `robot_bringup`+Master(DDS) | 你的 WS 总线替代 DDS |
| `robot_description` | `URDF/` | 已有 ✅ |

> 你的**逻辑分层已等于此框架**；差异仅在"包→模块、DDS→WS、launch→app.py、Pinocchio(Python)→C++ pinocchio/KDL"。
> 当前缺口：① `robot_agent` + `mcp_*` 尚未在 `aider_terminal` 落地（外部服务 aider_vla/aider_slam 已存在，需补 MCP 封装 + Agent 节点）；② `SensorManager` 应统一编排相机+IMU。
