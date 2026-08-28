# robot_deploy_system — ROS 2 机器人部署系统骨架 (dev/ros 分支)

> 纯骨架（空包 + 占位文件），不含实现逻辑。结构参照"面向部署产品的工程框架"：
> 按**运行阶段/能力层级**切分，而非按技术来源。

## 分层

```
基础设施层   robot_bringup / robot_description / robot_hardware / robot_inputs / robot_msgs / robot_utils
控制执行层   robot_control / robot_manipulation / robot_navigation / robot_perception / robot_sensors
空间感知层   robot_slam / robot_localization / robot_state_estimation
推理执行层   robot_inference/ (rl_policy_executor / vla_executor / skill_executor / model_manager)
在线适应层   robot_adaptation/ (adaptive_controller / online_calibration / parameter_tuner)
智能层       robot_agent / robot_interaction / robot_safety / robot_diagnostics / robot_monitoring
支撑层       robot_sim / robot_tests
```

## 关键设计

- **robot_inference 是推理执行层**：RL/VLA/技能/模型管理都是"把模型变成动作的执行器"，
  由 robot_agent 经 model_manager 的 MCP Server 统一调度（mcp_rl/mcp_vla/mcp_slam/mcp_navigate/mcp_hardware/mcp_exo）。
- **空间感知三拆**：slam(建图) / localization(滤波定位) / state_estimation(接触/动力学) 职责分离。
- **robot_adaptation 补在线学习空白**：轻量自适应控制/校准/调参，对应真实项目里的限位热更新、IMU 校准。
- **产品化支撑**：safety/diagnostics/monitoring 独立成包。

## 构建

```bash
cd robot_deploy_system
colcon build --symlink-install
source install/setup.bash
```
