#!/bin/bash
# 容器入口：加载 ROS 环境后启动入口脚本
set -e

# ROS 2 基础环境
if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

# 工作空间环境（若已 colcon build）
if [ -f /ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /ws/install/setup.bash
fi

# 项目入口脚本（挂载在 /ws/src/aiderminal/scripts/）
exec /bin/bash /ws/src/aiderminal/scripts/docker_entrypoint.sh "$@"
