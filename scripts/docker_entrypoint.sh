#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# 清理 Python bytecode 缓存，避免旧 .pyc 导致代码不生效
find /ws/src -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
find /ws/src -name '*.pyc' -type f -delete 2>/dev/null || true
rm -f /tmp/pybullet_diag.log 2>/dev/null || true

# 源码通过 volume 挂载，每次启动自动 colcon build（--symlink 秒级完成）
echo ">>> colcon build..."
cd /ws && colcon build --symlink-install --packages-select aiderminal 2>&1 | tail -5

source /ws/install/setup.bash
export PYTHONPATH=/ws/src/aiderminal:$PYTHONPATH

ROBOT_TYPE="${ROBOT_TYPE:-aider}"
NO_ROBOT="${NO_ROBOT:-true}"

echo ">>> 启动 terminal_node (robot=$ROBOT_TYPE, no_robot=$NO_ROBOT)"
exec ros2 launch aiderminal terminal.launch.py \
    robot_type:=$ROBOT_TYPE \
    no_robot:=$NO_ROBOT \
    "$@"
