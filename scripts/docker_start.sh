#!/bin/bash
set -e

# ============================================
# 后续运行（容器已存在，跳过编译）
#   ./scripts/docker_start.sh
#   ./scripts/docker_start.sh aloha
# ============================================

ROBOT="${1:-aider}"

docker start aiderminal

echo ">>> 启动 terminal_node (robot=$ROBOT) ..."
docker exec -it aiderminal bash -c "
    export PYTHONPATH=/ws/src/aiderminal:\$PYTHONPATH && \
    source /opt/ros/jazzy/setup.bash && \
    source /ws/install/setup.bash && \
    ros2 run aiderminal terminal_node --ros-args -p robot_type:=$ROBOT -p no_robot:=true
"
