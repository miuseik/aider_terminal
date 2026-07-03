#!/bin/bash
set -e

# ============================================
# 首次：创建容器 + 编译 + 运行
#   ./scripts/docker_run.sh
#   ./scripts/docker_run.sh -- aloha         # 用 Aloha 机器人
#   ./scripts/docker_run.sh -- aider --env-dev # 开发环境
# ============================================

ROBOT="${1:-aider}"
EXTRA_ARGS="${2:-}"

# 允许 X11
xhost +local:docker

# 创建容器（如果不存在）
if ! docker ps -a --format '{{.Names}}' | grep -q '^aiderminal$'; then
    echo ">>> 创建容器 aiderminal ..."
    docker run -d --name aiderminal \
        -v /home/miuseik/www/aider/aider_terminal:/ws/src/aiderminal:rw \
        -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
        -e DISPLAY=$DISPLAY \
        --network host \
        --device /dev/video0:/dev/video0 \
        aider_ros:x11 \
        bash -c "tail -f /dev/null"
else
    docker start aiderminal
fi

# 编译
echo ">>> colcon build ..."
docker exec aiderminal bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd /ws && colcon build --symlink-install --packages-select aiderminal
"

# 运行
echo ">>> 启动 terminal_node (robot=$ROBOT) ..."
docker exec -it aiderminal bash -c "
    export PYTHONPATH=/ws/src/aiderminal:\$PYTHONPATH && \
    source /opt/ros/jazzy/setup.bash && \
    source /ws/install/setup.bash && \
    ros2 run aiderminal terminal_node --ros-args -p robot_type:=$ROBOT -p no_robot:=true $EXTRA_ARGS
"
