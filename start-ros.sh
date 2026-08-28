#!/bin/bash
# start-ros.sh — 启动「新家」(src/) 的纯净 ROS 2 环境
#
# 与 start-dev.sh / start-pro.sh 完全隔离：
#   老业务 → 容器 aiderminal（挂载 . 到 /ws/src/aiderminal）
#   新家   → 容器 aider_ros2  （挂载 ./src 到 /ws/src）
#
# 用法:
#   ./start-ros.sh              构建并启动容器（后台常驻）
#   ./start-ros.sh shell        启动后进入容器 bash
#   ./start-ros.sh build        仅重新构建 workspace
#   ./start-ros.sh run <参数>   在容器内执行 ros2 命令
#       例: ./start-ros.sh run ros2 pkg list
#   ./start-ros.sh stop         停止并移除容器
#   ./start-ros.sh clean        停止容器并清除构建卷（彻底重来）
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

COMPOSE_FILE="docker-compose.ros.yml"
CONTAINER="aider_ros2"

# BuildKit 会强制向 registry-1.docker.io 校验 manifest，国内网络必超时。
# 强制使用传统 builder + 本地已有的 ubuntu:noble，跳过校验。
export DOCKER_BUILDKIT=0
export COMPOSE_DOCKER_CLI_BUILD=0

xhost +local:docker &>/dev/null || true

case "${1:-up}" in
  up)
    echo ">>> 构建镜像并启动新家容器 ..."
    ENV=dev docker compose -f "$COMPOSE_FILE" up -d --build
    echo ""
    echo ">>> 新家已启动"
    echo ">>> 进入容器:   ./start-ros.sh shell"
    echo ">>> 查看日志:   docker compose -f $COMPOSE_FILE logs -f"
    echo ">>> 停止服务:   ./start-ros.sh stop"
    ;;

  shell)
    docker exec -it "$CONTAINER" bash
    ;;

  build)
    echo ">>> 重新构建 workspace ..."
    docker exec "$CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install 2>&1 | tail -25"
    ;;

  run)
    shift
    [ $# -eq 0 ] && { echo "用法: ./start-ros.sh run <命令>"; exit 1; }
    docker exec "$CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && $*"
    ;;

  stop)
    echo ">>> 停止新家容器 ..."
    docker compose -f "$COMPOSE_FILE" down
    ;;

  clean)
    echo ">>> 停止容器并清除构建卷 ..."
    docker compose -f "$COMPOSE_FILE" down -v
    ;;

  *)
    echo "用法: ./start-ros.sh [up|shell|build|run <cmd>|stop|clean]"
    exit 1
    ;;
esac
