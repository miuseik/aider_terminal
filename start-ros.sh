#!/bin/bash
# start-ros.sh — 启动「新家」(src/) ROS 2 环境
# 等价于老业务系统的 start-pro.sh（同样是极简启动）
#
# 与老业务完全隔离：
#   老业务 → 容器 aiderminal
#   新家   → 容器 aider_ros2（只挂载 ./src 与 ./data）
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# BuildKit 会强制向 registry-1.docker.io 校验 manifest，国内网络必超时。
# 强制使用传统 builder + 本地已有的 ubuntu:noble，跳过校验。
export DOCKER_BUILDKIT=0

xhost +local:docker &>/dev/null || true

echo ">>> Aider 新家 — ROS 2 启动中"
docker compose -f docker-compose.ros.yml up -d

echo ""
echo ">>> 新家已启动"
echo ">>> 进入容器:   docker exec -it aider_ros2 bash"
echo ">>> 查看日志:   docker compose -f docker-compose.ros.yml logs -f"
echo ">>> 停止服务:   docker compose -f docker-compose.ros.yml down"
