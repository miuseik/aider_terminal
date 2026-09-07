#!/bin/bash
# docker_start.sh — 启动 Aider Terminal（兼容旧习惯）
#
# 用法:
#   ./scripts/docker_start.sh          # 默认 aider 机器人
#   ./scripts/docker_start.sh aloha     # 指定机器人
#
# 所有新电脑都一样，无需任何前置步骤。
# 首次会构建镜像（约 3-5 分钟），后续秒开。
set -e

ROBOT="${1:-aider}"
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

# 允许容器访问 X11（仿真窗口用）
xhost +local:docker &>/dev/null || true

# 一条命令：构建（如果需要） + 启动
echo ">>> 启动 Aider Terminal (robot=$ROBOT) ..."
docker compose down --remove-orphans &>/dev/null || true
ROBOT_TYPE=$ROBOT docker compose up --build -d

echo ""
echo ">>> 已启动"
echo ">>> 查看日志: docker compose logs -f"
echo ">>> 停止服务: docker compose down"
