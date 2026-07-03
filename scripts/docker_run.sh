#!/bin/bash
# 首次部署：构建镜像 + 删除旧容器 + 启动项目（带仿真窗口）
#   sudo ./scripts/docker_run.sh
#   sudo ./scripts/docker_run.sh aloha
set -e

ROBOT="${1:-aider}"
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

xhost +local:docker

echo ">>> 构建 Docker 镜像..."
cd "$SCRIPT_DIR" && sudo docker build -t aider_ros:x11 .

echo ">>> 删除旧容器..."
sudo docker rm -f aiderminal 2>/dev/null || true

echo ">>> 启动项目 (robot=$ROBOT，带仿真窗口) ..."
sudo docker run -d --name aiderminal \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    -e ROBOT_TYPE=$ROBOT \
    -e NO_ROBOT=true \
    aider_ros:x11

echo ">>> 日志："
sudo docker logs -f aiderminal
