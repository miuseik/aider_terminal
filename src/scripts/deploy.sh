#!/usr/bin/env bash
# 部署到机器人（占位脚本）
set -euo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"
TARGET="${1:-aider@aider.local}"
echo "Deploying to $TARGET ..."
# rsync -avz --exclude build --exclude install --exclude log "$WS_ROOT/" "$TARGET:~/robot_ws/src/"
echo "deploy.sh: 待实现"
