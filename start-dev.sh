#!/bin/bash
# start-dev.sh — 本地开发模式（连接 localhost）
# 等价于 Vue 的 npm run dev
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

xhost +local:docker &>/dev/null || true

echo ">>> Aider Terminal — 本地开发模式 (localhost)"
ENV=dev docker compose up -d

echo ""
echo ">>> 已启动（localhost 模式）"
echo ">>> 查看日志: docker compose logs -f"
echo ">>> 停止服务: docker compose down"
