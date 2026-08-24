#!/bin/bash
# start-pro.sh — 生产模式（连接 bot.houqicg.com / server.houqicg.com）
# 等价于 Vue 的 npm run pro
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

xhost +local:docker &>/dev/null || true

echo ">>> Aider Terminal — 生产模式 (bot.houqicg.com / server.houqicg.com)"
docker compose up -d

echo ""
echo ">>> 已启动（生产模式）"
echo ">>> 查看日志: docker compose logs -f"
echo ">>> 停止服务: docker compose down"
