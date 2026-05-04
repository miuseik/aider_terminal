#!/bin/bash
# Aider Terminal 启动脚本
# 支持开发环境和生产环境

ENV=${1:-"pro"}  # 默认生产环境

echo "🔍 清理旧进程..."
pkill -9 -f "aider_terminal/main.py" 2>/dev/null
sleep 1
echo "✅ 清理完成"
echo ""

echo "🤖 Aider Terminal 启动脚本"
echo "📡 环境: $ENV"
echo ""

if [ "$ENV" = "dev" ]; then
    echo "🔧 使用开发环境 (localhost)"
    /home/miuseik/miniconda3/envs/aider/bin/python /home/miuseik/www/aider/aider_terminal/main.py --env-dev
else
    echo "🌐 使用生产环境 (ws.houqicg.com)"
    /home/miuseik/miniconda3/envs/aider/bin/python /home/miuseik/www/aider/aider_terminal/main.py
fi
