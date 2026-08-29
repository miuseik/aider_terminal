#!/bin/bash
set -e

# 清理 Python bytecode 缓存，避免旧 .pyc 导致代码不生效
find /app -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
find /app -name '*.pyc' -type f -delete 2>/dev/null || true
rm -f /tmp/pybullet_diag.log 2>/dev/null || true

ROBOT_TYPE="${ROBOT_TYPE:-aider}"
ENV="${ENV:-pro}"

ARGS=(--robot-type "$ROBOT_TYPE" --log-level warning)
if [ "$ENV" = "dev" ]; then
    ARGS+=(--env-dev)
fi
if [ "$NO_SIM" = "true" ]; then
    ARGS+=(--no-sim)
fi
if [ "$NO_VIZ" = "true" ]; then
    ARGS+=(--no-viz)
fi

echo ">>> 启动 src.app (robot=$ROBOT_TYPE, env=$ENV)"
cd /app && exec python3 -m src.app "${ARGS[@]}" "$@"
