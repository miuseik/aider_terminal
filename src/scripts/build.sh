#!/usr/bin/env bash
# 构建工作空间（占位脚本）
set -euo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"
echo "Building workspace at $WS_ROOT ..."
# colcon build --symlink-install
echo "build.sh: 待实现"
