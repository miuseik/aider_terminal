#!/bin/bash
# docker_run.sh — 在容器内运行命令
#
# 用法:
#   ./scripts/docker_run.sh bash                 # 进入容器终端
#   ./scripts/docker_run.sh python3 some_script.py
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

docker compose exec aiderminal "$@"
