#!/bin/bash
# ============================================================
# Aider Terminal — 新机器一条龙部署
#
# 用法:
#   ./scripts/setup.sh
#
# 适用于: Ubuntu 22.04 / 24.04, Debian 12
# ============================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[✓]${NC} $1"; }
warn()  { echo -e "${YELLOW}[!]${NC} $1"; }
error() { echo -e "${RED}[✗]${NC} $1"; }

echo "============================================"
echo " Aider Terminal 一键部署"
echo "============================================"

# ── 1. 检查 / 安装 Docker ──
if ! command -v docker &>/dev/null; then
    echo ""
    warn "Docker 未安装，正在安装..."
    sudo apt-get update -qq
    sudo apt-get install -y -qq docker.io
    info "Docker 安装完成"
else
    info "Docker 已安装: $(docker --version)"
fi

# ── 2. docker compose 插件 ──
if ! docker compose version &>/dev/null; then
    warn "安装 docker compose 插件..."
    sudo apt-get install -y -qq docker-compose-v2 2>/dev/null || \
        sudo apt-get install -y -qq docker-compose-plugin 2>/dev/null || true
    if ! docker compose version &>/dev/null; then
        error "docker compose 插件安装失败，请手动安装"
        exit 1
    fi
fi
info "docker compose 可用"

# ── 3. 当前用户加入 docker 组 ──
if ! groups "$USER" | grep -q docker; then
    warn "将 $USER 加入 docker 组..."
    sudo usermod -aG docker "$USER"
    warn "请退出终端重新登录，或执行: newgrp docker"
    NEED_RELOGIN=true
else
    info "$USER 已在 docker 组内"
fi

# ── 4. CD 到项目目录 ──
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

# ── 5. X11 权限 ──
if [ -n "$DISPLAY" ]; then
    xhost +local:docker &>/dev/null || true
fi

# ── 6. 构建镜像（首次从 Docker Hub 下载官方 ros:jazzy，约 3-5 分钟）──
echo ""
echo ">>> 构建 Docker 镜像..."
echo "    首次构建会下载 ROS 基础镜像 (~500MB)，请耐心等待。"
echo "    后续启动无需重复构建，秒级完成。"
echo ""
docker compose build
info "镜像构建完成"

# ── 7. 启动 ──
echo ""
docker compose up -d

# ── 8. 等待启动 ──
sleep 3
if docker ps --format '{{.Names}}' | grep -q aiderminal; then
    info "项目已启动！"
    echo ""
    echo "  查看日志:  docker compose logs -f"
    echo "  停止项目:  docker compose down"
    echo "  改代码后:  docker compose restart"
    echo ""
    docker compose logs --tail=10
else
    error "启动失败，查看日志: docker compose logs"
    docker compose logs --tail=30
    exit 1
fi

if [ "$NEED_RELOGIN" = true ]; then
    echo ""
    warn "请退出终端并重新登录，使 docker 组权限生效"
fi
