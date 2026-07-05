#!/bin/bash
# ============================================================
# Aider Terminal — 新机器一键部署
#
# 用法:
#   ./scripts/setup.sh
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

# ═══════════════════════════════════════════════
# 配置区
# ═══════════════════════════════════════════════

# Docker 离线包仓库（临时 clone 到 /tmp，安装完自动清理）
OFFLINE_REPO="https://gitee.com/miuseik/docker-offline-29.6.1-multiarch.git"
OFFLINE_DIR="/tmp/docker-offline-29.6.1-multiarch"

# ═══════════════════════════════════════════════

# 项目根目录
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ═══════════════════════════════════════════════

# ── 1. 克隆离线包到临时目录 ──
if [ ! -d "$OFFLINE_DIR" ]; then
    warn "克隆 Docker 离线包..."
    git clone --depth 1 "$OFFLINE_REPO" "$OFFLINE_DIR"
fi

# ── 2. 离线安装 Docker（如未安装） ──
if ! command -v docker &>/dev/null; then
    if [ -f "$OFFLINE_DIR/install_docker.sh" ]; then
        warn "Docker 未安装，通过离线包安装..."
        sudo bash "$OFFLINE_DIR/install_docker.sh"
    else
        error "Docker 离线包不完整：$OFFLINE_DIR"
        exit 1
    fi
fi
info "Docker 已安装: $(docker --version)"

# ── 3. docker compose 检测 ──
if ! docker compose version &>/dev/null; then
    COMPOSE_SRC="$OFFLINE_DIR/compose-$(uname -m)"
    if [ -f "$COMPOSE_SRC" ]; then
        warn "docker compose 未生效，复制到标准插件路径..."
        sudo mkdir -p /usr/local/lib/docker/cli-plugins
        sudo cp "$COMPOSE_SRC" /usr/local/lib/docker/cli-plugins/docker-compose
        sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
    fi
    if ! docker compose version &>/dev/null; then
        error "docker compose 不可用，请手动检查"
        exit 1
    fi
fi
info "docker compose 可用: $(docker compose version | head -1)"
COMPOSE_CMD="docker compose"

# ── 4. docker socket 权限 ──
USE_SG=false
if ! docker ps &>/dev/null; then
    warn "当前 shell 无法访问 Docker socket，使用 sg docker 提权..."
    USE_SG=true
fi

docker_cmd() {
    if [ "$USE_SG" = true ]; then
        sg docker -c "$*"
    else
        eval "$*"
    fi
}

# ── 5. CD 到项目目录 ──
cd "$SCRIPT_DIR"

# ── 6. X11 权限 ──
if [ -n "$DISPLAY" ]; then
    xhost +local:docker &>/dev/null || true
fi

# ── 7. 加载基础镜像离线包（仅 x86_64，树莓派直接从 Docker Hub 拉） ──
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ] && [ -f "$OFFLINE_DIR/ubuntu-noble.tar.gz" ]; then
    info "加载基础镜像离线包..."
    docker_cmd "docker load -i $OFFLINE_DIR/ubuntu-noble.tar.gz"
fi

# ── 8. 构建镜像 ──
echo ""
echo ">>> 构建 Docker 镜像..."
echo "    ubuntu:noble 基础镜像已离线加载。"
echo "    构建过程会自动安装 ROS 系统包 (~500MB)，需要联网。"
echo "    后续启动无需重复构建，秒级完成。"
echo ""

docker_cmd "$COMPOSE_CMD build"
info "镜像构建完成"

# ── 9. 启动 ──
echo ""
# 清理旧容器避免冲突
docker_cmd "docker rm -f aiderminal 2>/dev/null || true"
docker_cmd "$COMPOSE_CMD up -d"

# ── 10. 等待启动 ──
sleep 3
RUNNING=$(docker_cmd "docker ps --format '{{.Names}}'" 2>/dev/null)

if echo "$RUNNING" | grep -q aiderminal; then
    info "项目已启动！"
    echo ""
    echo "  查看日志:  $COMPOSE_CMD logs -f"
    echo "  停止项目:  $COMPOSE_CMD down"
    echo "  改代码后:  $COMPOSE_CMD restart"
    echo ""
else
    error "启动失败，查看日志: $COMPOSE_CMD logs"
    docker_cmd "$COMPOSE_CMD logs --tail=30"
    exit 1
fi

if [ "$USE_SG" = true ]; then
    echo ""
    warn "已通过 sg 命令执行 docker 操作，无需重新登录"
fi

# ── 清理临时离线包 ──
if [ -d "$OFFLINE_DIR" ]; then
    rm -rf "$OFFLINE_DIR"
    info "临时离线包已清理"
fi
