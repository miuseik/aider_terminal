#!/bin/bash
# ============================================================
# 新家 (src/) — ROS 2 一键部署
#
# 沿用 scripts/setup.sh 的离线包机制（Docker Hub 不可达时仍可部署），
# 但只构建「新家」镜像，与老业务 aiderminal 完全隔离。
#
# 用法:
#   ./scripts/setup_ros.sh
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
echo " 新家 (src/) ROS 2 一键部署"
echo "============================================"

# ═══════════════════════════════════════════════
# 配置区（与 setup.sh 保持一致）
# ═══════════════════════════════════════════════
OFFLINE_REPO="https://gitee.com/miuseik/docker-offline-29.6.1-multiarch.git"
OFFLINE_DIR="/tmp/docker-offline-29.6.1-multiarch"

IMAGE_NAME="aider_ros2:latest"
COMPOSE_FILE="docker-compose.ros.yml"
CONTAINER="aider_ros2"
# ═══════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

ARCH=$(uname -m)

# ── 1. 克隆离线包到临时目录 ──
if [ ! -d "$OFFLINE_DIR" ]; then
    warn "克隆 Docker 离线包..."
    git clone --depth 1 "$OFFLINE_REPO" "$OFFLINE_DIR"
fi

# ── 2. 离线安装 Docker（版本过旧或 compose 不兼容也自动更新） ──
NEED_INSTALL=false
if ! command -v docker &>/dev/null; then
    NEED_INSTALL=true
    warn "Docker 未安装，通过离线包安装..."
else
    DOCKER_VER=$(docker --version | grep -oP '\d+\.\d+\.\d+' | head -1)
    COMPOSE_VER=$(docker compose version 2>/dev/null | grep -oP '\d+\.\d+\.\d+' | head -1)
    if [ "$(printf '%s\n' "28.0" "$DOCKER_VER" | sort -V | head -1)" != "28.0" ]; then
        NEED_INSTALL=true
        warn "Docker $DOCKER_VER 版本过旧，升级到离线包版本..."
    elif echo "$COMPOSE_VER" | grep -q '^5\.'; then
        NEED_INSTALL=true
        warn "Compose $COMPOSE_VER (v5) 缺少 buildx 插件，重新安装含 buildx..."
    fi
fi
if [ "$NEED_INSTALL" = true ]; then
    sudo rm -rf "$OFFLINE_DIR" 2>/dev/null || true
    git clone --depth 1 "$OFFLINE_REPO" "$OFFLINE_DIR"
    (cd "$OFFLINE_DIR" && sudo bash install_docker.sh)
fi
info "Docker 已安装: $(docker --version)"

# ── 2.5 配置 Docker 国内镜像 ──
if [ "$ARCH" != "x86_64" ]; then
    if [ ! -f /etc/docker/daemon.json ] || ! grep -q 'registry-mirrors' /etc/docker/daemon.json 2>/dev/null; then
        warn "配置 Docker 国内镜像加速..."
        sudo tee /etc/docker/daemon.json > /dev/null << 'EOF'
{
  "registry-mirrors": [
    "https://docker.1ms.run",
    "https://docker.m.daocloud.io"
  ]
}
EOF
        sudo systemctl restart docker
    fi
    info "Docker 镜像加速已配置"
fi

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

# ── 4. docker socket 权限（sg 提权，无需重新登录）──
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

# ── 5. X11 权限 ──
if [ -n "$DISPLAY" ]; then
    xhost +local:docker &>/dev/null || true
fi

# ── 6. 准备基础镜像（优先离线包，其次镜像源拉取）──
if ! docker image inspect ubuntu:noble &>/dev/null; then
    if [ "$ARCH" = "x86_64" ] && [ -f "$OFFLINE_DIR/ubuntu-noble.tar.gz" ]; then
        info "从离线包加载 ubuntu:noble (x86_64)..."
        docker_cmd "docker load -i $OFFLINE_DIR/ubuntu-noble.tar.gz"
    elif [ "$ARCH" = "aarch64" ] && [ -f "$OFFLINE_DIR/ubuntu-noble-aarch64.tar.gz" ]; then
        info "从离线包加载 ubuntu:noble (aarch64)..."
        docker_cmd "docker load -i $OFFLINE_DIR/ubuntu-noble-aarch64.tar.gz"
    else
        warn "离线包不可用，通过镜像源拉取 ubuntu:noble ($ARCH)..."
        docker_cmd "docker pull ubuntu:noble"
    fi
    info "ubuntu:noble 就绪"
else
    info "ubuntu:noble 已就绪"
fi

# ── 7. 构建新家镜像 ──
# 关键点：DOCKER_BUILDKIT=0 + --pull=false
#   BuildKit 会强制向 registry-1.docker.io 校验 manifest，国内网络必超时；
#   传统 builder + --pull=false 直接使用本地已有的 ubuntu:noble，秒过。
if docker image inspect "$IMAGE_NAME" &>/dev/null; then
    info "新家镜像 $IMAGE_NAME 已存在，跳过构建（如需重建先 docker rmi $IMAGE_NAME）"
else
    echo ""
    echo ">>> 构建新家镜像（安装 ROS 2 Jazzy + ros2_control 等，约需 10-20 分钟）..."
    echo "    基础镜像已就绪，系统包走清华源。后续启动无需重复构建。"
    echo ""
    DOCKER_BUILDKIT=0 docker build --pull=false \
        -f docker/Dockerfile.newros -t "$IMAGE_NAME" .
    info "新家镜像构建完成"
fi

# ── 8. 启动 ──
echo ""
docker_cmd "docker rm -f $CONTAINER 2>/dev/null || true"
docker_cmd "docker compose -f $COMPOSE_FILE up -d"

# ── 9. 等待并验证 ──
sleep 5
RUNNING=$(docker_cmd "docker ps --format '{{.Names}}'" 2>/dev/null)

if echo "$RUNNING" | grep -q "$CONTAINER"; then
    info "新家已启动！"
    echo ""
    echo "  进入容器:  ./start-ros.sh shell"
    echo "  构建 workspace: ./start-ros.sh build"
    echo "  执行命令:  ./start-ros.sh run ros2 pkg list"
    echo "  查看日志:  docker compose -f $COMPOSE_FILE logs -f"
    echo "  停止:      ./start-ros.sh stop"
    echo ""
else
    error "启动失败，查看日志: docker compose -f $COMPOSE_FILE logs"
    docker_cmd "docker compose -f $COMPOSE_FILE logs --tail=30"
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
