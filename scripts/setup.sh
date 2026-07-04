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

# ── 1.5 配置 Docker 国内镜像加速 ──
if [ ! -f /etc/docker/daemon.json ] || ! grep -q 'registry-mirrors' /etc/docker/daemon.json 2>/dev/null; then
    warn "配置 Docker 国内镜像加速..."
    sudo mkdir -p /etc/docker
    sudo tee /etc/docker/daemon.json > /dev/null << 'EOF'
{
  "registry-mirrors": [
    "https://mirror.ccs.tencentyun.com",
    "https://docker.mirrors.ustc.edu.cn",
    "https://registry.docker-cn.com"
  ]
}
EOF
    sudo systemctl daemon-reload
    sudo systemctl restart docker
    info "Docker 镜像加速已配置"
else
    info "Docker 镜像加速已配置"
fi

# ── 2. docker compose 插件 ──
if ! docker compose version &>/dev/null; then
    warn "安装 docker compose 插件..."

    # 清理可能损坏的旧插件（避免 segfault）
    sudo rm -f /usr/local/lib/docker/cli-plugins/docker-compose

    # 优先 apt 安装 docker compose v2
    sudo apt-get install -y -qq docker-compose-v2 2>/dev/null || \
        sudo apt-get install -y -qq docker-compose-plugin 2>/dev/null || true

    if ! docker compose version &>/dev/null; then
        # apt 失败 → pip 安装 standalone（走 PyPI 镜像，国内快）
        warn "apt 不可用，通过 pip 安装 docker-compose (v1)..."
        sudo pip3 install docker-compose 2>/dev/null || \
            sudo pip install docker-compose 2>/dev/null || true

        if command -v docker-compose &>/dev/null; then
            # 创建 docker CLI plugin wrapper
            # 这样 docker compose 命令也能透明使用 v1 standalone
            CLI_PLUGIN_DIR="/usr/local/lib/docker/cli-plugins"
            sudo mkdir -p "$CLI_PLUGIN_DIR"
            sudo tee "${CLI_PLUGIN_DIR}/docker-compose" > /dev/null << 'PLUGINEOF'
#!/bin/sh
exec docker-compose "$@"
PLUGINEOF
            sudo chmod +x "${CLI_PLUGIN_DIR}/docker-compose"
        fi
    fi

    # 最后兜底：GitHub 直接下载
    if ! docker compose version &>/dev/null; then
        ARCH=$(uname -m)
        case "$ARCH" in
            aarch64|arm64)   COMPOSE_ARCH="aarch64" ;;
            x86_64|amd64)    COMPOSE_ARCH="x86_64"  ;;
            *)               COMPOSE_ARCH="aarch64" ;;
        esac
        CLI_PLUGIN_DIR="/usr/local/lib/docker/cli-plugins"
        sudo mkdir -p "$CLI_PLUGIN_DIR"

        # 多镜像源尝试
        for URL in \
            "https://mirror.ghproxy.com/https://github.com/docker/compose/releases/latest/download/docker-compose-linux-${COMPOSE_ARCH}" \
            "https://ghfast.top/https://github.com/docker/compose/releases/latest/download/docker-compose-linux-${COMPOSE_ARCH}" \
            "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-${COMPOSE_ARCH}"; do
            warn "尝试下载 (60s超时): $URL"
            if sudo curl -fsSL --connect-timeout 10 --max-time 60 "$URL" -o "${CLI_PLUGIN_DIR}/docker-compose" 2>/dev/null; then
                sudo chmod +x "${CLI_PLUGIN_DIR}/docker-compose"
                break
            fi
            warn "该地址超时或失败，试下一条..."
        done
    fi

    if ! docker compose version &>/dev/null; then
        error "docker compose 插件安装失败，请手动执行:"
        error "  sudo pip3 install docker-compose"
        error "  然后重新运行 ./scripts/setup.sh"
        exit 1
    fi
fi
info "docker compose 可用"

# 检测使用 docker compose (v2) 还是 docker-compose (v1 standalone)
if docker compose version &>/dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
elif command -v docker-compose &>/dev/null; then
    COMPOSE_CMD="docker-compose"
else
    error "找不到 docker compose 命令"
    exit 1
fi
info "compose 命令: $COMPOSE_CMD"

# ── 3. 当前用户加入 docker 组 ──
if ! groups "$USER" | grep -q docker; then
    warn "将 $USER 加入 docker 组..."
    sudo usermod -aG docker "$USER"
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

# ── 6. 构建镜像 ──
echo ""
echo ">>> 构建 Docker 镜像..."
echo "    首次构建会下载 ROS 基础镜像 (~500MB)，请耐心等待。"
echo "    后续启动无需重复构建，秒级完成。"
echo ""

if [ "$NEED_RELOGIN" = true ]; then
    # 刚加入 docker 组，用 sg 获取新组权限
    sg docker -c "$COMPOSE_CMD build"
else
    $COMPOSE_CMD build
fi
info "镜像构建完成"

# ── 7. 启动 ──
echo ""
if [ "$NEED_RELOGIN" = true ]; then
    sg docker -c "$COMPOSE_CMD up -d"
else
    $COMPOSE_CMD up -d
fi

# ── 8. 等待启动 ──
sleep 3
if [ "$NEED_RELOGIN" = true ]; then
    RUNNING=$(sg docker -c "docker ps --format '{{.Names}}'" 2>/dev/null)
else
    RUNNING=$(docker ps --format '{{.Names}}' 2>/dev/null)
fi

if echo "$RUNNING" | grep -q aiderminal; then
    info "项目已启动！"
    echo ""
    echo "  查看日志:  $COMPOSE_CMD logs -f"
    echo "  停止项目:  $COMPOSE_CMD down"
    echo "  改代码后:  $COMPOSE_CMD restart"
    echo ""
else
    error "启动失败，查看日志: $COMPOSE_CMD logs"
    if [ "$NEED_RELOGIN" = true ]; then
        sg docker -c "$COMPOSE_CMD logs --tail=30"
    else
        $COMPOSE_CMD logs --tail=30
    fi
    exit 1
fi

if [ "$NEED_RELOGIN" = true ]; then
    echo ""
    warn "已通过 sg 命令执行 docker 操作，无需重新登录"
fi
