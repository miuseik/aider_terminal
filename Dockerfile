# ============================================================
# Aider Terminal — Docker 镜像
# ============================================================
# 国内网络下 docker.io 直连超时，基础镜像走 DaoCloud 加速源
FROM docker.m.daocloud.io/library/ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# ── 清华 apt 源 ──
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || \
    sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list

# ── 系统依赖 ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg \
    python3-pip python3-dev build-essential \
    can-utils usbutils \
    iproute2 libgl1 sudo \
    libglib2.0-0 libsm6 libxext6 libxrender1 \
    libportaudio2 alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# ── pip 清华源 ──
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# ── Python 依赖（核心，必须装成功）──
RUN pip3 install --break-system-packages --ignore-installed \
    setuptools "numpy<2" requests websockets pyyaml scipy opencv-python \
    pyserial trimesh fast-simplification "aiortc>=1.7.0" "av>=11.0.0" \
    pin-pink "qpsolvers[quadprog]" meshcat_shapes loop_rate_limiters \
    sounddevice

# ── pybullet（仅本地仿真可视化用，真机模式不依赖）──
# 新版 Ubuntu noble (Python 3.12 + gcc 14) 下从 sdist 编译 Bullet 会把
# -Wmaybe-uninitialized 等 warning 当 error 导致失败；用 CFLAGS="-Wno-error"
# 降级为 warning 后通常可编译通过。失败也不阻断后续步骤。
RUN CFLAGS="-Wno-error" CXXFLAGS="-Wno-error" \
    pip3 install --break-system-packages --ignore-installed pybullet || \
    echo "[warn] pybullet 编译失败，跳过（仅影响本地仿真可视化）"

# ── 工作目录（源码在运行时 volume 挂载）──
WORKDIR /app

# ── 入口 ──
COPY scripts/docker_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
