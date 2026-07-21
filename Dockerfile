# ============================================================
# Aider Terminal — Docker 镜像
# ============================================================
FROM ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# ── 清华 apt 源 ──
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || \
    sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list

# ── 系统依赖 ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg \
    python3-pip build-essential \
    can-utils usbutils \
    iproute2 libgl1 sudo \
    libportaudio2 alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Jazzy ──
RUN echo "deb [trusted=yes] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# ── pip 清华源 ──
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# ── Python 依赖 ──
RUN pip3 install --break-system-packages --ignore-installed \
    setuptools "numpy<2" requests websockets pyyaml scipy opencv-python \
    pybullet pyserial trimesh "aiortc>=1.7.0" "av>=11.0.0" \
    pin-pink "qpsolvers[quadprog]" meshcat_shapes loop_rate_limiters \
    colcon-common-extensions sounddevice aiohttp

# ── 创建工作空间骨架（源码在运行时 volume 挂载）──
WORKDIR /ws
RUN mkdir -p src

# ── 入口 ──
COPY scripts/docker_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
