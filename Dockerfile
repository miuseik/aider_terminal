FROM ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 清华 apt 源
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || \
    sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg \
    python3-pip build-essential \
    can-utils usbutils \
    iproute2 libgl1 sudo \
    && rm -rf /var/lib/apt/lists/*

# 添加 ROS 2 源（清华），用 trusted=yes 跳过 key
RUN echo "deb [trusted=yes] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

# 安装 ROS 2 Jazzy 基础包
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# pip 国内镜像
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# 安装 Python 依赖（含 colcon）
RUN pip3 install --break-system-packages --ignore-installed \
    setuptools numpy requests websockets pyyaml scipy opencv-python \
    pybullet pyserial trimesh "aiortc>=1.7.0" "av>=11.0.0" \
    pin-pink "qpsolvers[quadprog]" meshcat_shapes loop_rate_limiters \
    colcon-common-extensions

# 创建 ROS 工作空间
WORKDIR /ws
RUN mkdir -p src/aiderminal

# 复制项目
COPY . src/aiderminal/

# 构建
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select aiderminal

COPY scripts/docker_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
