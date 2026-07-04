#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# ── Xvfb 虚拟显示（pybullet GUI 需要合法的 X server）──
if command -v Xvfb &>/dev/null; then
    # 如果外部已设置 DISPLAY 且可用，跳过
    if [ -z "$DISPLAY" ] || ! xdpyinfo &>/dev/null 2>&1; then
        # 找空闲 display 号
        DISP_NUM=99
        while [ -e "/tmp/.X11-unix/X${DISP_NUM}" ]; do
            DISP_NUM=$((DISP_NUM + 1))
        done
        Xvfb ":${DISP_NUM}" -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
        XVFB_PID=$!
        export DISPLAY=":${DISP_NUM}"
        echo "🖥️  Xvfb 已启动 (DISPLAY=$DISPLAY, PID=$XVFB_PID)"
    fi
fi

# 清理 Python bytecode 缓存，避免旧 .pyc 导致代码不生效
find /ws/src -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
find /ws/src -name '*.pyc' -type f -delete 2>/dev/null || true
rm -f /tmp/pybullet_diag.log 2>/dev/null || true

# 源码通过 volume 挂载，每次启动自动 colcon build（--symlink 秒级完成）
echo ">>> colcon build..."
cd /ws && colcon build --symlink-install --packages-select aiderminal 2>&1 | tail -5

source /ws/install/setup.bash
export PYTHONPATH=/ws/src/aiderminal:$PYTHONPATH

ROBOT_TYPE="${ROBOT_TYPE:-aider}"
NO_ROBOT="${NO_ROBOT:-true}"

echo ">>> 启动 terminal_node (robot=$ROBOT_TYPE, no_robot=$NO_ROBOT)"
exec ros2 launch aiderminal terminal.launch.py \
    robot_type:=$ROBOT_TYPE \
    no_robot:=$NO_ROBOT \
    "$@"
