#!/bin/bash
# 新家 (src/) 容器入口：构建 workspace 后执行传入命令
set -e

source /opt/ros/jazzy/setup.bash

# 清理 Python 字节码缓存，避免旧 .pyc 导致代码不生效
find /ws/src -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
find /ws/src -name '*.pyc' -type f -delete 2>/dev/null || true

# 构建整个 workspace（源码已 volume 挂载，--symlink-install 增量秒级）
echo ">>> colcon build (workspace: /ws/src) ..."
cd /ws
# PKGS: 可选，指定只构建部分包（空格分隔），不设则全部构建
if [ -n "$PKGS" ]; then
    # shellcheck disable=SC2086
    colcon build --symlink-install --packages-select $PKGS 2>&1 | tail -20
else
    colcon build --symlink-install 2>&1 | tail -20
fi

source /ws/install/setup.bash

echo ">>> workspace 就绪"
echo ">>> 可用命令: ros2 pkg list | ros2 run <pkg> <node> | ros2 launch <pkg> <launch>"

# 执行传入命令，默认进入 bash
exec "$@"
