#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export PYTHONPATH=/ws/src/aiderminal:$PYTHONPATH

ROBOT_TYPE="${ROBOT_TYPE:-aider}"
NO_ROBOT="${NO_ROBOT:-true}"

echo ">>> 启动 terminal_node (robot=$ROBOT_TYPE, no_robot=$NO_ROBOT)"

exec ros2 launch aiderminal terminal.launch.py \
    robot_type:=$ROBOT_TYPE \
    no_robot:=$NO_ROBOT \
    "$@"
