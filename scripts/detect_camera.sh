#!/bin/bash
# ============================================
# 摄像头检测：输出 Docker 设备映射参数
# 用法：source scripts/detect_camera.sh
#       然后 $CAMERA_DEVICE_ARGS 就是映射参数
# ============================================

CAMERA_DEVICE_ARGS=()

for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    name=$(udevadm info --query=all --name="$dev" 2>/dev/null | grep ID_V4L_PRODUCT | head -1)
    if [ -n "$name" ]; then
        echo "📷 检测到摄像头: $dev ($name)"
        CAMERA_DEVICE_ARGS=(-v /dev:/dev:ro --device-cgroup-rule='c 81:* rmw')
        return 0
    fi
done

echo "⚠️  未检测到摄像头"
return 0
