# 相机驱动

## 概述

基于 `aiortc` 的 MediaPlayer 封装，用于摄像头采集和 WebRTC 推流。

## 依赖

```bash
pip install aiortc
```

## 快速开始

### 基本使用

```python
from drivers.camera import CameraDriver

# 配置摄像头
config = {
    'camera_id': 0,      # 摄像头设备号
    'width': 1920,       # 分辨率宽度
    'height': 1080,      # 分辨率高度
    'fps': 30            # 帧率
}

# 创建并连接
camera = CameraDriver(config)
camera.connect()

# 获取 MediaPlayer（用于 WebRTC）
player = camera.get_player()

# 断开连接
camera.disconnect()
```

### 多摄像头支持

```python
from drivers.camera import CameraDriver

cameras = []

# 打开多个摄像头
for cam_id in [0, 1]:
    config = {
        'camera_id': cam_id,
        'width': 640,
        'height': 480,
        'fps': 30
    }
    
    camera = CameraDriver(config)
    if camera.connect():
        cameras.append(camera)

# 使用后关闭
for camera in cameras:
    camera.disconnect()
```

## API 参考

### CameraDriver

#### 初始化

```python
CameraDriver(config: dict)
```

**配置参数**：
- `camera_id` (int): 摄像头设备号，默认 0
  - Linux: `/dev/video0`, `/dev/video1`...
  - Windows: 0, 1, 2...
- `width` (int): 分辨率宽度，默认 1920
- `height` (int): 分辨率高度，默认 1080
- `fps` (int): 帧率，默认 30

#### 方法

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `connect()` | 打开摄像头 | bool - 是否成功 |
| `disconnect()` | 关闭摄像头 | None |
| `get_player()` | 获取 MediaPlayer 对象 | MediaPlayer 或 None |

## 平台支持

| 平台 | 支持 | 说明 |
|------|------|------|
| **Linux** | ✅ | 使用 V4L2 (`/dev/video*`) |
| **Windows** | ⚠️ | 可能需要额外配置 |
| **macOS** | ⚠️ | 可能需要额外配置 |

## 常见分辨率

| 名称 | 分辨率 | 帧率 | 用途 |
|------|--------|------|------|
| **VGA** | 640x480 | 30 fps | 低带宽场景 |
| **720p** | 1280x720 | 30/60 fps | 标准高清 |
| **1080p** | 1920x1080 | 30 fps | 全高清 |
| **4K** | 3840x2160 | 30 fps | 超高清（需要高性能摄像头） |

## 运行示例

```bash
cd drivers/camera
python example.py
```

## 与 WebRTC 集成

此驱动设计用于与 WebRTC 配合使用：

```python
from aiortc import RTCPeerConnection
from drivers.camera import CameraDriver

# 创建摄像头
camera = CameraDriver({'camera_id': 0})
camera.connect()

# 获取 MediaPlayer
player = camera.get_player()

# 添加到 WebRTC 连接
pc = RTCPeerConnection()
pc.addTrack(player.video)

# ... 其他 WebRTC 逻辑
```

## 故障排除

### Q1: 无法打开摄像头

**A**: 检查以下几点：
1. 摄像头是否被其他程序占用
2. 是否有权限访问摄像头设备
3. 设备号是否正确

```bash
# Linux: 查看可用摄像头
ls /dev/video*

# 测试摄像头
ffplay /dev/video0
```

### Q2: 分辨率不支持

**A**: 尝试降低分辨率或使用摄像头支持的分辨率：

```python
# 尝试常见分辨率
configs = [
    {'width': 640, 'height': 480},
    {'width': 1280, 'height': 720},
    {'width': 1920, 'height': 1080},
]
```

### Q3: 帧率过低

**A**: 
1. 降低分辨率
2. 检查 USB 带宽（使用 USB 3.0 端口）
3. 减少同时运行的摄像头数量

## 注意事项

1. **资源管理**: 使用完毕后务必调用 `disconnect()` 释放资源
2. **多线程**: MediaPlayer 内部有异步处理，注意线程安全
3. **性能**: 高分辨率+高帧率会消耗较多 CPU 和带宽
4. **WebRTC**: 此驱动主要用于 WebRTC 推流，如需本地预览需额外处理
