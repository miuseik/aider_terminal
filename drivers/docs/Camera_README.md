# 相机驱动

## 概述

**OpenCVCameraDriver**: 基于 OpenCV 的摄像头驱动，支持 WebRTC 推流

## 依赖

```bash
# OpenCV 摄像头驱动
pip install opencv-python

# WebRTC 支持
pip install aiortc av
```

## 快速开始

### OpenCVCameraDriver（推荐）

```python
from drivers.camera import OpenCVCameraDriver

# 配置摄像头
config = {
    'index_or_path': '/dev/video0',  # 设备路径
    'width': 1280,                   # 分辨率宽度
    'height': 480,                   # 分辨率高度
    'fps': 30,                       # 帧率
    'fourcc': 'MJPG',                # 编码格式
    'color_mode': 'bgr',             # 颜色模式
    'rotation': 0,                   # 旋转角度
    'warmup_s': 0.5                  # 预热时间
}

# 创建并连接
camera = OpenCVCameraDriver(config)
camera.connect()

# 读取帧
frame = camera.read()        # 同步读取（阻塞）
frame = camera.read_latest() # 异步读取（非阻塞）

# 断开连接
camera.disconnect()
```

### WebRTC 集成

```python
from telegrip.inputs.webrtc_streamer import WebRTCStreamer

# WebRTC 推流器会自动使用配置的摄像头
streamer = WebRTCStreamer(ws_client, config)
await streamer.start_streaming()
```

### 多摄像头支持

```python
from drivers.camera import OpenCVCameraDriver

cameras = []

# 打开多个摄像头
for cam_id in [0, 1]:
    config = {
        'index_or_path': cam_id,
        'width': 640,
        'height': 480,
        'fps': 30
    }
    
    camera = OpenCVCameraDriver(config)
    if camera.connect():
        cameras.append(camera)

# 使用后关闭
for camera in cameras:
    camera.disconnect()
```

## API 参考

### OpenCVCameraDriver

#### 初始化

```python
OpenCVCameraDriver(config: dict)
```

**配置参数**：
- `index_or_path` (int/str): 摄像头索引或设备路径，默认 0
  - Linux: `/dev/video0`, `/dev/video1`...
  - Windows: 0, 1, 2...
- `width` (int): 分辨率宽度，默认 640
- `height` (int): 分辨率高度，默认 480
- `fps` (int): 帧率，默认 30
- `fourcc` (str): 编码格式，如 "MJPG", "YUYV"（可选）
- `color_mode` (str): "rgb" 或 "bgr"，默认 "rgb"
- `rotation` (int): 旋转角度 0/90/180/270，默认 0
- `warmup_s` (float): 预热时间秒数，默认 0.5

#### 方法

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `connect()` | 打开摄像头 | bool - 是否成功 |
| `disconnect()` | 关闭摄像头 | None |
| `read()` | 同步读取一帧（阻塞） | np.ndarray 或 None |
| `read_latest()` | 异步读取最新帧（非阻塞） | np.ndarray 或 None |
| `get_info()` | 获取相机信息 | dict |

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

此驱动已集成到 WebRTC 推流器中，自动使用：

```python
from telegrip.inputs.webrtc_streamer import WebRTCStreamer

# WebRTC 推流器会自动使用配置的摄像头
streamer = WebRTCStreamer(ws_client, config)
await streamer.start_streaming()
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
2. **分辨率配置**: 确保摄像头支持请求的分辨率，否则会自动降级
3. **性能**: 高分辨率+高帧率会消耗较多 CPU 和带宽
4. **WebRTC**: 推流时自动使用 `read_latest()` 非阻塞读取，保证流畅性
