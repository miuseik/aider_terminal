"""
相机驱动使用示例
演示如何打开和使用摄像头
"""

import sys
import time


def check_dependencies():
    """检查依赖是否安装"""
    try:
        from aiortc.contrib.media import MediaPlayer
        return True
    except ImportError:
        print("❌ 缺少依赖: aiortc")
        print("\n请安装:")
        print("  pip install aiortc")
        return False


def example_basic_camera():
    """示例1: 基本摄像头操作"""
    print("\n=== 示例1: 基本摄像头操作 ===")
    
    from camera_driver import CameraDriver
    
    # 配置摄像头
    config = {
        'camera_id': 0,      # 摄像头设备号 (Linux: /dev/video0, Windows: 0)
        'width': 1920,       # 分辨率宽度
        'height': 1080,      # 分辨率高度
        'fps': 30            # 帧率
    }
    
    # 创建驱动
    camera = CameraDriver(config)
    
    # 连接摄像头
    if not camera.connect():
        print("连接失败")
        return
    
    print("✅ 摄像头已打开")
    print(f"   设备: {config['camera_id']}")
    print(f"   分辨率: {config['width']}x{config['height']}")
    print(f"   帧率: {config['fps']} fps")
    
    # 获取 MediaPlayer（用于 WebRTC 推流）
    player = camera.get_player()
    if player:
        print("✅ MediaPlayer 已就绪")
    
    # 保持运行一段时间
    print("\n摄像头运行中... (按 Ctrl+C 停止)")
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print("\n用户中断")
    
    # 断开连接
    camera.disconnect()
    print("✅ 摄像头已关闭")


def example_multiple_cameras():
    """示例2: 多摄像头支持"""
    print("\n=== 示例2: 多摄像头支持 ===")
    
    from camera_driver import CameraDriver
    
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
            print(f"✅ 摄像头 {cam_id} 已打开")
            cameras.append(camera)
        else:
            print(f"❌ 摄像头 {cam_id} 打开失败")
    
    print(f"\n共打开 {len(cameras)} 个摄像头")
    
    # 保持运行
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        pass
    
    # 关闭所有摄像头
    for camera in cameras:
        camera.disconnect()
    
    print("✅ 所有摄像头已关闭")


def example_custom_resolution():
    """示例3: 自定义分辨率"""
    print("\n=== 示例3: 自定义分辨率 ===")
    
    from camera_driver import CameraDriver
    
    # 不同的分辨率配置
    resolutions = [
        {'width': 1920, 'height': 1080, 'fps': 30},  # 1080p
        {'width': 1280, 'height': 720, 'fps': 60},   # 720p 高帧率
        {'width': 640, 'height': 480, 'fps': 30},    # VGA
    ]
    
    for i, res in enumerate(resolutions):
        print(f"\n测试分辨率 {i+1}: {res['width']}x{res['height']} @ {res['fps']}fps")
        
        config = {
            'camera_id': 0,
            'width': res['width'],
            'height': res['height'],
            'fps': res['fps']
        }
        
        camera = CameraDriver(config)
        if camera.connect():
            print(f"✅ 成功")
            camera.disconnect()
            time.sleep(0.5)
        else:
            print(f"❌ 失败（可能不支持此分辨率）")


if __name__ == "__main__":
    print("=" * 60)
    print("相机驱动示例")
    print("=" * 60)
    
    # 检查依赖
    if not check_dependencies():
        sys.exit(1)
    
    # 根据需要注释/取消注释要运行的示例
    example_basic_camera()
    # example_multiple_cameras()
    # example_custom_resolution()
    
    print("\n" + "=" * 60)
    print("所有示例执行完毕")
    print("=" * 60)
