"""
视频控制器
管理阿里云 ARTC 视频推流的启动和停止
"""
import time
import threading
from typing import Optional


class VideoController:
    """
    视频推流控制器
    
    负责按需启动和停止 ARTC 视频推流，节省云服务费用
    """
    
    def __init__(self):
        """初始化视频控制器"""
        self._is_running = False
        self._stream_thread: Optional[threading.Thread] = None
        print("📹 视频控制器初始化完成")
    
    @property
    def is_running(self) -> bool:
        """检查视频推流是否正在运行"""
        return self._is_running
    
    def handle_command(self, action: str):
        """
        处理视频控制命令
        
        Args:
            action: 操作类型 ('start' 或 'stop')
        """
        if action == 'start':
            self.start_video_stream()
        elif action == 'stop':
            self.stop_video_stream()
        else:
            print(f"⚠️ 未知的视频操作: {action}")
    
    def start_video_stream(self):
        """
        启动视频推流（按需）
        
        如果推流已在运行，先停止再重新启动
        """
        import src.rtc_video as rtc_video
        
        # 如果正在运行，先停止
        if rtc_video.isRunning:
            print("📹 推流线程正在运行，先停止...")
            rtc_video.stopSignal = True
            time.sleep(1)  # 等待线程退出
            
            # 检查是否真的停止了
            for i in range(5):
                if not rtc_video.isRunning:
                    break
                print(f"📹 等待推流线程退出... ({i+1}/5)")
                time.sleep(0.5)
        
        # 重置状态
        rtc_video.stopSignal = False
        
        print("📹 收到前端请求，启动视频推流...")
        thread = threading.Thread(target=rtc_video.main, daemon=True, name="AliRTCStreamer")
        thread.start()
        
        self._is_running = True
        self._stream_thread = thread
        print("✅ 视频推流已启动")
    
    def stop_video_stream(self):
        """
        停止视频推流（按需）
        
        设置停止信号，让推流线程优雅退出
        """
        import src.rtc_video as rtc_video
        
        if rtc_video.isRunning:
            print("📹 收到前端请求，停止视频推流...")
            rtc_video.stopSignal = True
            self._is_running = False
            print("✅ 视频推流停止信号已发送")
        else:
            print("ℹ️ 视频推流未运行，无需停止")
    
    def cleanup(self):
        """
        清理资源
        
        在程序退出时调用，确保推流线程正确停止
        """
        if self._is_running:
            print("📹 清理视频推流资源...")
            self.stop_video_stream()
