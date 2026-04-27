"""
摄像头驱动 - 简单封装 aiortc MediaPlayer
"""

import logging
from aiortc.contrib.media import MediaPlayer

logger = logging.getLogger(__name__)


class CameraDriver:
    """摄像头驱动（简化版）"""
    
    def __init__(self, config):
        self.camera_id = config.get('camera_id', 0)
        self.width = config.get('width', 1920)
        self.height = config.get('height', 1080)
        self.fps = config.get('fps', 30)
        self.player = None
    
    def connect(self) -> bool:
        """打开摄像头"""
        try:
            self.player = MediaPlayer(
                self.camera_id,
                format="v4l2",
                options={
                    "video_size": f"{self.width}x{self.height}",
                    "framerate": str(self.fps)
                }
            )
            logger.info(f"✅ 摄像头已打开: {self.camera_id}")
            return True
        except Exception as e:
            logger.error(f"❌ 打开摄像头失败: {e}")
            return False
    
    def disconnect(self):
        """关闭摄像头"""
        if self.player:
            try:
                self.player.video.stop()
            except Exception:
                pass
            self.player = None
        logger.info("🔌 摄像头已关闭")
    
    def get_player(self):
        """获取 MediaPlayer"""
        return self.player
