"""
RTC摄像头驱动 - 基于WebRTC的视频推流

职责:
1. 管理摄像头硬件(打开/关闭)
2. 管理RTC推流(启动/停止/信令交换)
3. 提供视频帧读取接口
"""

import logging
import asyncio
from typing import Optional
import numpy as np
from .camera_driver import CameraDriver

# 延迟导入aiortc,避免启动时加载慢
# from aiortc import RTCPeerConnection, RTCSessionDescription
# from aiortc.contrib.media import MediaPlayer

logger = logging.getLogger(__name__)


class RTCCameraDriver(CameraDriver):
    """RTC摄像头驱动"""
    
    def __init__(self, config):
        """
        初始化RTC摄像头驱动
        
        Args:
            config: 配置信息
                - camera_id: 摄像头设备 (如 "/dev/video0")
                - width: 分辨率宽度
                - height: 分辨率高度
                - fps: 帧率
                - ws_client: WebSocket客户端(用于信令传输)
        """
        super().__init__(config)
        self.ws_client = config.get('ws_client')
        
        # RTC连接
        self.pc = None
        self.player = None
        self.is_streaming = False
    
    def connect(self) -> bool:
        """打开摄像头"""
        try:
            # 延迟导入aiortc
            from aiortc.contrib.media import MediaPlayer
            
            # 创建MediaPlayer
            self.player = MediaPlayer(
                self.camera_id,
                format="v4l2",
                options={
                    "video_size": f"{self.width}x{self.height}",
                    "framerate": str(self.fps)
                }
            )
            
            logger.info(f"✅ 摄像头已打开: {self.camera_id}")
            self.is_connected = True
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
        
        self.is_connected = False
        logger.info("🔌 摄像头已关闭")
    
    def is_ready(self) -> bool:
        """检查摄像头是否就绪"""
        return self.is_connected and self.player is not None
    
    def read_frame(self) -> Optional[np.ndarray]:
        """
        读取一帧图像
        
        Returns:
            np.ndarray: BGR格式图像, 失败返回None
        
        Note: WebRTC模式下,视频由aiortc自动推送,此方法主要用于兼容
        """
        if not self.player:
            return None
        
        try:
            # TODO: 从MediaPlayer读取帧
            # 注意: WebRTC模式下通常不需要手动读帧
            logger.warning("⚠️ WebRTC模式下不建议调用read_frame()")
            return None
        except Exception as e:
            logger.error(f"❌ 读取帧失败: {e}")
            return None
    
    async def start_streaming(self) -> bool:
        """
        启动WebRTC推流
        
        Returns:
            bool: 是否成功
        """
        if self.is_streaming:
            logger.warning("⚠️ 已经在推流中")
            return True
        
        if not self.ws_client:
            logger.error("❌ WebSocket客户端未提供")
            return False
        
        try:
            # 延迟导入aiortc
            from aiortc import RTCPeerConnection
            
            # 确保摄像头已打开
            if not self.is_connected:
                self.connect()
            
            # 创建PeerConnection
            self.pc = RTCPeerConnection()
            
            # 配置STUN/TURN服务器
            self.pc.iceServers = [
                # Google STUN
                {'urls': 'stun:stun.l.google.com:19302'},
                {'urls': 'stun:stun1.l.google.com:19302'},
                
                # 国内STUN
                {'urls': 'stun:stun.miwifi.com:3478'},
                {'urls': 'stun:stun.qq.com:3478'},
                
                # TURN服务器(如果有)
                # {
                #     'urls': 'turn:your-turn-server.com:3478',
                #     'username': 'user',
                #     'credential': 'pass'
                # }
            ]
            self.pc.iceCandidatePoolSize = 10
            
            # 添加视频轨道
            self.pc.addTrack(self.player.video)
            
            # 处理ICE候选
            @self.pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    await self._send_ice_candidate(candidate)
            
            # 创建Offer
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            
            # 通过WebSocket发送Offer
            await self.ws_client.send_message({
                "type": "offer",
                "sdp": self.pc.localDescription.sdp
            })
            
            self.is_streaming = True
            logger.info("📹 WebRTC推流已启动")
            return True
            
        except Exception as e:
            logger.error(f"❌ 启动推流失败: {e}")
            await self.stop_streaming()
            return False
    
    async def handle_answer(self, answer_data: dict) -> bool:
        """
        处理Answer信令
        
        Args:
            answer_data: {"sdp": "...", "type": "answer"}
            
        Returns:
            bool: 是否成功
        """
        if not self.pc:
            logger.warning("⚠️ PeerConnection未初始化")
            return False
        
        # 检查状态
        if self.pc.signalingState != "have-local-offer":
            logger.warning(f"⚠️ 跳过answer(当前状态: {self.pc.signalingState})")
            return False
        
        try:
            from aiortc import RTCSessionDescription
            
            answer = RTCSessionDescription(
                sdp=answer_data["sdp"],
                type="answer"
            )
            await self.pc.setRemoteDescription(answer)
            logger.info("✅ WebRTC连接已建立")
            return True
        except Exception as e:
            logger.error(f"❌ 处理Answer失败: {e}")
            return False
    
    async def stop_streaming(self):
        """停止WebRTC推流"""
        self.is_streaming = False
        
        if self.pc:
            try:
                await self.pc.close()
            except Exception:
                pass
            self.pc = None
        
        logger.info("🛑 WebRTC推流已停止")
    
    async def _send_ice_candidate(self, candidate):
        """发送ICE候选"""
        if not self.ws_client:
            return
        
        try:
            await self.ws_client.send_message({
                "type": "candidate",
                "candidate": {
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex
                }
            })
        except Exception as e:
            logger.error(f"❌ 发送ICE候选失败: {e}")
    
    async def restart_streaming(self) -> bool:
        """重新启动推流(UI刷新后调用)"""
        logger.info("🔄 重新启动WebRTC推流...")
        
        await self.stop_streaming()
        await asyncio.sleep(0.5)
        
        return await self.start_streaming()
