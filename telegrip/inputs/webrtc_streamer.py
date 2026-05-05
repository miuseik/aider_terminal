"""WebRTC 视频推流器 - 使用高画质摄像头 + aiortc"""
import asyncio
import json
from fractions import Fraction
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import MediaStreamTrack
from av import VideoFrame
import numpy as np
from drivers.camera import OpenCVCameraDriver


class OpenCVVideoTrack(MediaStreamTrack):
    """基于 OpenCV 摄像头的 WebRTC 视频轨道"""
    
    kind = "video"
    
    def __init__(self, camera_driver, fps=30):
        super().__init__()
        self.camera = camera_driver
        self._timestamp = 0
        self._time_base = Fraction(1, fps)  # 使用 Fraction 而不是 float
    
    async def recv(self):
        """异步读取帧并转换为 WebRTC 格式"""
        pts = int(self._timestamp / self._time_base)
        
        # 读取帧
        frame = self.camera.read_latest()
        
        if frame is None:
            # 如果读取失败，等待一下再试
            await asyncio.sleep(0.01)
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 转换 BGR -> RGB
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame_rgb = frame[:, :, ::-1]  # BGR to RGB
        else:
            frame_rgb = frame
        
        # 创建 VideoFrame
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = self._time_base
        
        self._timestamp += self._time_base
        
        return video_frame


class WebRTCStreamer:
    """WebRTC 视频推流器（高画质版）"""
    
    def __init__(self, ws_client, config=None):
        self.ws_client = ws_client
        self.config = config
        self.pc = None
        self.is_streaming = False
        self.camera = None
        self.video_track = None
        
        # 从配置读取摄像头参数
        if config and hasattr(config, 'video_source'):
            video_source = config.video_source
        else:
            video_source = '/dev/video0'
        
        # 初始化高画质摄像头驱动
        self.camera_config = {
            'index_or_path': video_source,
            'width': getattr(config, 'camera_width', 1280) if config else 1280,
            'height': getattr(config, 'camera_height', 480) if config else 480,
            'fps': getattr(config, 'camera_fps', 30) if config else 30,
            'fourcc': getattr(config, 'camera_fourcc', 'MJPG') if config else 'MJPG',
            'color_mode': 'bgr',
            'rotation': 0,
            'warmup_s': 0.5
        }

    async def start_streaming(self):
        """开始视频推流"""
        if self.is_streaming:
            print("⚠️ 已经在推流中")
            return
        
        try:
            # 初始化摄像头
            self.camera = OpenCVCameraDriver(self.camera_config)
            
            if not self.camera.connect():
                print(f"❌ 无法打开摄像头: {self.camera_config['index_or_path']}")
                return
            
            print(f"✅ 摄像头已连接: {self.camera_config['width']}x{self.camera_config['height']}@{self.camera_config['fps']}fps")
            
            # 创建视频轨道
            fps = self.camera_config.get('fps', 30)
            self.video_track = OpenCVVideoTrack(self.camera, fps=fps)
            
            # 创建 PeerConnection
            self.pc = RTCPeerConnection()

            # 配置 STUN/TURN 服务器
            self.pc.iceServers = [
                # Google STUN
                {'urls': 'stun:stun.l.google.com:19302'},
                {'urls': 'stun:stun1.l.google.com:19302'},
                {'urls': 'stun:stun2.l.google.com:19302'},
                # 国内 STUN
                {'urls': 'stun:stun.miwifi.com:3478'},
                {'urls': 'stun:stun.qq.com:3478'},
                {'urls': 'stun:stun.bige0.com:3391'},
                # 自建 TURN 服务器
                {
                    'urls': 'turn:ws.houqicg.com:3478',
                    'username': 'aider',
                    'credential': 'aider123456'
                },
                {
                    'urls': 'turns:ws.houqicg.com:5349',
                    'username': 'aider',
                    'credential': 'aider123456'
                },
            ]
            self.pc.iceCandidatePoolSize = 10
            
            # 添加视频轨道
            self.pc.addTrack(self.video_track)
            
            # 处理 ICE 候选
            @self.pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    await self._send_ice_candidate(candidate)
            
            # 创建 Offer
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)

            # 通过 WebSocket 发送 Offer
            await self.ws_client.send_message({
                "type": "offer",
                "sdp": self.pc.localDescription.sdp
            })
            
            self.is_streaming = True
            print("📹 WebRTC 推流已启动（高画质模式）")
            
        except Exception as e:
            print(f"❌ 启动推流失败: {e}")
            import traceback
            traceback.print_exc()
            await self.stop_streaming()
    
    async def handle_answer(self, answer_data):
        """处理 Answer"""
        if not self.pc:
            print("⚠️ PeerConnection 未初始化")
            return
        
        # 检查 PeerConnection 状态
        if self.pc.signalingState != "have-local-offer":
            print(f"⚠️ 跳过 answer（当前状态: {self.pc.signalingState}）")
            return
        
        try:
            answer = RTCSessionDescription(sdp=answer_data["sdp"], type="answer")
            await self.pc.setRemoteDescription(answer)
            print("✅ WebRTC 连接已建立")
        except Exception as e:
            print(f"❌ 处理 Answer 失败: {e}")
    
    async def restart_streaming(self):
        """重新启动推流(UI 刷新后调用)"""
        print("🔄 重新启动 WebRTC 推流...")

        # 关闭旧的连接
        await self.stop_streaming()
        
        # 等待资源释放
        await asyncio.sleep(0.5)
        
        # 重新创建推流
        await self.start_streaming()
    
    async def _send_ice_candidate(self, candidate):
        """发送 ICE 候选"""
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
            print(f"❌ 发送 ICE 候选失败: {e}")
    
    async def stop_streaming(self):
        """停止视频推流"""
        self.is_streaming = False
        
        # 关闭摄像头
        if self.camera:
            self.camera.disconnect()
            self.camera = None

        # 关闭 PeerConnection
        if self.pc:
            try:
                await self.pc.close()
            except Exception:
                pass
            self.pc = None
        
        self.video_track = None

        print("🛑 WebRTC 推流已停止")
