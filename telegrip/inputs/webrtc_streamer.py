"""WebRTC 视频推流器 - 使用 aiortc 推送摄像头视频流"""
import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer


class WebRTCStreamer:
    """WebRTC 视频推流器"""
    
    def __init__(self, ws_client, video_source="/dev/video0"):
        self.ws_client = ws_client
        self.video_source = video_source
        self.pc = None
        self.player = None
        self.is_streaming = False

    async def start_streaming(self):
        """开始视频推流"""
        if self.is_streaming:
            print("⚠️ 已经在推流中")
            return
        
        try:
            # 打开摄像头
            self.player = MediaPlayer(self.video_source, format="v4l2", options={
                "video_size": "1920x1080",
                "framerate": "30"
            })
            print(f"✅ 摄像头已打开: {self.video_source}")
            
            # 创建 PeerConnection
            self.pc = RTCPeerConnection()

            # 配置 STUN 服务器
            self.pc.iceServers = [
                # Google STUN(最稳定)
                {'urls': 'stun:stun.l.google.com:19302'},
                {'urls': 'stun:stun1.l.google.com:19302'},
                {'urls': 'stun:stun2.l.google.com:19302'},
#                 国内 STUN
                {'urls': 'stun:stun.miwifi.com:3478'},
                {'urls': 'stun:stun.qq.com:3478'},
                {'urls': 'stun:stun.bige0.com:3391'},
#                 自建 TURN 服务器
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
                # iceCandidatePoolSize: 10
            ]
            self.pc.iceCandidatePoolSize=10
            # 添加视频轨道
            self.pc.addTrack(self.player.video)
            
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
            print("📹 WebRTC 推流已启动")
            
        except Exception as e:
            print(f"❌ 启动推流失败: {e}")
            await self.stop_streaming()
    
    async def handle_answer(self, answer_data):
        """处理 Answer"""
        if not self.pc:
            print("⚠️ PeerConnection 未初始化")
            return
        
        # 检查 PeerConnection 状态，只在 have-local-offer 状态下才能处理 answer
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
        
        # 等待一小段时间确保资源释放
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
        
        if self.player:
            try:
                self.player.video.stop()
            except Exception:
                pass
            self.player = None

        if self.pc:
            try:
                await self.pc.close()
            except Exception:
                pass
            self.pc = None

        print("🛑 WebRTC 推流已停止")
