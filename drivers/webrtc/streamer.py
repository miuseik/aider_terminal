"""
WebRTC 视频推流器

通过 aiortc 将摄像头画面推送到 aider_server 的 /ws/signaling 信令端点。
作为 asyncio Task 运行，由 TelegripSystem 管理生命周期。
"""

import asyncio
import fractions
import json
import logging
import ssl
import time
from typing import Optional

import numpy as np
from aiortc import (
    RTCConfiguration,
    RTCIceCandidate,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from av import VideoFrame
from websockets.asyncio.client import connect

from config.settings import TelegripConfig
from drivers.camera.opencv_camera_driver import OpenCVCameraDriver

logger = logging.getLogger("webrtc.streamer")


class CameraVideoTrack(VideoStreamTrack):
    """aiortc VideoStreamTrack: 从 OpenCVCameraDriver 读取帧"""

    kind = "video"

    def __init__(self, camera: OpenCVCameraDriver, fps: int):
        super().__init__()
        self._camera = camera
        self._fps = fps
        self._start_time: Optional[float] = None
        self._frame_count = 0
        self._width = camera.width
        self._height = camera.height

        logger.info("CameraVideoTrack: %dx%d @ %dfps", self._width, self._height, self._fps)

    async def next_timestamp(self):
        if self._start_time is None:
            self._start_time = time.time()
        elapsed = time.time() - self._start_time
        pts = int(elapsed * self._fps)
        return pts, fractions.Fraction(1, self._fps)

    async def recv(self) -> VideoFrame:
        loop = asyncio.get_event_loop()
        frame: Optional[np.ndarray] = await loop.run_in_executor(None, self._camera.read)

        if frame is None:
            logger.warning("读帧失败，返回黑帧")
            frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)

        self._frame_count += 1
        if self._frame_count % (self._fps * 5) == 0:
            logger.debug("已采集 %d 帧", self._frame_count)

        pts, time_base = await self.next_timestamp()
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        self._camera.disconnect()
        logger.info("CameraVideoTrack 已释放 (%d 帧)", self._frame_count)


class WebRTCStreamer:
    """WebRTC 推流器 - 连接 aider_server 信令并推送摄像头画面"""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self._ws = None
        self._pc: Optional[RTCPeerConnection] = None
        self._video_track: Optional[CameraVideoTrack] = None
        self._camera: Optional[OpenCVCameraDriver] = None
        self._ice_servers: list = []
        self._running = False

    # ── 信令连接 ──
    async def _connect_signaling(self):
        url = self.config.webrtc_signaling_url
        logger.info("信令: %s", url)

        kwargs = {"ping_interval": 30, "ping_timeout": 10, "max_size": 2 ** 20}
        if url.startswith("wss://"):
            ssl_ctx = ssl.create_default_context()
            ssl_ctx.check_hostname = False
            ssl_ctx.verify_mode = ssl.CERT_NONE
            kwargs["ssl"] = ssl_ctx

        self._ws = await connect(url, **kwargs)
        logger.info("信令已连接")

    async def _join_room(self):
        await self._ws.send(json.dumps({
            "type": "join", "role": "pub", "room_id": self.config.webrtc_room_id,
        }))
        resp = json.loads(await self._ws.recv())
        if resp.get("type") != "joined":
            raise RuntimeError(f"加入房间失败: {resp}")
        self._ice_servers = resp.get("ice_servers", [])
        logger.info("已加入房间 %s, ICE ×%d", self.config.webrtc_room_id, len(self._ice_servers))

    # ── PeerConnection ──
    def _create_peer_connection(self):
        ice_cfg = RTCConfiguration(iceServers=[
            RTCIceServer(urls=s["urls"], username=s.get("username"), credential=s.get("credential"))
            for s in self._ice_servers
        ])
        self._pc = RTCPeerConnection(configuration=ice_cfg)

        self._video_track = CameraVideoTrack(self._camera, self.config.camera_fps)
        self._pc.addTrack(self._video_track)
        logger.info("PeerConnection 已创建")

        @self._pc.on("connectionstatechange")
        async def _on_conn():
            logger.info("连接状态: %s", self._pc.connectionState)

        @self._pc.on("iceconnectionstatechange")
        async def _on_ice():
            logger.info("ICE: %s", self._pc.iceConnectionState)

        @self._pc.on("icecandidate")
        async def _on_ice_candidate(candidate):
            if not candidate:
                return
            await self._ws.send(json.dumps({
                "type": "ice_candidate",
                "candidate": {
                    "foundation": candidate.foundation,
                    "component": candidate.component,
                    "ip": candidate.ip,
                    "port": candidate.port,
                    "priority": candidate.priority,
                    "protocol": candidate.protocol,
                    "type": candidate.type,
                },
                "sdpMid": candidate.sdpMid,
                "sdpMLineIndex": candidate.sdpMLineIndex,
            }))

    async def _send_offer(self):
        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        await self._ws.send(json.dumps({
            "type": "offer",
            "sdp": self._pc.localDescription.sdp,
            "room_id": self.config.webrtc_room_id,
        }))
        logger.info("SDP Offer 已发送")

    # ── 消息处理 ──
    async def _handle_answer(self, msg: dict):
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=msg["sdp"], type="answer"))
        logger.info("远程 SDP 已设置")

    async def _handle_ice(self, msg: dict):
        c = msg.get("candidate")
        if not c:
            return
        await self._pc.addIceCandidate(RTCIceCandidate(
            component=c.get("component", 1),
            foundation=c.get("foundation", ""), ip=c.get("ip", ""),
            port=c.get("port", 0), priority=c.get("priority", 0),
            protocol=c.get("protocol", "udp"), type=c.get("type", "host"),
            sdpMid=msg.get("sdpMid"), sdpMLineIndex=msg.get("sdpMLineIndex"),
        ))

    async def _handle_subscriber_joined(self, msg: dict):
        logger.info("订阅者加入 (×%d), 重发 Offer", msg.get("count", 0))
        if self._pc and self._pc.localDescription:
            await self._ws.send(json.dumps({
                "type": "offer",
                "sdp": self._pc.localDescription.sdp,
                "room_id": self.config.webrtc_room_id,
            }))

    async def _signaling_loop(self):
        """处理信令消息直到连接关闭"""
        handlers = {
            "answer": self._handle_answer,
            "ice_candidate": self._handle_ice,
            "subscriber_joined": self._handle_subscriber_joined,
        }
        async for raw in self._ws:
            if not self._running:
                break
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            handler = handlers.get(msg.get("type"))
            if handler:
                await handler(msg)

    # ── 声明周期 ──
    async def run(self):
        """主入口: 作为 asyncio Task 运行"""
        self._running = True
        try:
            # 摄像头
            camera_cfg = {
                "index_or_path": self.config.video_source,
                "width": self.config.camera_width,
                "height": self.config.camera_height,
                "fps": self.config.camera_fps,
                "fourcc": self.config.camera_fourcc,
                "color_mode": "bgr",
            }
            self._camera = OpenCVCameraDriver(camera_cfg)
            if not self._camera.connect():
                raise RuntimeError("摄像头连接失败")

            # 信令
            await self._connect_signaling()
            await self._join_room()

            # WebRTC
            self._create_peer_connection()
            await asyncio.sleep(0.5)
            await self._send_offer()

            # 消息循环
            await self._signaling_loop()

        except asyncio.CancelledError:
            logger.info("推流任务取消")
        except Exception:
            logger.exception("推流异常")
        finally:
            await self._cleanup()

    async def _cleanup(self):
        self._running = False
        if self._video_track:
            self._video_track.stop()
        if self._pc:
            await self._pc.close()
        if self._ws:
            await self._ws.close()
        logger.info("WebRTC 资源已清理")
