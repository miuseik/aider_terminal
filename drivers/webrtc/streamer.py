"""
WebRTC 视频推流器

通过 aiortc 将摄像头画面推送到 aider_server，信令复用在 /ws/terminal 通道上。
作为 asyncio Task 运行，由 TelegripSystem 管理生命周期。
"""

import asyncio
import fractions
import json
import logging
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
        """停止帧读取（摄像头本身由 WebRTCStreamer._cleanup 释放）"""
        logger.info("CameraVideoTrack 已停止 (%d 帧)", self._frame_count)


class WebRTCStreamer:
    """WebRTC 推流器 - 信令复用 /ws/terminal 通道，不再单独连接 /ws/signaling"""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self._transport = None          # WSTransport（复用 terminal 通道）
        self._msg_queue: Optional[asyncio.Queue] = None
        self._pc: Optional[RTCPeerConnection] = None
        self._video_track: Optional[CameraVideoTrack] = None
        self._camera: Optional[OpenCVCameraDriver] = None
        self._ice_servers: list = []
        self._running = False
        self._joined: asyncio.Event = asyncio.Event()  # 加入房间完成信号

    # ── 设置 transport ──

    def set_transport(self, transport):
        """注入 terminal 的 WSTransport，信令消息通过它收发"""
        self._transport = transport
        self._msg_queue = asyncio.Queue()
        transport.add_handler(self._on_signaling_message)
        logger.info("WebRTC 信令已绑定到 terminal 通道")

    async def _on_signaling_message(self, raw: str):
        """transport 消息回调：把 WebRTC 相关消息推入队列"""
        if self._msg_queue is None:
            return
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        msg_type = msg.get("type", "")
        # 只处理 WebRTC 信令消息
        if msg_type in ("webrtc_joined", "answer", "ice_candidate", "subscriber_joined"):
            await self._msg_queue.put(msg)

    # ── 发送信令 ──

    async def _send(self, data: dict):
        """通过 terminal transport 发送信令消息"""
        if self._transport and self._transport.is_connected:
            await self._transport.send_raw(json.dumps(data))

    # ── 加入房间 ──

    async def _join_room(self):
        await self._send({
            "type": "webrtc_join",
            "role": "pub",
            "room_id": self.config.webrtc_room_id,
        })
        # 等待服务器响应（带轮询，避免 PyBullet 等阻塞事件循环时收不到消息）
        deadline = time.time() + 60  # PyBullet 加载 + Pinocchio 构建可能耗时 20s+
        while time.time() < deadline:
            await asyncio.sleep(0)
            try:
                resp = self._msg_queue.get_nowait()
                if resp.get("type") == "webrtc_joined":
                    self._ice_servers = resp.get("ice_servers", [])
                    self._joined.set()
                    logger.info("已加入房间 %s, ICE ×%d", self.config.webrtc_room_id, len(self._ice_servers))
                    return
                await self._msg_queue.put(resp)
            except asyncio.QueueEmpty:
                pass
            await asyncio.sleep(0.05)
        
        # 超时
        if self.config.ice_servers:
            self._ice_servers = self.config.ice_servers
            logger.warning("webrtc_joined 超时，降级使用本地 ICE 配置 ×%d", len(self._ice_servers))
            return
        raise RuntimeError("加入 WebRTC 房间超时，且无本地 ICE 配置")

    # ── PeerConnection ──

    def _create_peer_connection(self):
        # 停止旧的 video track（如果是重建）
        if self._video_track:
            self._video_track.stop()
            self._video_track = None

        ice_cfg = RTCConfiguration(iceServers=[
            RTCIceServer(urls=s["urls"], username=s.get("username"), credential=s.get("credential"))
            for s in self._ice_servers
        ])
        self._pc = RTCPeerConnection(configuration=ice_cfg)

        if not self._no_camera and self._camera:
            self._video_track = CameraVideoTrack(self._camera, self.config.camera_fps)
            self._pc.addTrack(self._video_track)
            logger.info("PeerConnection 已创建 (含摄像头)")
        else:
            logger.info("PeerConnection 已创建 (无摄像头, 仅信令测试)")

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
            await self._send({
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
            })

    async def _send_offer(self):
        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        await self._send({
            "type": "offer",
            "sdp": self._pc.localDescription.sdp,
            "room_id": self.config.webrtc_room_id,
        })
        logger.info("SDP Offer 已发送")

    # ── 消息处理 ──

    async def _handle_answer(self, msg: dict):
        if not self._pc or self._pc.signalingState == "closed":
            logger.warning("忽略 answer: PC 已关闭")
            return
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=msg["sdp"], type="answer"))
        logger.info("远程 SDP 已设置")

    async def _handle_ice(self, msg: dict):
        if not self._pc or self._pc.signalingState == "closed":
            return
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
        logger.info("订阅者加入 (×%d)", msg.get("count", 0))
        # 如果 PC 不存在或不健康，重建后发新 offer
        unhealthy = (
            not self._pc
            or self._pc.signalingState == "closed"
            or self._pc.connectionState in ("failed", "closed")
        )
        if unhealthy:
            logger.info("PC 状态异常 (%s)，重建 PeerConnection",
                        self._pc.signalingState if self._pc else "None")
            if self._pc:
                await self._pc.close()
            # 旧的 video track 不再需要，断开与 PC 的关联即可，
            # camera 本身由 self._camera 持有，不需要重新打开
            self._create_peer_connection()
            await asyncio.sleep(0.3)
            await self._send_offer()
        elif self._pc.localDescription:
            await self._send({
                "type": "offer",
                "sdp": self._pc.localDescription.sdp,
                "room_id": self.config.webrtc_room_id,
            })

    async def _signaling_loop(self):
        """从消息队列读取信令，直到 _running 为 False 或 PC 关闭"""
        handlers = {
            "answer": self._handle_answer,
            "ice_candidate": self._handle_ice,
            "subscriber_joined": self._handle_subscriber_joined,
        }
        while self._running and self._msg_queue is not None:
            # PC 已关闭则退出信令循环
            if self._pc and self._pc.signalingState == "closed":
                logger.info("PC 已关闭，退出信令循环")
                break
            try:
                msg = await asyncio.wait_for(self._msg_queue.get(), timeout=30)
            except asyncio.TimeoutError:
                continue
            handler = handlers.get(msg.get("type"))
            if handler:
                await handler(msg)

    # ── 生命周期 ──

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
            self._no_camera = (self.config.video_source == "none")
            if not self._no_camera and not self._camera.connect():
                raise RuntimeError("摄像头连接失败")

            # 等待 terminal 通道就绪
            for _ in range(30):
                if self._transport and self._transport.is_connected:
                    break
                logger.info("等待 terminal 通道就绪...")
                await asyncio.sleep(1)
            else:
                raise RuntimeError("等待 terminal 通道超时")

            # 加入房间
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
            self._video_track = None
        if self._pc:
            await self._pc.close()
            self._pc = None
        if self._camera:
            self._camera.disconnect()
            self._camera = None
        logger.info("WebRTC 资源已清理")
