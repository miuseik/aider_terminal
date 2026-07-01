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
from collections import deque
from typing import Optional

import numpy as np
from aiortc import (
    RTCConfiguration,
    RTCIceCandidate,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
    AudioStreamTrack,
    VideoStreamTrack,
)
from av import AudioFrame, VideoFrame
from av.audio.resampler import AudioResampler

from config.settings import TelegripConfig
from drivers.audio.audio_driver import AudioDriver
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


class MicrophoneAudioTrack(AudioStreamTrack):
    """aiortc AudioStreamTrack: 从 AudioDriver (input) 读取音频帧"""

    kind = "audio"

    def __init__(self, mic: AudioDriver, sample_rate: int = 16000):
        super().__init__()
        self._mic = mic
        self._sample_rate = sample_rate
        self._start_time: Optional[float] = None
        self._sample_count = 0
        self._samples_per_frame = 960  # 60ms at 16kHz, opus 兼容

        logger.info("MicrophoneAudioTrack: %dHz mono", self._sample_rate)

    async def recv(self) -> AudioFrame:
        loop = asyncio.get_event_loop()
        samples: Optional[np.ndarray] = await loop.run_in_executor(
            None, self._mic.read, self._samples_per_frame
        )

        if samples is None:
            samples = np.zeros(self._samples_per_frame, dtype=np.int16)

        self._sample_count += len(samples)
        if self._start_time is None:
            self._start_time = time.time()

        pts = int((time.time() - self._start_time) * self._sample_rate)
        frame = AudioFrame(format="s16", layout="mono", samples=len(samples))
        frame.planes[0].update(samples.tobytes())
        frame.sample_rate = self._sample_rate
        frame.pts = pts
        frame.time_base = fractions.Fraction(1, self._sample_rate)
        return frame

    def stop(self):
        logger.info("MicrophoneAudioTrack 已停止 (%d samples)", self._sample_count)


class WebRTCStreamer:
    """WebRTC 推流器 - 信令复用 /ws/terminal 通道，不再单独连接 /ws/signaling"""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self._transport = None          # WSTransport（复用 terminal 通道）
        self._msg_queue: Optional[asyncio.Queue] = None
        self._pc: Optional[RTCPeerConnection] = None
        self._video_track: Optional[CameraVideoTrack] = None
        self._audio_track: Optional[MicrophoneAudioTrack] = None
        self._camera: Optional[OpenCVCameraDriver] = None
        self._mic: Optional[AudioDriver] = None
        self._speaker: Optional[AudioDriver] = None
        self._remote_audio = None
        self._remote_audio_task: Optional[asyncio.Task] = None
        self._ice_servers: list = []
        self._running = False
        self._need_reconnect = False
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
        if msg_type in ("webrtc_joined", "answer", "ice_candidate", "subscriber_joined", "offer"):
            print(f"📡 Streamer 收到信令: {msg_type}")
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
        # 停止旧的 video/audio track（如果是重建）
        if self._video_track:
            self._video_track.stop()
            self._video_track = None
        if self._audio_track:
            self._audio_track.stop()
            self._audio_track = None

        ice_cfg = RTCConfiguration(iceServers=[
            RTCIceServer(urls=s["urls"], username=s.get("username"), credential=s.get("credential"))
            for s in self._ice_servers
        ])
        self._pc = RTCPeerConnection(configuration=ice_cfg)
        self._need_reconnect = False

        track_count = 0
        if not self._no_camera and self._camera:
            self._video_track = CameraVideoTrack(self._camera, self.config.camera_fps)
            self._pc.addTrack(self._video_track)
            track_count += 1

        if self.config.audio_enabled and self._mic:
            self._audio_track = MicrophoneAudioTrack(self._mic, self.config.audio_sample_rate)
            self._pc.addTrack(self._audio_track)
            track_count += 1

        print(f"📹 PeerConnection 已创建 (tracks={track_count})")
        logger.info("PeerConnection 已创建 (tracks=%d)", track_count)

        @self._pc.on("connectionstatechange")
        async def _on_conn():
            state = self._pc.connectionState
            print(f"🔗 WebRTC 连接状态: {state}")
            logger.info("连接状态: %s", state)
            if state in ("failed", "disconnected", "closed"):
                self._need_reconnect = True

        @self._pc.on("iceconnectionstatechange")
        async def _on_ice():
            state = self._pc.iceConnectionState
            print(f"🧊 ICE 状态: {state}")
            logger.info("ICE: %s", state)
            if state == "failed":
                self._need_reconnect = True

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

        @self._pc.on("track")
        async def _on_track(track):
            """接收远端（客户端）音频/视频 track"""
            print(f"📡 收到远端 track: kind={track.kind}")
            logger.info("收到远端 track: kind=%s", track.kind)
            if track.kind == "audio":
                self._remote_audio = track
                # 如果有 speaker，启动音频播放任务
                if self._speaker and self._speaker.is_connected:
                    await self._start_remote_audio(track)
            # 注：视频不需要从客户端接收（客户端不推视频）

    async def _send_offer(self):
        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        await self._send({
            "type": "offer",
            "sdp": self._pc.localDescription.sdp,
            "room_id": self.config.webrtc_room_id,
        })
        print("📹 SDP Offer 已发送")
        logger.info("SDP Offer 已发送")

    # ── 消息处理 ──

    async def _handle_answer(self, msg: dict):
        # 只有在 have-local-offer 状态才能接受 answer（stable/closed 都无效）
        if not self._pc or self._pc.signalingState != "have-local-offer":
            print(f"📡 忽略 answer: PC 状态 {self._pc.signalingState if self._pc else 'None'}")
            return
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=msg["sdp"], type="answer"))
        print("📡 远程 SDP (answer) 已设置")
        logger.info("远程 SDP 已设置")

    async def _handle_ice(self, msg: dict):
        if not self._pc or self._pc.signalingState == "closed":
            return
        c = msg.get("candidate")
        if not c:
            return
        try:
            await self._pc.addIceCandidate(RTCIceCandidate(
                component=c.get("component", 1),
                foundation=c.get("foundation", ""), ip=c.get("ip", ""),
                port=c.get("port", 0), priority=c.get("priority", 0),
                protocol=c.get("protocol", "udp"), type=c.get("type", "host"),
                sdpMid=msg.get("sdpMid"), sdpMLineIndex=msg.get("sdpMLineIndex"),
            ))
        except Exception:
            pass

    async def _handle_subscriber_joined(self, msg: dict):
        print(f"📹 订阅者加入 (×{msg.get('count', 0)})")
        logger.info("订阅者加入 (×%d)", msg.get("count", 0))
        # 每次都重建 PC + 发全新 offer，保证 ICE candidates 新鲜有效
        # （旧 offer 的 candidates 经过几百 ms 可能已过期）
        if self._pc:
            await self._pc.close()
            self._pc = None
        self._create_peer_connection()
        await asyncio.sleep(0.5)
        await self._send_offer()
        print("📹 新 Offer 已发送")

    async def _handle_client_offer(self, msg: dict):
        """处理客户端发来的 renegotiation offer（客户端添加 mic track 时）"""
        if not self._pc:
            print("📡 忽略客户端 offer: PC 未创建")
            return
        print("📡 收到客户端 renegotiation offer")
        logger.info("收到客户端 renegotiation offer")
        try:
            await self._pc.setRemoteDescription(
                RTCSessionDescription(sdp=msg["sdp"], type="offer"))
            answer = await self._pc.createAnswer()
            await self._pc.setLocalDescription(answer)
            await self._send({
                "type": "answer",
                "sdp": self._pc.localDescription.sdp,
                "room_id": self.config.webrtc_room_id,
            })
            print("📡 Renegotiation answer 已发送")
            logger.info("Renegotiation answer 已发送")
        except Exception:
            logger.exception("处理客户端 renegotiation offer 失败")

    async def _start_remote_audio(self, track):
        """启动远端音频播放任务（带重采样和抖动缓冲）"""
        if self._remote_audio_task and not self._remote_audio_task.done():
            return  # 已经在播放
        if not self._speaker or not self._speaker.is_connected:
            print("📡 扬声器未就绪，跳过远端音频")
            return

        target_rate = self._speaker.sample_rate
        resampler: Optional[AudioResampler] = None

        async def _play():
            nonlocal resampler
            print("🔊 开始播放远端音频")
            logger.info("开始播放远端音频")

            # 抖动缓冲：预填充 3 帧再开始播放，平滑网络抖动
            buffer: deque = deque()
            MIN_PREBUFFER = 3
            MAX_BUFFER = 12
            started = False

            try:
                while self._running and self._speaker and self._speaker.is_connected:
                    frame = await track.recv()

                    # ── 重采样 ──
                    if frame.sample_rate != target_rate:
                        if resampler is None:
                            fmt = frame.format.name if frame.format else "s16"
                            layout = frame.layout.name if frame.layout else "mono"
                            resampler = AudioResampler(
                                format=fmt, layout=layout, rate=target_rate
                            )
                            logger.info(
                                "音频重采样: %dHz %s → %dHz",
                                frame.sample_rate, layout, target_rate,
                            )
                        frame = resampler.resample(frame)

                    data = frame.to_ndarray()

                    # ── 抖动缓冲 ──
                    buffer.append(data)
                    while len(buffer) > MAX_BUFFER:
                        buffer.popleft()  # 丢弃最旧帧防止延迟累积

                    if not started:
                        if len(buffer) < MIN_PREBUFFER:
                            continue
                        started = True
                        logger.debug("抖动缓冲就绪 (%d 帧)", len(buffer))

                    # 从缓冲区取一帧写入扬声器
                    data = buffer.popleft()
                    loop = asyncio.get_event_loop()
                    await loop.run_in_executor(None, self._speaker.write, data)

            except asyncio.CancelledError:
                pass
            except Exception:
                logger.exception("远端音频播放异常")
            finally:
                print("🔇 远端音频播放停止")
                logger.info("远端音频播放停止")

        self._remote_audio_task = asyncio.ensure_future(_play())

    async def _reconnect(self):
        """断线重连：关闭旧 PC，重建并重发 Offer"""
        print("🔄 开始 WebRTC 重连...")
        logger.info("开始重连...")
        if self._pc:
            try:
                await self._pc.close()
            except Exception:
                pass
            self._pc = None
        self._need_reconnect = False

        # 等待一小段时间再重连
        await asyncio.sleep(1)

        try:
            self._create_peer_connection()
            await asyncio.sleep(0.5)
            await self._send_offer()
            print("✅ WebRTC 重连完成")
            logger.info("重连完成")
        except Exception as e:
            print(f"❌ WebRTC 重连失败: {e}")
            logger.exception("重连失败")
            raise

    async def _signaling_loop(self):
        """从消息队列读取信令，支持断线自动重连"""
        print("📡 信令循环已启动")
        handlers = {
            "answer": self._handle_answer,
            "ice_candidate": self._handle_ice,
            "subscriber_joined": self._handle_subscriber_joined,
            "offer": self._handle_client_offer,
        }
        reconnect_count = 0
        max_reconnects = 10

        while self._running and self._msg_queue is not None:
            # 检测是否需要重连
            if self._need_reconnect:
                reconnect_count += 1
                if reconnect_count > max_reconnects:
                    print(f"❌ 已达最大重连次数 ({max_reconnects})，退出")
                    break
                await self._reconnect()
                continue

            try:
                msg = await asyncio.wait_for(self._msg_queue.get(), timeout=10)
            except asyncio.TimeoutError:
                # 超时时也检查是否需要重连
                if self._need_reconnect:
                    continue
                # 检查 transport 是否还连着
                if self._transport and not self._transport.is_connected:
                    print("⚠️ Transport 已断开，等待重连...")
                    self._need_reconnect = True
                    continue
                continue
            handler = handlers.get(msg.get("type"))
            if handler:
                print(f"📡 信令循环处理: {msg.get('type')}")
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

            # 麦克风
            if self.config.audio_enabled:
                mic_cfg = {
                    "device": self.config.audio_device,
                    "sample_rate": self.config.audio_sample_rate,
                    "channels": 1,
                }
                self._mic = AudioDriver(mic_cfg, mode="input")
                if not self._mic.connect():
                    logger.warning("麦克风连接失败，音频已禁用")
                    self.config.audio_enabled = False
                    self._mic = None

            # 扬声器（接收远端音频）
            if self.config.audio_enabled:
                speaker_cfg = {
                    "device": self.config.audio_device,
                    "sample_rate": self.config.audio_sample_rate,
                    "channels": 1,
                }
                self._speaker = AudioDriver(speaker_cfg, mode="output")
                if not self._speaker.connect():
                    logger.warning("扬声器连接失败")
                    self._speaker = None

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
        if self._audio_track:
            self._audio_track.stop()
            self._audio_track = None
        if self._pc:
            await self._pc.close()
            self._pc = None
        if self._camera:
            self._camera.disconnect()
            self._camera = None
        if self._mic:
            self._mic.disconnect()
            self._mic = None
        if self._remote_audio_task:
            self._remote_audio_task.cancel()
            self._remote_audio_task = None
        if self._speaker:
            self._speaker.disconnect()
            self._speaker = None
        logger.info("WebRTC 资源已清理")
