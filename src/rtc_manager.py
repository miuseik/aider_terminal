"""RTC 生命周期管理 — 初始化/入会/推流/离会/释放"""

import sys
import os
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "sdk", "aliRtc")))

from AliRTCLinuxSdkDefine import *
import AliRTCEngine
from src.event_handler import MyEventHandler


class RTCManager:
    """封装 AliRTC 引擎的完整生命周期"""

    def __init__(self, event_handler: MyEventHandler):
        self._handler = event_handler
        self.engine = None
        self._base_dir = os.path.dirname(os.path.dirname(__file__))

    # ---------- 连接 ----------

    def connect(self) -> None:
        """初始化引擎 + 入会"""
        self._create_engine()
        self._join_channel()

    def disconnect(self) -> None:
        """离会 + 释放引擎"""
        if self.engine:
            self.engine.LeaveChannel()
            import asyncio
            loop = asyncio.get_event_loop()
            loop.run_until_complete(asyncio.sleep(1))
            self.engine.Release()
            self.engine = None

    # ---------- 推流 ----------

    def start_publish(self) -> None:
        """开启音视频推流"""
        self.engine.PublishLocalVideoStream(True)
        self.engine.PublishLocalAudioStream(True)
        self.engine.SetExternalVideoSource(True,
            sourceType=VideoSource.VideoSourceCamera,
            renderMode=RenderMode.RenderModeFill)
        self.engine.SetExternalAudioSource(True, sampleRate=16000, channelsPerFrame=1)

    def stop_publish(self) -> None:
        """关闭音视频推流"""
        self.engine.PublishLocalVideoStream(False)
        self.engine.PublishLocalAudioStream(False)

    # ---------- 推送数据 ----------

    def push_video(self, video_sample: VideoDataSample) -> int:
        """推送一帧视频"""
        return self.engine.PushExternalVideoFrame(video_sample, VideoSource.VideoSourceCamera)

    def push_audio(self, pcm_data: bytes, timestamp: int) -> int:
        """推送 PCM 音频数据"""
        return self.engine.PushExternalAudioFrameRawData(pcm_data, len(pcm_data), timestamp)

    # ---------- 内部 ----------

    def _create_engine(self) -> None:
        core_path = os.path.join(self._base_dir, "sdk", "aliRtc", "Release", "lib", "AliRtcCoreService")
        self.engine = AliRTCEngine.CreateAliRTCEngine(
            self._handler,
            42000, 45000,
            "/tmp",           # 日志目录
            core_path,
            False,            # h5 兼容模式
            "{}",             # 额外配置
        )

    def _join_channel(self) -> None:
        auth = AuthInfo()
        auth.appid = "1295a524-ff41-4bfc-ba3f-7c1c786738cd"
        auth.userid = "python_terminal"
        auth.username = "Python终端"
        auth.channel = "test123"

        appkey = "659fe17ceb1494befefd57559b094a0d"
        auth.token = self.engine.GenerateToken(auth, appkey)  # 生产环境换 AppServer 获取

        cfg = JoinChannelConfig()
        cfg.channelProfile = ChannelProfile.ChannelProfileInteractiveLive
        cfg.isAudioOnly = False
        cfg.subscribeMode = SubscribeMode.SubscribeAutomatically
        cfg.publishMode = PublishMode.PublishAutomatically

        self.engine.JoinChannel(auth.token, auth.channel, auth.userid, auth.username, cfg)
