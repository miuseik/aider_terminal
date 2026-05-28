"""RTC 回调处理 — 所有 SDK 事件在这里统一响应"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "sdk", "aliRtc")))

from AliRTCLinuxSdkDefine import *
import AliRTCEngine


class MyEventHandler(AliRTCEngine.EngineEventHandlerInterface):
    """
    收到 SDK 事件时会自动调用对应方法，只需重写你关心的事件。
    业务逻辑建议通过回调/队列传给 src/ 下其他模块处理，不要在这里写太重。
    """

    # ---- 错误 & 状态 ----
    def OnError(self, error_code: ERROR_CODE) -> None:
        print(f"[RTC] 出错: {error_code}")

    def OnWarning(self, warning_code: WARNNING_CODE) -> None:
        print(f"[RTC] 警告: {warning_code}")

    def OnConnectionStatusChanged(self, status, reason) -> None:
        print(f"[RTC] 连接状态变化: status={status}, reason={reason}")

    # ---- 频道 ----
    def OnJoinChannelResult(self, result: int, channel: str, userId: str) -> None:
        if result == 0:
            print(f"[RTC] 入会成功! channel={channel}, userId={userId}")
        else:
            print(f"[RTC] 入会失败, error={result}")

    def OnLeaveChannelResult(self, result: int) -> None:
        print(f"[RTC] 已离会, result={result}")

    # ---- 远端用户 ----
    def OnRemoteUserOnLineNotify(self, uid: str) -> None:
        print(f"[RTC] 远端用户上线: {uid}")

    def OnRemoteUserOffLineNotify(self, uid: str) -> None:
        print(f"[RTC] 远端用户下线: {uid}")

    def OnRemoteUserSubscribedDataChannel(self, uid: str) -> None:
        print(f"[RTC] 远端 {uid} 的 DataChannel 已就绪")

    # ---- 推流状态 ----
    def OnAudioPublishStateChanged(self, oldState, newState, elapse, channel):
        if newState == AliRTCEngine.AliEnginePublishState.AliEngineStatsPublished:
            print("[RTC] 音频推流已就绪")

    def OnVideoPublishStateChanged(self, oldState, newState, elapse, channel):
        if newState == AliRTCEngine.AliEnginePublishState.AliEngineStatsPublished:
            print("[RTC] 视频推流已就绪")

    # ---- 接收远端数据 ----
    def OnSubscribeAudioFrame(self, uid: str, frame: AliRTCEngine.AudioFrame) -> None:
        """远端单路音频 (AudioFormatPcmBeforMixing 模式)"""
        # TODO: 传给 src/audio_receiver.py
        pass

    def OnSubscribeMixAudioFrame(self, frame: AliRTCEngine.AudioFrame) -> None:
        """远端混音 (AudioFormatMixedPcm 模式)"""
        # TODO: 传给 src/audio_receiver.py
        pass

    def OnRemoteVideoSample(self, uid: str, frame: AliRTCEngine.VideoFrame) -> None:
        """远端视频帧"""
        # TODO: 传给 src/video_receiver.py
        pass

    def OnDataChannelMsg(self, uid: str, msg: AliRTCEngine.AliEngineDataChannelMsg) -> None:
        """自定义消息"""
        data = msg.data.decode("utf-8") if msg.data else ""
        print(f"[RTC] DataChannel [{uid}]: {data}")
        # TODO: 传给 src/message_handler.py

    def OnMediaExtensionMsgReceived(self, userid: str, message: bytes, size: int) -> None:
        """SEI 消息"""
        # TODO: 传给 src/sei_handler.py
        pass

    # ---- 缓冲状态（推流过快时会触发，需做流控） ----
    def OnPushAudioFrameBufferFull(self, isFull: bool) -> None:
        if isFull:
            print("[RTC] 音频缓冲已满，请降速")

    def OnPushVideoFrameBufferFull(self, isFull: bool) -> None:
        if isFull:
            print("[RTC] 视频缓冲已满，请降速")
