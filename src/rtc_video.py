import datetime
import asyncio
import json
import time
import threading
import os
import sys
import cv2

# 将 drivers/ali_rtc 目录加入 Python 路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'drivers', 'ali_rtc')))

from AliRTCLinuxSdkDefine import *
import AliRTCEngine

stopSignal = False
isRunning = False  # 新增：跟踪视频流是否运行中
PushAudioFull, PushVideoFull = False, False
startPush = False

def get_input(linuxEngine):
    while True:
        user_input = input()    
        command_array = user_input.split()
        if command_array[0] == "exit":
            global stopSignal
            stopSignal = True
            print(f"Python exitting...")
            break

class EngineEventListener(AliRTCEngine.EngineEventHandlerInterface):
    def OnError(self, error_code:ERROR_CODE) -> None:
        print(f"[Python] on error occurred: {error_code.value}")

    def OnWarning(self, warning_code:WARNNING_CODE) -> None:
        print(f"[Python] on warning occurred: {warning_code.value}")
   
    def OnJoinChannelResult(self, result:int, channel:str, userId:str) -> None:
        print(f"[Python] on join channel result. Channel: {channel}, user: {userId}, result: {result}")

    def OnLeaveChannelResult(self, result:int) -> None:
        print(f"[Python] on leave channel result: {result}")
        global stopSignal
        stopSignal = True

    def OnAudioPublishStateChanged(self, oldState: AliRTCEngine.AliEnginePublishState, newState: AliRTCEngine.AliEnginePublishState, elapseSinceLastState: int, channel: str) -> None:
        print(f"[Python] on audio publish state changed, oldState: {oldState.value}, newState: {newState.value}")
        if newState == AliRTCEngine.AliEnginePublishState.AliEngineStatsPublished:
            global startPush
            startPush = True

    def OnAudioSubscribeStateChanged(self, uid: str, oldState: AliRTCEngine.AliEngineSubscribeState, newState: AliRTCEngine.AliEngineSubscribeState, elapseSinceLastState: int, channel: str) -> None:
        print(f"[Python] on audio subscribe state of {uid}, oldState: {oldState.value}, newState: {newState.value}")
    
    def OnRemoteUserOnLineNotify(self, uid: str) -> None:
        print(f"[Python] on remote user online: {uid}")

    def OnRemoteUserOffLineNotify(self, uid: str) -> None:
        print(f"[Python] on remote user offline: {uid}")
    
    def OnVideoPublishStateChanged(self, oldState: AliRTCEngine.AliEnginePublishState, newState: AliRTCEngine.AliEnginePublishState, elapseSinceLastState: int, channel: str) -> None:
        print(f"[Python] on video publish state changed, oldState: {oldState.value}, newState: {newState.value}")
        if newState == AliRTCEngine.AliEnginePublishState.AliEngineStatsPublished:
            global startPush
            startPush = True

    def OnVideoSubscribeStateChanged(self, uid: str, oldState: AliRTCEngine.AliEngineSubscribeState, newState: AliRTCEngine.AliEngineSubscribeState, elapseSinceLastState: int, channel: str) -> None:
        print(f"[Python] on video subscribe state of {uid}, oldState: {oldState.value}, newState: {newState.value}")
    
    def OnDualStreamPublishStateChanged(self, oldState: AliRTCEngine.AliEnginePublishState, newState: AliRTCEngine.AliEnginePublishState, elapseSinceLastState: int, channel: str) -> None:
        print(f"[Python] on dual stream publish state changed, oldState: {oldState.value}, newState: {newState.value}")

    def OnUpdateRoleNotify(self, oldRole: AliRTCEngine.AliEngineClientRole, newRole: AliRTCEngine.AliEngineClientRole) -> None:
        print(f"[Python] on update role, oldRole: {oldRole.value}, newRole: {newRole.value}")

    def OnPushAudioFrameBufferFull(self, isFull: bool) -> None:
        global PushAudioFull
        PushAudioFull = isFull

    def OnPushVideoFrameBufferFull(self, isFull:bool) -> None:
        global PushVideoFull
        PushVideoFull = isFull

    def OnDataChannelMsg(self, uid: str, msg: AliRTCEngine.AliEngineDataChannelMsg) -> None:
        dataChannelMsg = msg.data.decode('utf-8')
        print(f"[Python] on data channel msg from {uid}: {dataChannelMsg}")
    
    def OnMediaExtensionMsgReceived(self, userid: str, message: bytes, size: int) -> None:
        seiMsg = message.decode('utf-8')
        print(f"[Python] on sei msg from {userid}: {seiMsg}")
    
    def OnRemoteVideoSample(self, uid: str, frame: AliRTCEngine.VideoFrame) -> None:
        pass
    
    def OnSubscribeAudioFrame(self, uid: str, frame: AliRTCEngine.AudioFrame) -> None:
        pass
    
    def OnSubscribeMixAudioFrame(self, frame: AliRTCEngine.AudioFrame) -> None:
        pass

    def OnConnectionStatusChanged(self, status:AliEngineConnectionStatus, reason:AliEngineConnectionStatusChangeReason) -> None:
        pass

def sleepFor(seconds: float) -> None:
    async def internalSleep(seconds: float) -> None:
        await asyncio.sleep(seconds)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(internalSleep(seconds))

def main():
    # 必须在函数体最开始声明所有全局变量
    global isRunning, PushVideoFull, PushAudioFull, startPush, stopSignal
    isRunning = True

    # Python 3.10+ 不会自动创建事件循环，需要手动创建
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    eventHandler = EngineEventListener()
    h5mode = True # 与Web端互通请设置为True，开启H5兼容模式
    currentPath = os.path.dirname(os.path.abspath(__file__))
    coreServicePath = os.path.abspath(os.path.join(currentPath, "..", "drivers", "ali_rtc", "Release", "native", "AliRtcCoreService"))
    extra_jobj = {
        "user_specified_disable_audio_ranking": "true"
    }
    extra = json.dumps(extra_jobj)
    linuxEngine = AliRTCEngine.CreateAliRTCEngine(eventHandler, 42000, 45000, "/tmp", coreServicePath, h5mode, extra)
    
    authInfo = AuthInfo()
    authInfo.appid = '1295a524-ff41-4bfc-ba3f-7c1c786738cd'
    appkey = '659fe17ceb1494befefd57559b094a0d'
    authInfo.userid = 'python_terminal'
    authInfo.username = 'Python终端'
    authInfo.channel = 'test123'
    expire = datetime.datetime.now() + datetime.timedelta(days=1)
    authInfo.timestamp = int(time.mktime(expire.timetuple()))
    authInfo.token = linuxEngine.GenerateToken(authInfo, appkey)

    joinConfig = JoinChannelConfig()
    joinConfig.channelProfile = ChannelProfile.ChannelProfileInteractiveWithLowLatencyLive
    joinConfig.subscribeAudioFormat = AudioFormat.AudioFormatPcmBeforMixing
    joinConfig.subscribeVideoFormat = VideoFormat.VideoFormatH264
    joinConfig.isAudioOnly = False
    joinConfig.publishAvsyncMode = PublishAvsyncMode.PublishAvsyncNoDelay
    joinConfig.subscribeMode = SubscribeMode.SubscribeAutomatically
    joinConfig.publishMode = PublishMode.PublishAutomatically

    linuxEngine.PublishLocalVideoStream(True)
    linuxEngine.PublishLocalAudioStream(True)
    videoConfig = AliEngineVideoEncoderConfiguration(width=1920, height=1080, f=AliEngineFrameRate.AliEngineFrameRateFps25, b=5000, \
                                                     ori=AliEngineVideoEncoderOrientationMode.AliEngineVideoEncoderOrientationModeAdaptive, \
                                                     mr=AliEngineVideoMirrorMode.AliEngineVideoMirrorModeDisabled, \
                                                     rotation=AliEngineRotationMode.AliEngineRotationMode_0)
    videoConfig.keyFrameInterval = 500   # 0.5秒一个关键帧，快速清除残影
    videoConfig.minBitrate = 3000
    linuxEngine.SetVideoEncoderConfiguration(videoConfig)
    linuxEngine.SetExternalVideoSource(True, sourceType=VideoSource.VideoSourceCamera, renderMode=RenderMode.RenderModeFill)
    linuxEngine.SetExternalAudioSource(True, sampleRate=16000, channelsPerFrame=1)
    # 设置用户角色，若Linux虚拟用户入会不想被其他主播察觉，请设置为观众角色
    linuxEngine.SetClientRole(AliEngineClientRole.AliEngineClientRoleInteractive)

    linuxEngine.JoinChannel(authInfo.token, authInfo.channel, authInfo.userid, authInfo.username, joinConfig)

    input_thread = threading.Thread(target=get_input, args=(linuxEngine,))
    
    # 优化：等待推流就绪后再启动输入线程
    while not startPush:
        sleepFor(0.1)
    input_thread.start()
    
    print("[Python] 推流已就绪，正在等待观众...")
    print("[Python] 提示：即使没有观众，推流也会产生费用！")

    fps = 25
    audioSampleRate = 16000
    audioSampleChannel = 1
    sampleMs = 1000 // fps
    v_ts, a_ts = 0, 0
    
    # 打开真实摄像头
    cap = None
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[Python] 摄像头打开失败！")
        else:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            cap.set(cv2.CAP_PROP_FPS, fps)
            videoWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            videoHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"[Python] 摄像头已打开: {videoWidth}x{videoHeight}")
            frameSize = videoWidth * videoHeight * 3
            
            # 静音音频帧
            frameLength = (audioSampleRate // 1000) * sampleMs * 2 * audioSampleChannel
            silenceFrame = b'\x00' * frameLength
            
            frameInterval = 1.0 / fps
            lastPushTime = 0
            
            while True:
                if stopSignal:
                    break
                
                elapsed = time.time() - lastPushTime
                
                # 按帧率推视频帧
                if not PushVideoFull and elapsed >= frameInterval:
                    ret, frame = cap.read()
                    if not ret:
                        print("[Python] 读帧失败，停止推流")
                        break
                    # OpenCV 读出来是 BGR，直接推 BGR 避免 RGB 转换开销
                    videoSample = VideoDataSample()
                    videoSample.width = videoWidth
                    videoSample.height = videoHeight
                    videoSample.strideY = videoSample.width
                    videoSample.strideU = videoSample.width
                    videoSample.strideV = videoSample.width
                    videoSample.dataLen = frameSize
                    videoSample.format = VideoDataFormat.VideoDataFormatBGR24
                    videoSample.bufferType = VideoBufferType.VideoBufferTypeRawData
                    videoSample.rotation = 0
                    videoSample.data = frame.tobytes()
                    videoSample.timeStamp = v_ts
                    linuxEngine.PushExternalVideoFrame(videoSample, VideoSource.VideoSourceCamera)
                    v_ts += 1000 // fps
                    lastPushTime = time.time()
                
                # 推音频帧（按音频速率推送，避免每循环都推）
                if not PushAudioFull:
                    linuxEngine.PushExternalAudioFrameRawData(silenceFrame, frameLength, a_ts)
                    a_ts += sampleMs
                
                time.sleep(0.001)
    finally:
        if cap is not None:
            cap.release()
            print("[Python] 摄像头已释放")

    # 注意：这里不调用 input_thread.join()，因为 input_thread 只是等待用户输入"exit"，
    # 这个输入永远不会来，join() 会永远阻塞
    # input_thread.join()
    # sleepFor(20) # 等待素材播放完成

    isRunning = False  # 标记推流已停止

    linuxEngine.PublishLocalVideoStream(False)
    linuxEngine.PublishLocalAudioStream(False)
    linuxEngine.LeaveChannel()

    # 不再等待 stopSignal，因为我们已经处理完退出逻辑了
    # linuxEngine.Release() 会释放资源
    try:
        print("[Python] 释放 AliRTC 引擎...")
        linuxEngine.Release()
        print("[Python] AliRTC 引擎已释放")
    except Exception as e:
        print(f"[Python] Release 时出错: {e}")

