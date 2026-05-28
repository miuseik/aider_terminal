import datetime
import asyncio
import json
import time
import threading
import os

from AliRTCLinuxSdkDefine import *
import AliRTCEngine

stopSignal = False
PushAudioFull, PushVideoFull = False, False
startPush = False

def get_input(linuxEngine):
    loop2 = asyncio.new_event_loop()
    asyncio.set_event_loop(loop2)
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
    eventHandler = EngineEventListener()
    h5mode = False # 与Web端互通请设置为True，开启H5兼容模式
    currentPath = os.getcwd()
    coreServicePath = os.path.abspath(os.path.join(currentPath, "Release", "lib", "AliRtcCoreService"))
    extra_jobj = {
        "user_specified_disable_audio_ranking": "true"
    }
    extra = json.dumps(extra_jobj)
    linuxEngine = AliRTCEngine.CreateAliRTCEngine(eventHandler, 42000, 45000, "/tmp", coreServicePath, h5mode, extra)
    
    authInfo = AuthInfo()
    authInfo.appid = 'xxx'
    appkey = 'xxx'
    authInfo.userid = 'testUser'
    authInfo.username = authInfo.userid
    authInfo.channel = '12345'
    expire = datetime.datetime.now() + datetime.timedelta(days=1)
    authInfo.timestamp = int(time.mktime(expire.timetuple()))
    authInfo.token = linuxEngine.GenerateToken(authInfo, appkey)

    joinConfig = JoinChannelConfig()
    joinConfig.channelProfile = ChannelProfile.ChannelProfileInteractiveLive
    joinConfig.subscribeAudioFormat = AudioFormat.AudioFormatPcmBeforMixing
    joinConfig.subscribeVideoFormat = VideoFormat.VideoFormatH264
    joinConfig.isAudioOnly = False
    joinConfig.publishAvsyncMode = PublishAvsyncMode.PublishAvsyncWithPts
    joinConfig.subscribeMode = SubscribeMode.SubscribeAutomatically
    joinConfig.publishMode = PublishMode.PublishAutomatically

    linuxEngine.PublishLocalVideoStream(True)
    linuxEngine.PublishLocalAudioStream(True)
    videoConfig = AliEngineVideoEncoderConfiguration(width=640, height=480, f=AliEngineFrameRate.AliEngineFrameRateFps15, b=512, \
                                                     ori=AliEngineVideoEncoderOrientationMode.AliEngineVideoEncoderOrientationModeAdaptive, \
                                                     mr=AliEngineVideoMirrorMode.AliEngineVideoMirrorModeDisabled, \
                                                     rotation=AliEngineRotationMode.AliEngineRotationMode_0)
    linuxEngine.SetVideoEncoderConfiguration(videoConfig)
    linuxEngine.SetExternalVideoSource(True, sourceType=VideoSource.VideoSourceCamera, renderMode=RenderMode.RenderModeFill)
    linuxEngine.SetExternalAudioSource(True, sampleRate=16000, channelsPerFrame=1)
    # 设置用户角色，若Linux虚拟用户入会不想被其他主播察觉，请设置为观众角色
    linuxEngine.SetClientRole(AliEngineClientRole.AliEngineClientRoleInteractive)

    linuxEngine.JoinChannel(authInfo.token, authInfo.channel, authInfo.userid, authInfo.username, joinConfig)

    input_thread = threading.Thread(target=get_input, args=(linuxEngine,))
    while not startPush:
        sleepFor(0.1)
    input_thread.start()

    videoPushEnd, audioPushEnd = False, False
    fps = 15
    audioSampleRate = 16000
    audioSampleChannel = 1
    sampleMs = 1000//fps
    v_ts, a_ts = 0, 0
    with open("/mnt/test.rgb", 'rb') as videoFile, open("/mnt/test.pcm", 'rb') as audioFile:
        while not videoPushEnd or not audioPushEnd:
            if stopSignal:
                break
            # construct video sample
            if not PushVideoFull and not videoPushEnd:
                videoSample = VideoDataSample()
                videoSample.width = 720
                videoSample.height = 1280
                videoSample.strideY = videoSample.width
                videoSample.strideU = videoSample.width
                videoSample.strideV = videoSample.width
                videoSample.dataLen = videoSample.width * videoSample.height * 3
                videoSample.format = VideoDataFormat.VideoDataFormatRGB24
                videoSample.bufferType = VideoBufferType.VideoBufferTypeRawData
                videoSample.rotation = 0
                videoSample.data = videoFile.read(videoSample.dataLen)
                if not videoSample.data or len(videoSample.data) < videoSample.dataLen:
                    videoPushEnd = True
                else:
                    videoSample.timeStamp = v_ts
                    linuxEngine.PushExternalVideoFrame(videoSample, VideoSource.VideoSourceCamera)
                    v_ts += 1000 // fps

            # construct audio sample
            if not PushAudioFull and not audioPushEnd:
                frameLength = (audioSampleRate // 1000) * sampleMs * 2 * audioSampleChannel
                audioSample = audioFile.read(frameLength)
                if not audioSample or len(audioSample) < frameLength:
                    audioPushEnd = True
                else:
                    linuxEngine.PushExternalAudioFrameRawData(audioSample, frameLength, a_ts)
                    a_ts += sampleMs
            
            sleepFor(10/1000) # 10ms，PublishAvsyncWithPts模式下以时间戳为准

    input_thread.join()
    # sleepFor(20) # 等待素材播放完成

    linuxEngine.PublishLocalVideoStream(False)
    linuxEngine.PublishLocalAudioStream(False)
    linuxEngine.LeaveChannel()

    while not stopSignal:
        sleepFor(1)
    linuxEngine.Release() # 建议离会后再释放资源
    linuxEngine = None


if __name__ == '__main__':
    main()
