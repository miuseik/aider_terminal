# 一、准备工作
1. 解压附件，交付产物包括如下：
```bash
linux_sdk_demo/
├── Release
│   └── lib
│       ├── AliRtcCoreService
│       ├── libAliRtcLinuxEngine.so
│       └── libonnxruntime.so.1.16.3
└── Demo
    ├── demo.py
    ├── AliRTCEngine.py
    ├── AliRTCEngineImpl.py
    └── AliRTCLinuxSdkDefine.py
```
1. 请确保您的Linux内核版本在2.6以上，且Python运行时为3.6及以上版本。若您希望基于C++接口进行额外开发，还需确保GCC版本在4.8以上。
</br> 
1. Release目录包含需要链接的SDK动态库(lib)，以及打包后的elf文件。Demo目录包含对应的Python接口封装，从而通过Python代码实现与C++接口一致的RTC推拉流功能。
</br>
1. Demo/demo.py目录提供简易例程，需要将appid、appkey替换以执行。
</br>

# 二、核心API
### 1.初始化SDK
步骤：

1）实现`EngineEventHandlerInterface`类，该类型包含RTC所需回调，收到数据或状态变更，其中的回调函数将被触发 </br>
2）创建`EngineEventListener`实例 </br>
3）调用`AliRTCEngine.CreateAliRTCEngine`函数创建AliRTCEngine实例，过程中会将`EngineEventListener`实例注册为engine的回调对象。后面只需要调用AliRTCEngine实例的各方法完成推拉流设置 </br>
4）若需要向多个房间推流，请创建多个AliRTCEngine实例，每个实例对应一个房间内的虚拟用户</br>

**Note**：Demo中基于协程管理调用和回调API，因此为避免eventLoop自锁，若有并发管理多AliRTCEngine实例的需求，建议优先使用多线程或多进程。
</br>

示例如下：
```python
# 实现EngineEventHandlerInterface类
...

# 初始化SDK
eventHandler = EngineEventListener()
coreServicePath = '/path/AliRtcCoreService' # AliRtcCoreService路径
extra_jobj = {
    "user_specified_disable_audio_ranking": "true"
}
extra = json.dumps(extra_jobj)
linuxEngine = AliRTCEngine.CreateAliRTCEngine(eventHandler, 42000, 45000, "/tmp", coreServicePath, True, extra)
```
CreateAliRTCEngine函数被调用后，将启动一个AliRtcCoreService进程，函数参数如下：
- `eventHandler:EngineEventHandlerInterface`：回调对象，负责处理回调逻辑
- `lowPort:int`：端口号下限，lowPort~highPort之间的端口将被随机选择用于进程间通信
- `highPort:int`：端口号上限
- `logPath:str`：Linux SDK运行过程中，日志文件的保存路径
- `coreServicePath:str`：AliRtcCoreService的实际路径
- `h5mode:bool`：h5兼容模式，若要与Web端互通请务必设置为True，其他场景一般为False即可
- `extra:str`：对SDK进行额外配置时传入的json格式字符串


### 2.入会
> 请先参考官网文档了解阿里云RTC鉴权过程：https://help.aliyun.com/zh/live/user-guide/token-based-authentication?spm=a2c4g.11186623.0.0.540319dfH0GWoV

入会token存在两个版本，建议使用单参数入会token，Python接口仅提供该token对应的入会API。

步骤：

1）设置用户名、房间号等基本信息，从appserver请求获取单参数token </br>
2）配置JoinChannelConfig，设置推拉流模式等参数 </br>
3）调用JoinChannel成员方法以入会 </br>
4）EngineEventListener实例的OnJoinChannelResult函数将被触发，以告知入会结果
```python
# -------- 获取入会所需基本信息 --------
authInfo = AuthInfo()
authInfo.appid = ''      # appid，业务方传入
authInfo.userid = ''     # 用户名，房间内各用户的唯一标识，若有重复将被踢出房间
authInfo.username = ''   # 用户昵称，可与userid一致，也可自行设置
authInfo.channel = ''    # 入会房间号，需要与
# ...向appserver请求，获取单参数入会token
authInfo.token = ''      # appserver响应的入会token

# -------- 配置JoinChannelConfig --------
joinConfig = JoinChannelConfig()
joinConfig.channelProfile = ChannelProfile.ChannelProfileInteractiveLive # 互动模式
joinConfig.publishAvsyncMode = PublishAvsyncMode.PublishAvsyncWithPts      # 推流音画同步模式
joinConfig.subscribeAudioFormat = AudioFormat.AudioFormatPcmBeforMixing  # 音频订阅格式
joinConfig.subscribeVideoFormat = VideoFormat.VideoFormatH264            # 视频订阅格式
joinConfig.isAudioOnly = False                                           # 仅音频模式，一般为False
joinConfig.subscribeMode = SubscribeMode.SubscribeAutomatically          # 订阅模式
joinConfig.publishMode = PublishMode.PublishAutomatically                # 推流模式

# -------- 调用JoinChannel入会 --------
linuxEngine.JoinChannel(authInfo.token, authInfo.channel, authInfo.userid, authInfo.username, joinConfig)
```

JoinChannelConfig参数说明：
1. 推流模式：支持自动和手动推流模式。前者入会后自动开启音视频推流，后者则需要手动调用API后才能进行推流。
2. 订阅模式：支持自动订阅和手动订阅模式。前者当房间内有主播入会后，便会自动订阅该主播的音视频流，后者需要手动调用API，指定要订阅的主播uid。
    
    在仅订阅音频或视频的场景下，建议使用`SubscribeAudioAutoAndOnly`或`SubscribeCameraAutoAndOnly`，以降低解码器带来的CPU占用。
3. 音频订阅格式：`AudioFormatMixedPcm`表示合流，房间内所有远端用户的音频被订阅后，会混合为一路音频数据回调；而`AudioFormatPcmBeforMixing`表示分流，每位远端用户的音频数据，将会被单独回调。
4. 视频订阅格式：设置为`VideoFormatH264`，当收到数据后，会触发`OnRemoteVideoSample`回调，否则即使收到数据也不会触发回调。
5. 音画同步模式：选择`PublishAvysncWithPts`后，在推送音视频数据时将根据API传入的timestamp进行音画同步，否则将不参考时间戳，按照实际调用的频率进行推送

**Note**：视频订阅不支持合流回调，每个uid对应单独一路视频。

注意事项：
- 在未离会时，请勿重复调用JoinChannel
- Demo中演示了用GenerateToken模拟appserver生成入会参数的过程，该接口是专为简化开发、测试过程而提供的，生产环境中，请通过与app server的交互获取token，避免appkey泄漏。

### 3.手动开启与关闭推流
以下API设置是否推送音视频流，请在发送数据前调用接口打开推流。
```python
'''
    * @brief 是否推送本地视频(摄像头)流
    * @param enabled 是否开启/关闭本地视频流推送
    - true: 开启视频流推送
    - false: 关闭视频流推送
    * @return
    - 0: 设置成功
    - <0: 设置失败，返回错误码
    * @note SDK默认设置推送视频流，在加入频道前也可以调用此接口修改默认值，并在加入频道成功时生效
'''
def PublishLocalVideoStream(enabled:bool) -> int

'''
    * @brief 是否推送本地音频流
    * @param enabled 是否开启/关闭本地音频流推送
    - true: 开启音频流推送
    - false: 关闭音频流推送
    * @return
    - 0: 设置成功
    - <0: 设置失败，返回错误码
    * @note SDK默认设置推送音频流，在加入频道前也可以调用此接口修改默认值，并在加入频道成功时生效
'''
def PublishLocalAudioStream(enabled:bool) -> int
```
若数据发送已结束，但不立即离会，建议调用上述两接口关闭推流


### 4.推送外部视频数据
```python
'''
    * @brief 启用外部视频输入源
    * @param enable true 开启, false 关闭
    * @param type 流类型
    * @note 启用后使用PushExternalVideoFrame接口输入视频数据
'''
def SetExternalVideoSource(enable:bool, sourceType:VideoSource, renderMode:RenderMode) -> int

'''
     * @brief 输入外部输视频, 暂不支持2k及以上的视频输入
     * @param frame 帧数据
     * @param type 流类型
     * @param 支持多种输入视频帧类型，如YUV、RGB24等
'''
def PushExternalVideoFrame(frame:VideoDataSample, sourceType:VideoSource) -> int
```

### 5.推送外部音频数据
不调用`SetExternalAudioPublishVolume`默认按100音量推音频
```python
'''
     * @brief 设置是否启用外部音频输入推流
     * @param enable true 开启，false 关闭
     * @param sampleRate 采样率 16k 48k...
     * @param channelsPerFrame 通道数 1 2...
     * @return >=0表示成功， <0表示失败
     * @note 可通过SetExternalAudioPublishVolume设置输入音频推流音量
'''
def SetExternalAudioSource(enable:bool, sampleRate:int, channelsPerFrame:int) -> int

'''
     * @brief 输入外部音频数据推流
     * @param audioSamples 音频数据
     * @param sampleLength 音频数据长度
     * @param timestamp 时间戳
     * @return <0表示失败，返回值为ERR_AUDIO_BUFFER_FULL时，需要在间隔投递数据时间长度后再次重试投递
'''
def PushExternalAudioFrameRawData(audioSamples:bytes, sampleLength:int, timestamp:int) -> int

'''
     * @brief 设置外部输入音频推流混音音量
     * @param vol 音量 0-100
'''
def SetExternalAudioPublishVolume(volume:int) -> int

'''
     * @brief 获取外部输入音频推流混音音量
     * @return vol 音量
'''
def GetExternalAudioPublishVolume() -> int
```

**Note**：推流过程中请关注`OnPushAudioFrameBufferFull`和`OnPushAudioFrameBufferFull`的回调内容，从而判断当前数据速度推送过快或过慢。此外若RTC进程异常退出（如被恶意kill），将触发`OnError`回调。

### 6.手动订阅
在自动订阅的模式下，无需调用这部分接口拉流 </br>
```python
'''
     * @brief 停止/恢复订阅特定远端用户的音频流, 用于会中调用, 会前调用无效
     * @param uid 用户ID，从App server分配的唯一标示符
     * @param sub 是否订阅远端用户的音频流
     * - true:订阅指定用户的音频流
     * - false:停止订阅指定用户的音频流
     * @return
     * - 0: 成功
     * - 非0: 失败
'''
def SubscribeRemoteAudioStream(uid:str, sub:bool) -> int:

'''
     * @brief 停止/恢复订阅远端用户的视频流, 用于会中调用, 会前调用无效
     * @param uid 用户ID，从App server分配的唯一标示符
     * @param track 视频流类型
     * - AliEngineVideoTrackNo: 无效参数，设置不会有任何效果
     * - AliEngineVideoTrackCamera: 相机流
     * - AliEngineVideoTrackScreen: 屏幕共享流
     * - AliEngineVideoTrackBoth: 相机流和屏幕共享流
     * @param sub 是否订阅远端用户的视频流
     * - true:订阅指定用户的视频流
     * - false:停止订阅指定用户的视频流
     * @return
     * - 0:设置成功
     * - <0:设置失败
     * @note
'''
def SubscribeRemoteVideoStream(uid:str, videoTrack:VideoTrack, sub:bool) -> int
```

### 7.数据回调
#### 7.1 音频
选择`AudioFormatPcmBeforMixing`模式后，收到音频帧将触发EventHandler实例的回调函数`OnSubscribeAudioFrame`。
- uid: 表示此时收到的音频帧来自哪个远端用户，借此区分订阅的各路音频流
- frame: 收到的音频帧，pcm格式
```python
'''
     * @brief 本地订阅音频数据回调
     * @details 远端所有用户混音后待播放的音频数据 对应AliRTCSdk::Linux::AudioFormatMixedPcm
     * @param frame 音频数据，详见{@link AliRTCSdk::Linux::AudioFrame}
'''
def OnSubscribeMixAudioFrame(self, frame:AudioFrame) -> None
```
选择AudioFormatMixedPcm模式，收到音频帧将触发EventHandler实例的回调函数`OnSubscribeMixAudioFrame`。
```python
'''
     * @brief 本地订阅音频数据回调
     * @details 远端所有用户混音后待播放的音频数据 对应AliRTCSdk::Linux::AudioFormatMixedPcm
     * @param frame 音频数据，详见{@link AliRTCSdk::Linux::AudioFrame}
'''
def OnSubscribeMixAudioFrame(self, frame:AudioFrame) -> None
```

#### 7.2 视频
收到视频帧时将触发EventHandler实例的回调函数`OnRemoteVideoSample`。
- uid: 表示此时收到的视频帧来自哪个远端用户，借此区分订阅的各路视频流
- frame: 收到的视频帧，yuv I420格式
```python
'''
     * @brief 订阅的远端视频数据回调
     * @param uid 用户ID
     * @param frame 视频裸数据
     * @return
'''
def OnRemoteVideoSample(self, uid:str, frame:VideoFrame) -> None
```

### 8.离会
```python
# 结束推流
linuxEngine.PublishLocalVideoStream(False)
linuxEngine.PublishLocalAudioStream(False)
```
若要在停止推流后，仍在会中，请手动调用此接口；直接离会也具有停止推流的效果

```python
linuxEngine.LeaveChannel()
```

### 9.销毁SDK

```python
linuxEngine.Release()
linuxEngine = None
```

# 三、Demo使用方式
demo.py中替换您的appid，并正确指定AliRtcCoreService的路径后，在Demo目录执行`python3 demo.py`

若终端有以下输出，表示入会、推流成功
```bash
# 入会成功
[Python] on join channel result. Channel: 12301, user: linux, result: 0

# 推流成功
[Python] on audio publish state changed, oldState: 0, newState: 2
[Python] on video publish state changed, oldState: 0, newState: 2
[Python] on audio publish state changed, oldState: 2, newState: 3
[Python] on video publish state changed, oldState: 2, newState: 3

# 远端用户web已上线
[Python] on remote user online: web

# 收到来自远端用户abcd的音视频流
[Python] on audio subscribe state of web, oldState: 0, newState: 2
[Python] on video subscribe state of web, oldState: 0, newState: 2
[Python] on audio subscribe state of web, oldState: 2, newState: 3
[Python] on video subscribe state of web, oldState: 2, newState: 3
```

音频订阅选择AudioFormatMixedPcm模式时，无论是否有远端用户在会上，入会时就会有音频回调。选择AudioFormatPcmBeforMixing时，只有会上有其他用户，且其他用户在推音频流时，才会触发音频回调。

# 四、消息通讯
除音视频互动外，RTC SDK还支持实时消息收发，以用于有消息互动需求的场景。您可以通过Data Channel和SEI（Supplemental Enhancement Information）两种通道进行消息收发，前者独立于音视频传输通道，后者依赖视频数据收发，一般场景下建议使用Data Channel。
### 1. Data Channel
请调用SetParameter以开启Data Channel：`linuxEngine.SetParameter("{\"data\":{\"enablePubDataChannel\":true,\"enableSubDataChannel\":true}}")`
通过`SendDataChannelMessage`发送消息：
```python
dataChannelMsg = AliEngineDataChannelMsg()
message = "This is data channel message".encode('utf-8')
dataChannelMsg.data = message
dataChannelMsg.dataLen = len(message)
dataChannelMsg.networkTime = t0; // 网络时间字段，也可以自定义取值，不影响消息收发
dataChannelMsg.progress = 0; // 保留字段
dataChannelMsg.type = AliEngineDataMsgType.AliEngineDataChannelCustom

linuxEngine.SendDataChannelMessage(dataChannelMsg)
```
当接收到Data Channel消息后，会触发回调：
```python
'''
     * @brief 获得dataChannel远端数据
     * @param msg 远端传来的消息
'''
def OnDataChannelMsg(self, uid:str, msg:AliEngineDataChannelMsg) -> None
```

### 2. SEI
```C++
/**
 * @brief 发送 媒体扩展信息(SEI)， 最大长度为4*1024字节，用于业务的少量数据传输
 * @param message 扩展信息内容，可以传递4K Bytes数据
 * @param length 扩展信息长度，单位:字节
 * @param repeatCount 重复次数，用于防止网络丢包导致的消息丢失
 * @param delay 延迟多久发出去，单位:毫秒
 * @param isKeyFrame 是否只在关键帧上增加SEI
 * @return <0: 成功，-1: SDK内部错误>
*/
int SendMediaExtensionMsg(const char* message,
                          size_t length,
                          int repeatCount,
                          uint32_t delay,
                          bool isKeyFrame);
```
通过`SendMediaExtensionMsg`发送消息：
```python
seiMsg = "This is SEI message".encode('utf-8')
linuxEngine.SendMediaExtensionMsg(seiMsg, len(seiMsg), 3, 0 , False)
```
由于关键帧间隔较长，若需要高频收发SEI消息，建议isKeyFrame设置为false。同时为保证消息有效送达，建议重复次数大于1。

当接收到SEI消息后，会触发回调：
```python
'''
     * @brief 收到媒体扩展信息回调
     * @param uid 发送用户userId
     * @param message 扩展信息内容
     * @param size 扩展信息长度
     * @note 当一端通过 {@link SendMediaExtensionMsg} 发送信息后，其他端通过该回调接收数据
'''
def OnMediaExtensionMsgReceived(self, userid:str, message:bytes, size:int) -> None:
```

# 五、其他功能
创建引擎时的extra字段，可用于开闭部分额外功能。extra字段为json格式。

### 1. 关闭Audio Ranking
Audio Ranking是多人语聊房中，仅订阅音量较大几路音频流的功能，使听众听到的声音更为清晰，默认为开启状态。若场景需要订阅全部音频流，需要通过如下字段关闭Audio Ranking功能：
```python
{"user_specified_disable_audio_ranking" : "true"}
```

### 2. 开启订阅音频时的转码AAC功能
对于有拉流存档需求的场景，SDK提供将订阅到的远端音频转码至AAC编码的能力。您可通过AudioTranscodingCodec中的各项选择以何种格式获取远端音频（PCM、AAC或二者皆要），若开启AAC转码功能，还可选择是否进行重采样以降低CPU和存储占用。当user_specified_audio_observer_resample_rate设置为0表示不进行重采样。
```python
{"user_specified_audio_observer_codec" : 
    AudioTranscodingCodec.AudioTranscodingCodecBothPcmAndAac.value}
{"user_specified_audio_observer_codec_format" : 0} // 目前只支持adts格式
{"user_specified_audio_observer_resample_rate" : 16000} // 重采样率
{"user_specified_audio_observer_codec_bitrate" : 64000} // 转码的目标码率
```

### 3. 开启订阅视频时直接获取H264码流功能
对于有拉流存档需求的场景，SDK提供直接获取远端视频H264码流的能力。您可通过VideoTranscodingCodec的各项，选择以何种格式获取远端视频（YUV、H264码流或二者皆要）。
```python
{"user_specified_video_observer_codec" :
    VideoTranscodingCodec.VideoTranscodingCodecBothYuvAndH264.value}
{"user_specified_video_observer_codec_format" : 0} // 目前只支持annexb格式
```
