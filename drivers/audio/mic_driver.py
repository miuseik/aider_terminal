"""
SoundDevice 麦克风驱动

基于 sounddevice (PortAudio) 采集麦克风音频，提供与 OpenCVCameraDriver
一致的 connect/read/disconnect 接口，供 MicrophoneAudioTrack 调用。
"""

import logging
from typing import Optional

import numpy as np
import sounddevice as sd

logger = logging.getLogger(__name__)


class MicrophoneDriver:
    """
    SoundDevice 麦克风驱动

    支持：
    - 自动选择默认输入设备
    - 指定采样率 / 声道数
    - 阻塞式 read(n) 读取固定帧数
    - connect / disconnect 生命周期管理
    """

    def __init__(self, config: dict):
        """
        Args:
            config: 配置字典，包含：
                - device: 设备名称或索引 (None = 系统默认)
                - sample_rate: 采样率 Hz (默认 16000)
                - channels: 声道数 (默认 1)
        """
        self.device = config.get("device", None)
        self.sample_rate = config.get("sample_rate", 16000)
        self.channels = config.get("channels", 1)
        self.dtype = "int16"

        self._stream: Optional[sd.InputStream] = None
        self.is_connected = False

        print(
            f"MicDriver 初始化: {self.sample_rate}Hz, "
            f"channels={self.channels}, device={self.device}"
        )

    def connect(self) -> bool:
        """打开麦克风输入流。"""
        try:
            self._stream = sd.InputStream(
                device=self.device,
                channels=self.channels,
                samplerate=self.sample_rate,
                dtype=self.dtype,
            )
            self._stream.start()
            self.is_connected = True
            print(f"麦克风已连接: {self.sample_rate}Hz mono")
            logger.info("麦克风已连接: %dHz mono", self.sample_rate)
            return True
        except Exception:
            logger.exception("麦克风连接失败")
            self.is_connected = False
            return False

    def read(self, num_frames: int) -> Optional[np.ndarray]:
        """
        阻塞读取 num_frames 帧采样点。

        Args:
            num_frames: 要读取的采样点数

        Returns:
            int16 numpy 数组 shape=(num_frames,)，失败返回 None
        """
        if not self.is_connected or self._stream is None:
            return None
        try:
            data, _overflowed = self._stream.read(num_frames)
            # sd.InputStream.read 返回 (frames, channels)，压平成 1D
            if data.ndim == 2 and data.shape[1] == 1:
                data = data.flatten()
            return data.astype(np.int16, copy=False)
        except Exception:
            logger.exception("读取麦克风异常")
            return None

    def disconnect(self):
        """关闭麦克风流。"""
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                logger.exception("释放麦克风资源失败")
            finally:
                self._stream = None
                self.is_connected = False
                print("麦克风已断开")
                logger.info("麦克风已断开")
