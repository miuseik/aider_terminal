"""
统一的音频驱动

基于 sounddevice (PortAudio) 同时支持输入（麦克风）和输出（扬声器），
提供与 OpenCVCameraDriver 一致的 connect/read/disconnect 生命周期。
"""

import logging
import threading
from typing import Optional

import numpy as np
import sounddevice as sd

logger = logging.getLogger(__name__)

MODE_INPUT = "input"
MODE_OUTPUT = "output"


class AudioDriver:
    """
    SoundDevice 统一音频驱动

    mode="input"   → sd.InputStream  + read(num_frames)
    mode="output"  → sd.OutputStream + write(data)
    """

    def __init__(self, config: dict, mode: str = MODE_INPUT):
        """
        Args:
            config: 配置字典，包含：
                - device: 设备名称或索引 (None = 系统默认)
                - sample_rate: 采样率 Hz (默认 16000)
                - channels: 声道数 (默认 1)
            mode: "input" 或 "output"
        """
        self.mode = mode
        self.device = config.get("device", None)
        self.sample_rate = config.get("sample_rate", 16000)
        self.channels = config.get("channels", 1)
        self.dtype = "int16"

        self._stream = None  # sd.InputStream 或 sd.OutputStream
        self._lock = threading.Lock()
        self.is_connected = False

        label = "输入" if self.mode == MODE_INPUT else "输出"
        print(
            f"AudioDriver ({label}) 初始化: {self.sample_rate}Hz, "
            f"channels={self.channels}, device={self.device}"
        )

    # ── 生命周期 ──

    def connect(self) -> bool:
        """打开音频流（输入或输出）。"""
        try:
            if self.mode == MODE_INPUT:
                self._stream = sd.InputStream(
                    device=self.device,
                    channels=self.channels,
                    samplerate=self.sample_rate,
                    dtype=self.dtype,
                )
            else:
                # 输出模式：增大缓冲避免网络抖动导致的 underrun
                blocksize = int(self.sample_rate * 0.02)  # 20ms
                self._stream = sd.OutputStream(
                    device=self.device,
                    channels=self.channels,
                    samplerate=self.sample_rate,
                    dtype=self.dtype,
                    blocksize=blocksize,
                    latency="high",
                )
            self._stream.start()
            self.is_connected = True
            label = "麦克风" if self.mode == MODE_INPUT else "扬声器"
            print(f"{label}已连接: {self.sample_rate}Hz mono")
            logger.info("%s已连接: %dHz mono", label, self.sample_rate)
            return True
        except Exception:
            label = "麦克风" if self.mode == MODE_INPUT else "扬声器"
            logger.exception("%s连接失败", label)
            self.is_connected = False
            return False

    def disconnect(self):
        """关闭音频流。"""
        with self._lock:
            if self._stream is not None:
                try:
                    self._stream.stop()
                    self._stream.close()
                except Exception:
                    logger.exception("释放音频资源失败")
                finally:
                    self._stream = None
                    self.is_connected = False
                    label = "麦克风" if self.mode == MODE_INPUT else "扬声器"
                    print(f"{label}已断开")
                    logger.info("%s已断开", label)

    # ── 输入：读取麦克风数据 ──

    def read(self, num_frames: int) -> Optional[np.ndarray]:
        """
        阻塞读取 num_frames 帧采样点（仅 input 模式）。

        Returns:
            int16 numpy 数组 shape=(num_frames,)，失败返回 None
        """
        if self.mode != MODE_INPUT:
            logger.error("read() 仅支持 input 模式")
            return None
        if not self.is_connected or self._stream is None:
            return None
        try:
            data, _overflowed = self._stream.read(num_frames)
            if data.ndim == 2 and data.shape[1] == 1:
                data = data.flatten()
            return data.astype(np.int16, copy=False)
        except Exception:
            logger.exception("读取麦克风异常")
            return None

    # ── 输出：写入扬声器数据 ──

    def write(self, data: np.ndarray) -> bool:
        """
        写入音频数据到输出流（仅 output 模式）。

        Args:
            data: int16 numpy 数组 shape=(num_frames,) 或 (num_frames, channels)

        Returns:
            成功返回 True
        """
        if self.mode != MODE_OUTPUT:
            logger.error("write() 仅支持 output 模式")
            return False
        if not self.is_connected or self._stream is None:
            return False
        try:
            with self._lock:
                # OutputStream.write 期望 (frames, channels)
                if data.ndim == 1:
                    data = data.reshape(-1, 1)
                elif data.shape[1] != self.channels:
                    # planar 格式检测 (channels, samples) → (samples, channels)
                    if data.shape[1] > 8 and data.shape[0] <= 8:
                        logger.debug("检测到 planar 格式，自动转置: %s", data.shape)
                        data = data.T.copy()
                    if data.shape[1] != self.channels:
                        logger.warning(
                            "通道数不匹配: 数据=%dch, 流=%dch, 已自动转换",
                            data.shape[1], self.channels,
                        )
                        if data.shape[1] > self.channels:
                            data = data.mean(axis=1, keepdims=True).astype(data.dtype)
                        else:
                            data = np.repeat(data, self.channels, axis=1)
                self._stream.write(data.astype(np.int16, copy=False))
            return True
        except Exception:
            logger.exception("扬声器写入异常")
            return False


# ── 向后兼容别名 ──

class MicrophoneDriver(AudioDriver):
    """向后兼容：麦克风驱动 (AudioDriver mode="input")"""

    def __init__(self, config: dict):
        super().__init__(config, mode=MODE_INPUT)


class SpeakerDriver(AudioDriver):
    """向后兼容：扬声器驱动 (AudioDriver mode="output")"""

    def __init__(self, config: dict):
        super().__init__(config, mode=MODE_OUTPUT)
