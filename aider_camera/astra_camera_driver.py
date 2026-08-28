"""
Astra RGB-D 相机驱动 (基于 pyorbbecsdk2 / Orbbec SDK v2 开源分支)

支持奥比中光 Astra Mini Pro 等 RGB-D 相机，输出同步的彩色帧 + 深度帧。
驱动遵循 aider_camera 包的约定生命周期接口：connect() / read() / disconnect()，
零 ROS 依赖，可被 terminal 的 sensors 采集层直接调用。

依赖：
    pip install --upgrade pyorbbecsdk2 opencv-python numpy
    # Linux 上还需配置 udev 规则（一次）：
    # python3 $(python3 -c "import pyorbbecsdk, os; print(os.path.dirname(pyorbbecsdk.__file__))")/shared/setup_env.py

基本用法：
    from aider_camera import AstraCameraDriver
    cam = AstraCameraDriver({"color_w": 640, "color_h": 480, "fps": 30,
                             "align": True, "depth_to_color": True})
    cam.connect()
    frame = cam.read()          # -> dict | None
    cam.disconnect()

read() 返回的 dict 字段：
    color:      np.ndarray (H, W, 3) uint8, BGR (OpenCV 习惯) 或 RGB (color_mode="rgb")
    depth:      np.ndarray (H, W)    uint16, 单位 mm (0 = 无效)
    depth_scale: float, 原始深度值 -> 米 的换算系数
    color_frame / depth_frame: 原始 VideoFrame 对象 (可选, 默认不返回)
    timestamp:  float 秒
"""

import os
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
os.environ.setdefault('OPENCV_VIDEOIO_LOG_LEVEL', 'ERROR')

import time
import logging
from typing import Optional

import cv2
import numpy as np

try:
    from pyorbbecsdk import (
        Pipeline,
        Config,
        OBError,
        OBFormat,
        OBSensorType,
        OBAlignMode,
        VideoFrame,
    )
except Exception as exc:  # pragma: no cover - 运行环境可能没有装 SDK
    Pipeline = Config = OBError = OBFormat = OBSensorType = OBAlignMode = VideoFrame = None
    _IMPORT_ERROR = exc


logger = logging.getLogger(__name__)


class AstraCameraDriver:
    """
    Orbbec Astra RGB-D 相机驱动。

    设计要点：
    - 用 Pipeline + Config 显式开启 color / depth 流（不依赖默认 XML 配置，
      便于在代码里固定分辨率/帧率，跨设备一致）。
    - 可选硬件/软件对齐（深度图对齐到彩色图分辨率）。
    - read() 走阻塞式 wait_for_frames()，线程安全由调用方保证（单消费者模型，
      与现有 control_loop 每帧读一帧的节奏一致）。
    - 深度值统一转成 mm（uint16），无效点保持 0。
    """

    def __init__(self, config: Optional[dict] = None):
        """
        Args:
            config: 配置字典，支持以下键（均有默认值）：
                color_w / color_h: 彩色流分辨率 (默认 640x480)
                depth_w / depth_h: 深度流分辨率 (默认 640x480)
                fps:               帧率 (默认 30)
                color_format:      "RGB" | "BGR" | "YUYV" | "MJPG" (默认 RGB)
                depth_format:      "Y16" (默认，16bit 深度)
                align:             bool, 是否把深度对齐到彩色 (默认 True)
                depth_to_color:    bool, 对齐目标用彩色图 (True) 还是深度图 (False)
                color_mode:        "rgb" | "bgr"，read() 返回的 color 通道顺序 (默认 "bgr")
                return_raw:        bool, 是否一并返回原始 VideoFrame (默认 False)
        """
        cfg = config or {}
        self.color_w = int(cfg.get('color_w', 640))
        self.color_h = int(cfg.get('color_h', 480))
        self.depth_w = int(cfg.get('depth_w', 640))
        self.depth_h = int(cfg.get('depth_h', 480))
        self.fps = int(cfg.get('fps', 30))
        self.color_format = cfg.get('color_format', 'RGB')
        self.depth_format = cfg.get('depth_format', 'Y16')
        self.align = bool(cfg.get('align', True))
        self.depth_to_color = bool(cfg.get('depth_to_color', True))
        self.color_mode = cfg.get('color_mode', 'bgr').lower()
        self.return_raw = bool(cfg.get('return_raw', False))

        self.pipeline: Optional['Pipeline'] = None
        self.config: Optional['Config'] = None
        self._depth_scale = 1.0  # 原始深度单位 -> 米
        self._align_mode = None
        self.is_connected = False

        if 'pyorbbecsdk' not in globals() or Pipeline is None:
            logger.warning(
                "pyorbbecsdk 未安装，AstraCameraDriver 不可用：%s",
                globals().get('_IMPORT_ERROR', 'unknown'),
            )

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def connect(self) -> bool:
        """初始化并启动相机流水线。成功返回 True。"""
        if Pipeline is None:
            logger.error("pyorbbecsdk 未安装，无法连接 Astra 相机")
            return False
        try:
            self.pipeline = Pipeline()
            self.config = Config()

            # 解析格式枚举
            color_fmt = getattr(OBFormat, self.color_format, OBFormat.RGB)
            depth_fmt = OBFormat.Y16 if self.depth_format.upper() == 'Y16' else \
                getattr(OBFormat, self.depth_format, OBFormat.Y16)

            # 开启彩色流
            self.config.enable_stream(OBSensorType.COLOR_SENSOR,
                                      self.color_w, self.color_h,
                                      color_fmt, self.fps)
            # 开启深度流
            self.config.enable_stream(OBSensorType.DEPTH_SENSOR,
                                      self.depth_w, self.depth_h,
                                      depth_fmt, self.fps)

            # 对齐：把深度图配准到彩色图，方便后续像素级操作
            if self.align:
                mode = OBAlignMode.ALIGN_D2C if self.depth_to_color else OBAlignMode.ALIGN_C2D
                self._align_mode = mode
                self.config.set_align_mode(mode)

            self.pipeline.start(self.config)

            # 读取深度换算系数（SDK 返回的是“原始值 -> 米”）
            self._depth_scale = self._query_depth_scale()
            self.is_connected = True
            logger.info(
                "Astra 相机已连接: color=%dx%d depth=%dx%d@%dfps align=%s",
                self.color_w, self.color_h, self.depth_w, self.depth_h,
                self.fps, self.align,
            )
            return True

        except OBError as e:
            logger.error("Astra 相机启动失败: %s", e)
            self.disconnect()
            return False
        except Exception as e:  # noqa: BLE001
            logger.exception("Astra 相机连接异常: %s", e)
            self.disconnect()
            return False

    def _query_depth_scale(self) -> float:
        """尝试从设备拿到深度单位换算系数（米/原始值），失败回退 0.001。"""
        try:
            dev = self.pipeline.get_device()
            prof = dev.get_depth_scale()
            if prof:
                return float(prof)
        except Exception:  # noqa: BLE001
            pass
        return 0.001

    def disconnect(self):
        """停止流水线并释放资源。"""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception as e:  # noqa: BLE001
                logger.warning("Astra 停止流水线异常: %s", e)
            finally:
                self.pipeline = None
                self.config = None
                self.is_connected = False

    # ------------------------------------------------------------------ #
    # 帧读取
    # ------------------------------------------------------------------ #
    def read(self, timeout_ms: int = 1000) -> Optional[dict]:
        """
        阻塞读取一帧同步的 (彩色 + 深度)。

        Args:
            timeout_ms: 等待帧的超时 (毫秒)

        Returns:
            dict (见模块 docstring) 或 None（超时/出错）
        """
        if not self.is_connected or self.pipeline is None:
            return None
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms)
            if frames is None:
                return None

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                return None

            result: dict = {
                'timestamp': time.time(),
                'color': self._color_to_image(color_frame),
                'depth': self._depth_to_mm(depth_frame),
                'depth_scale': self._depth_scale,
            }
            if self.return_raw:
                result['color_frame'] = color_frame
                result['depth_frame'] = depth_frame
            return result

        except OBError:
            # 超时或设备拔出，返回 None 让上层决定重试/重连
            return None
        except Exception:  # noqa: BLE001
            logger.exception("Astra 读取帧异常")
            return None

    def _color_to_image(self, frame: 'VideoFrame') -> np.ndarray:
        """把彩色 VideoFrame 转成 np.ndarray (H, W, 3) uint8。"""
        width = frame.get_width()
        height = frame.get_height()
        fmt = frame.get_format()
        data = np.asanyarray(frame.get_data())

        if fmt == OBFormat.RGB:
            img = data.reshape((height, width, 3)).copy()
            if self.color_mode == 'bgr':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif fmt == OBFormat.BGR:
            img = data.reshape((height, width, 3)).copy()
            if self.color_mode == 'rgb':
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif fmt == OBFormat.MJPG:
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if self.color_mode == 'rgb' and img is not None:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif fmt == OBFormat.YUYV:
            img = data.reshape((height, width, 2))
            img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
            if self.color_mode == 'rgb':
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            # 兜底用 SDK 自带转换
            img = self._format_convert_to_bgr(frame)
            if self.color_mode == 'rgb' and img is not None:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def _format_convert_to_bgr(self, frame: 'VideoFrame') -> Optional[np.ndarray]:
        """对其他格式（NV12/NV21/I420 等）用 FormatConvertFilter 转 BGR。"""
        try:
            from pyorbbecsdk import FormatConvertFilter, OBConvertFormat
            mapping = {
                OBFormat.I420: OBConvertFormat.I420_TO_RGB888,
                OBFormat.MJPG: OBConvertFormat.MJPG_TO_RGB888,
                OBFormat.YUYV: OBConvertFormat.YUYV_TO_RGB888,
                OBFormat.NV21: OBConvertFormat.NV21_TO_RGB888,
                OBFormat.NV12: OBConvertFormat.NV12_TO_RGB888,
                OBFormat.UYVY: OBConvertFormat.UYVY_TO_RGB888,
            }
            conv = mapping.get(frame.get_format())
            if conv is None:
                return None
            filt = FormatConvertFilter()
            filt.set_format_convert_format(conv)
            rgb = filt.process(frame)
            if rgb is None:
                return None
            w, h = rgb.get_width(), rgb.get_height()
            arr = np.asanyarray(rgb.get_data()).reshape((h, w, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception:  # noqa: BLE001
            logger.exception("格式转换失败")
            return None

    def _depth_to_mm(self, frame: 'VideoFrame') -> np.ndarray:
        """
        把深度 VideoFrame 转成 mm (uint16) 的 2D 数组。
        无效点保持 0。
        """
        width = frame.get_width()
        height = frame.get_height()
        raw = np.frombuffer(frame.get_data(), dtype=np.uint16)
        if raw.size != width * height:
            raw = raw.reshape((height, width))
        else:
            raw = raw.reshape((height, width))
        # 原始值 (通常是 mm 或 0.001m 单位) -> mm
        depth_mm = raw.astype(np.float32) * (self._depth_scale * 1000.0)
        return depth_mm.astype(np.uint16)

    # ------------------------------------------------------------------ #
    # 工具
    # ------------------------------------------------------------------ #
    def get_info(self) -> dict:
        """返回当前相机配置信息。"""
        return {
            'connected': self.is_connected,
            'color_w': self.color_w,
            'color_h': self.color_h,
            'depth_w': self.depth_w,
            'depth_h': self.depth_h,
            'fps': self.fps,
            'align': self.align,
            'color_mode': self.color_mode,
            'depth_scale': self._depth_scale,
        }

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    def __del__(self):
        self.disconnect()
