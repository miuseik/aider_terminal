"""
OpenCV 摄像头驱动
基于 LeRobot 的 OpenCVCamera 实现，提供稳定可靠的摄像头访问
"""

import os
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
os.environ.setdefault('OPENCV_VIDEOIO_LOG_LEVEL', 'ERROR')

import cv2
import threading
import time
import logging
import platform
from pathlib import Path
from typing import Optional, Tuple
import numpy as np

logger = logging.getLogger(__name__)


class OpenCVCameraDriver:
    """
    OpenCV 摄像头驱动

    支持：
    - 自动检测可用摄像头
    - 配置分辨率、帧率、FOURCC 编码
    - 同步/异步读取帧
    - 图像后处理（旋转、颜色转换）
    """

    # Linux 下最大设备索引
    MAX_OPENCV_INDEX = 60

    def __init__(self, config: dict):
        """
        初始化摄像头驱动

        Args:
            config: 配置字典，包含：
                - index_or_path: 摄像头索引或设备路径 (默认: 0)
                - width: 分辨率宽度 (默认: 640)
                - height: 分辨率高度 (默认: 480)
                - fps: 帧率 (默认: 30)
                - fourcc: 编码格式，如 "MJPG", "YUYV" (可选)
                - color_mode: "rgb" 或 "bgr" (默认: "rgb")
                - rotation: 旋转角度 0/90/180/270 (默认: 0)
                - warmup_s: 预热时间秒数 (默认: 0.5)
        """
        self.index_or_path = config.get('index_or_path', 0)
        self.width = config.get('width', 640)
        self.height = config.get('height', 480)
        self.fps = config.get('fps', 30)
        self.fourcc = config.get('fourcc', None)
        self.color_mode = config.get('color_mode', 'rgb')
        self.rotation = config.get('rotation', 0)
        self.warmup_s = config.get('warmup_s', 0.5)

        self.videocapture: Optional[cv2.VideoCapture] = None
        self.is_connected = False
        self._read_lock = threading.Lock()  # 保护 V4L2 ioctl 调用，防止多线程竞态导致 SIGSEGV

        # 旋转映射
        self.rotation_map = {
            0: None,
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE
        }

        # print(f"📷 OpenCVCameraDriver 初始化: {self.index_or_path}")

    @staticmethod
    def find_cameras() -> list[dict]:
        """
        检测系统中所有可用的摄像头

        Returns:
            摄像头信息列表，每个元素包含：
            - id: 设备索引或路径
            - name: 设备名称
            - default_width: 默认宽度
            - default_height: 默认高度
            - default_fps: 默认帧率
            - backend: 后端名称
        """
        found_cameras = []

        # 确定要扫描的目标
        if platform.system() == "Linux":
            possible_paths = sorted(Path("/dev").glob("video*"), key=lambda p: p.name)
            targets = [str(p) for p in possible_paths]
        else:
            targets = list(range(OpenCVCameraDriver.MAX_OPENCV_INDEX))

        for target in targets:
            try:
                cap = cv2.VideoCapture(target)
                if cap.isOpened():
                    # 获取默认参数
                    default_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    default_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    default_fps = cap.get(cv2.CAP_PROP_FPS)

                    # 获取 FOURCC
                    fourcc_code = int(cap.get(cv2.CAP_PROP_FOURCC))
                    fourcc = "".join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

                    camera_info = {
                        'id': target,
                        'name': f"Camera @ {target}",
                        'default_width': default_width,
                        'default_height': default_height,
                        'default_fps': default_fps,
                        'default_fourcc': fourcc,
                        'backend': cap.getBackendName()
                    }

                    found_cameras.append(camera_info)
                    # print(f"发现摄像头: {camera_info}")
                    cap.release()
            except Exception as e:
                # print(f"检测设备 {target} 失败: {e}")
                continue

        # print(f"✅ 检测到 {len(found_cameras)} 个摄像头")
        return found_cameras

    def connect(self) -> bool:
        """
        连接到摄像头

        Returns:
            是否成功连接
        """
        try:
            # 设置 OpenCV 线程数为 1，避免多线程冲突
            cv2.setNumThreads(1)

            # 打开摄像头（显式指定 V4L2 后端，避免后端选择不一致）
            self.videocapture = cv2.VideoCapture(self.index_or_path, cv2.CAP_V4L2)

            if not self.videocapture.isOpened():
                # print(f"❌ 无法打开摄像头: {self.index_or_path}")
                return False

            # 配置摄像头参数
            self._configure_settings()

            # 预热（可选）
            if self.warmup_s > 0:
                self._warmup()

            self.is_connected = True
            print(f"✅ 摄像头已连接: {self.index_or_path} @ {self.width}x{self.height}@{self.fps}fps")
            return True

        except Exception as e:
            print(f"❌ 连接摄像头失败: {e}")
            self.disconnect()
            return False

    def _configure_settings(self):
        """配置摄像头参数"""
        if self.videocapture is None:
            raise RuntimeError("摄像头未初始化")

        # 设置 FOURCC（如果指定）
        if self.fourcc:
            fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc)
            success = self.videocapture.set(cv2.CAP_PROP_FOURCC, fourcc_code)
            if success:
                print(f"设置 FOURCC: {self.fourcc}")
            else:
                print(f"设置 FOURCC {self.fourcc} 失败，使用默认格式")

        # 设置分辨率
        self.videocapture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.videocapture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))

        # 验证分辨率
        actual_width = int(self.videocapture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.videocapture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if actual_width != self.width or actual_height != self.height:
            print(
                f"分辨率不匹配: 请求 {self.width}x{self.height}, "
                f"实际 {actual_width}x{actual_height}"
            )
            self.width = actual_width
            self.height = actual_height

        # 设置帧率
        if self.fps:
            self.videocapture.set(cv2.CAP_PROP_FPS, float(self.fps))
            actual_fps = self.videocapture.get(cv2.CAP_PROP_FPS)
            print(f"帧率: 请求 {self.fps}, 实际 {actual_fps}")

    def _warmup(self):
        """预热摄像头，丢弃初始几帧"""
        print(f"预热摄像头 {self.warmup_s} 秒...")
        start_time = time.time()
        frame_count = 0

        while time.time() - start_time < self.warmup_s:
            with self._read_lock:
                ret, _ = self.videocapture.read()
            if ret:
                frame_count += 1
            time.sleep(0.01)

        print(f"预热完成，捕获 {frame_count} 帧")

    def read(self) -> Optional[np.ndarray]:
        """
        同步读取一帧（阻塞，线程安全）

        Returns:
            图像数组 (H, W, C)，颜色模式根据配置转换。
            帧数据已从 V4L2 缓冲区深拷贝，调用方可安全持有。
            失败返回 None。
        """
        if not self.is_connected or self.videocapture is None:
            return None

        try:
            with self._read_lock:
                ret, frame = self.videocapture.read()
            if not ret or frame is None:
                return None
            # ⚠️ 必须深拷贝：OpenCV V4L2 后端返回的 ndarray 可能引用 mmap 缓冲区，
            #    不拷贝的话 VideoFrame.from_ndarray 编码时缓冲区被覆盖会 SIGSEGV
            frame = frame.copy()
            return self._postprocess_frame(frame)
        except Exception:
            logger.exception("读取帧异常")
            return None

    def read_latest(self) -> Optional[np.ndarray]:
        """
        读取最新帧（跳过积压帧，拿到最新的一帧）

        V4L2 默认只有 4 个缓冲区，用 grab()+retrieve() 代替 read()
        避免过度 DQBUF 触发 ioctl(VIDIOC_DQBUF): Invalid argument。

        Returns:
            图像数组（深拷贝），失败返回 None
        """
        if not self.is_connected or self.videocapture is None:
            return None
        try:
            with self._read_lock:
                # grab() 只做 DQBUF→QBUF，不解码，比 read() 轻量
                for _ in range(4):  # 最多跳过 4 帧（V4L2 默认 BUF 数）
                    if not self.videocapture.grab():
                        break
                ret, frame = self.videocapture.retrieve()
            if not ret or frame is None:
                return None
            frame = frame.copy()
            return self._postprocess_frame(frame)
        except Exception:
            logger.exception("read_latest 异常")
            return None

    def _postprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        后处理帧：颜色转换和旋转

        Args:
            frame: 原始帧（BGR 格式）

        Returns:
            处理后的帧
        """
        # 颜色空间转换
        if self.color_mode == 'rgb':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 旋转
        rotation_angle = self.rotation_map.get(self.rotation)
        if rotation_angle is not None:
            frame = cv2.rotate(frame, rotation_angle)

        return frame

    def disconnect(self):
        """断开摄像头连接"""
        if self.videocapture is not None:
            try:
                self.videocapture.release()
            except Exception as e:
                print(f"释放摄像头资源失败: {e}")
            finally:
                self.videocapture = None
                self.is_connected = False
                # print("🔌 摄像头已断开")

    def get_info(self) -> dict:
        """
        获取摄像头当前信息

        Returns:
            摄像头信息字典
        """
        if not self.is_connected or self.videocapture is None:
            return {'connected': False}

        return {
            'connected': True,
            'index_or_path': self.index_or_path,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'fourcc': self.fourcc,
            'color_mode': self.color_mode,
            'rotation': self.rotation
        }

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()
        return False

    def __del__(self):
        """析构函数"""
        self.disconnect()
