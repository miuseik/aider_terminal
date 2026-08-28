"""Hiwonder 十轴 AHRS 串口驱动。

协议（实测确认，波特率 460800 而非常见的 9600）:
    帧结构: 0x55 | type | 8字节数据 | 校验(前10字节和 & 0xFF)
    type:
        0x51 加速度  量程 ±16g     -> g = raw/32768*16
        0x52 角速度  量程 ±2000°/s -> °/s = raw/32768*2000
        0x53 姿态角  量程 ±180°    -> ° = raw/32768*180
    数据为小端有符号 16 位补码

注意: 部分模块仅输出 0x52(角速度) 与 0x53(姿态角)，无 0x51(加速度)。
      此时加速度由姿态角反推重力方向得到（见 ImuHiwonderNode）。

实测设备: USB 串口 CH340 (1a86:7523)，默认 /dev/ttyUSB0
"""

import math
import threading
import time

import serial

# 帧类型
FRAME_ACC = 0x51
FRAME_GYRO = 0x52
FRAME_ANGLE = 0x53

# 量程系数
SCALE_ACC_G = 16.0        # ±16 g
SCALE_GYRO_DPS = 2000.0   # ±2000 °/s
SCALE_ANGLE_DEG = 180.0   # ±180 °

FRAME_LEN = 11


def _s16(lo: int, hi: int) -> int:
    """两个字节 -> 有符号 16 位补码。"""
    v = lo | (hi << 8)
    return v - 65536 if v >= 32768 else v


def euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """欧拉角(度, ZYX 顺序) -> 四元数 (x, y, z, w)。"""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class HiwonderImu:
    """Hiwonder 串口 IMU 读取器（后台线程持续收帧）。"""

    def __init__(self, port="/dev/ttyUSB0", baud=460800):
        self._port = port
        self._baud = baud
        self._ser = None
        self._thread = None
        self._running = False

        # 最新数据（原始值），由后台线程写入
        self._lock = threading.Lock()
        self._acc_raw = None
        self._gyro_raw = None
        self._angle_raw = None
        self._frames = 0
        self._last_time = 0.0

    def connect(self):
        self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frames

    def _read_loop(self):
        """后台收帧循环：按 0x55 同步，校验通过后更新最新值。"""
        buf = b""
        while self._running and self._ser and self._ser.is_open:
            try:
                chunk = self._ser.read(self._ser.in_waiting or 128)
                if not chunk:
                    time.sleep(0.001)
                    continue

                buf += chunk
                while len(buf) >= FRAME_LEN:
                    idx = buf.find(b"\x55")
                    if idx < 0:
                        buf = b""
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < FRAME_LEN:
                        break

                    frame = buf[:FRAME_LEN]
                    if (sum(frame[:10]) & 0xFF) != frame[10]:
                        buf = buf[1:]      # 校验失败，滑动重同步
                        continue

                    t = frame[1]
                    x = _s16(frame[2], frame[3])
                    y = _s16(frame[4], frame[5])
                    z = _s16(frame[6], frame[7])

                    with self._lock:
                        if t == FRAME_ACC:
                            self._acc_raw = (x, y, z)
                        elif t == FRAME_GYRO:
                            self._gyro_raw = (x, y, z)
                        elif t == FRAME_ANGLE:
                            self._angle_raw = (x, y, z)
                        self._frames += 1
                        self._last_time = time.time()

                    buf = buf[FRAME_LEN:]

            except Exception:
                time.sleep(0.01)

    def read(self):
        """返回物理单位的字典（非阻塞，取最新值）。

        返回 None 表示尚未收到有效帧。
        无加速度帧时，由姿态角反推重力方向。
        """
        with self._lock:
            acc = self._acc_raw
            gyro = self._gyro_raw
            angle = self._angle_raw

        if angle is None and gyro is None:
            return None

        result = {
            'timestamp': time.time(),
            'accel': None,
            'gyro': None,
            'angle': None,
            'accel_derived': False,
        }

        if gyro is not None:
            result['gyro'] = {
                'x': gyro[0] / 32768.0 * SCALE_GYRO_DPS,
                'y': gyro[1] / 32768.0 * SCALE_GYRO_DPS,
                'z': gyro[2] / 32768.0 * SCALE_GYRO_DPS,
            }  # °/s

        if angle is not None:
            roll = angle[0] / 32768.0 * SCALE_ANGLE_DEG
            pitch = angle[1] / 32768.0 * SCALE_ANGLE_DEG
            yaw = angle[2] / 32768.0 * SCALE_ANGLE_DEG
            result['angle'] = {'roll': roll, 'pitch': pitch, 'yaw': yaw}  # °

        if acc is not None:
            result['accel'] = {
                'x': acc[0] / 32768.0 * SCALE_ACC_G,
                'y': acc[1] / 32768.0 * SCALE_ACC_G,
                'z': acc[2] / 32768.0 * SCALE_ACC_G,
            }  # g
        elif result['angle'] is not None:
            # 无加速度帧：由姿态角反推重力在机体系的投影
            # a_body = R^T * [0,0,g]，静止时加速度计读到的就是重力
            result['accel'] = self._gravity_from_euler(result['angle'])
            result['accel_derived'] = True

        return result

    @staticmethod
    def _gravity_from_euler(angle):
        """由欧拉角反推机体坐标系下的重力加速度（单位 g）。

        用于模块不输出加速度帧时，仍能给 RViz 提供正确的重力方向箭头。
        """
        r = math.radians(angle['roll'])
        p = math.radians(angle['pitch'])

        # 仅用 roll/pitch 即可确定重力方向（yaw 绕重力轴，不影响）
        return {
            'x': -math.sin(p),
            'y': math.sin(r) * math.cos(p),
            'z': math.cos(r) * math.cos(p),
        }
