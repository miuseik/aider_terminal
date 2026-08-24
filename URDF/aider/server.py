#!/usr/bin/env python3
"""URDF viewer server — HTTP static files + WebSocket 纯推送。

架构:
  同端口处理两件事:
    1. HTTP GET  → 静态文件 (viewer.html, meshes/, urdf/)
    2. WebSocket → 纯推送: 每 16ms 推送 RobotState 运动数据

仿真服务器不处理任何控制逻辑:
  - 不接收键盘事件
  - 不做模式切换
  - 只推送 pipeline 产出的 RobotState (joint_values, ee 坐标, target 等)
  - 浏览器用 URDF + RobotState 驱动 3D 渲染
"""
import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import struct
import sys
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

URDF_DIR = os.path.dirname(os.path.abspath(__file__))

# ── MIME 类型 ──────────────────────────────────────────
mimetypes.init()
MIME_MAP = {
    **{k: v for k, v in mimetypes.types_map.items()},
    ".stl": "application/octet-stream",
    ".STL": "application/octet-stream",
    ".urdf": "application/xml",
    ".wasm": "application/wasm",
}

# ── WebSocket 常量 ─────────────────────────────────────
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_TEXT = 0x1
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA
STATE_HZ = 60

# ── 全局 (app.py 注入) ────────────────────────────────
_pipeline = None  # MotionPipeline
_sim_writers: list = []  # 仿真 WebSocket writers


def set_pipeline(pipeline) -> None:
    """注入 MotionPipeline (app.py 初始化时调用)."""
    global _pipeline
    _pipeline = pipeline


def register_sim_writer(writer) -> None:
    """注册仿真 WebSocket writer."""
    _sim_writers.append(writer)


def unregister_sim_writer(writer) -> None:
    """注销仿真 WebSocket writer."""
    if writer in _sim_writers:
        _sim_writers.remove(writer)


def set_state_map(mapping: dict) -> None:
    """注入 motor_id → joint_name 映射，浏览器按需加载。"""
    global _state_map
    _state_map = mapping


def _motor_targets_to_dict(targets: dict) -> dict:
    """将 {1: 1.57, 2: -0.5} 转为 JSON 序列化格式（key 转为字符串）。"""
    return {str(k): v for k, v in targets.items()}


# ── WebSocket 帧编解码 ─────────────────────────────────

def _ws_accept_key(key: str) -> str:
    digest = hashlib.sha1((key + WS_GUID).encode()).digest()
    return base64.b64encode(digest).decode()


def _encode_ws_frame(payload: bytes, opcode: int = OP_TEXT) -> bytes:
    """编码服务器→客户端帧（无 mask）."""
    b0 = 0x80 | opcode
    n = len(payload)
    if n < 126:
        return bytes([b0, n]) + payload
    elif n < 65536:
        return bytes([b0, 126]) + struct.pack(">H", n) + payload
    else:
        return bytes([b0, 127]) + struct.pack(">Q", n) + payload


class _WSDecoder:
    """WebSocket 帧流式解码器."""

    def __init__(self) -> None:
        self._buf = b""

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf += data
        frames: list[tuple[int, bytes]] = []
        while True:
            opcode, payload, consumed = self._decode_one(self._buf)
            if consumed == 0:
                break
            self._buf = self._buf[consumed:]
            frames.append((opcode, payload))
        return frames

    @staticmethod
    def _decode_one(data: bytes) -> tuple[int, bytes, int]:
        if len(data) < 2:
            return -1, b"", 0
        b0 = data[0]
        b1 = data[1]
        opcode = b0 & 0x0F
        length = b1 & 0x7F

        offset = 2
        if length == 126:
            if len(data) < 4:
                return -1, b"", 0
            length = struct.unpack(">H", data[2:4])[0]
            offset = 4
        elif length == 127:
            if len(data) < 10:
                return -1, b"", 0
            length = struct.unpack(">Q", data[2:10])[0]
            offset = 10

        mask_key = b""
        if (b1 & 0x80) != 0:
            if len(data) < offset + 4:
                return -1, b"", 0
            mask_key = data[offset:offset + 4]
            offset += 4

        total = offset + length
        if len(data) < total:
            return -1, b"", 0

        payload = data[offset:total]
        if mask_key:
            payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
        return opcode, payload, total


# ── HTTP 请求解析 ──────────────────────────────────────

class _HttpRequest:
    __slots__ = ("method", "path", "headers")

    def __init__(self) -> None:
        self.method = ""
        self.path = ""
        self.headers: dict[str, str] = {}

    @classmethod
    def parse(cls, data: bytes) -> Optional["_HttpRequest"]:
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            return None
        lines = text.split("\r\n")
        if not lines:
            return None
        parts = lines[0].split(" ")
        if len(parts) < 2:
            return None
        req = cls()
        req.method = parts[0]
        req.path = parts[1]
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                req.headers[k.strip().lower()] = v.strip()
        return req


# ── 静态文件服务 ───────────────────────────────────────

def _safe_path(path: str) -> Optional[str]:
    if path in ("", "/"):
        path = "/viewer.html"
    safe = os.path.normpath(os.path.join(URDF_DIR, path.lstrip("/")))
    if not safe.startswith(URDF_DIR):
        return None
    if not os.path.isfile(safe):
        return None
    return safe


async def _serve_file(writer: asyncio.StreamWriter, path: str) -> None:
    safe = _safe_path(path)
    if safe is None:
        body = b"Not Found"
        response = (
            b"HTTP/1.1 404 Not Found\r\n"
            b"Content-Type: text/plain\r\n"
            b"Content-Length: " + str(len(body)).encode() + b"\r\n"
            b"Connection: close\r\n\r\n"
        ) + body
        writer.write(response)
        await writer.drain()
        return

    ext = os.path.splitext(safe)[1]
    content_type = MIME_MAP.get(ext, "application/octet-stream")

    try:
        with open(safe, "rb") as f:
            data = f.read()
    except OSError:
        body = b"Internal Server Error"
        response = (
            b"HTTP/1.1 500 Internal Server Error\r\n"
            b"Content-Type: text/plain\r\n"
            b"Content-Length: " + str(len(body)).encode() + b"\r\n"
            b"Connection: close\r\n\r\n"
        ) + body
        writer.write(response)
        await writer.drain()
        return

    header = (
        "HTTP/1.1 200 OK\r\n"
        f"Content-Type: {content_type}\r\n"
        f"Content-Length: {len(data)}\r\n"
        "Connection: close\r\n\r\n"
    ).encode()
    writer.write(header + data)
    await writer.drain()


# ── WebSocket: 纯推送 ─────────────────────────────────

async def _ws_state_pusher(writer: asyncio.StreamWriter) -> None:
    """60fps 推送 — 全量 RobotState 给浏览器渲染。

    只读取最新状态快照（pipeline 由 KeyboardHandler 驱动 tick），
    不自行调用 tick()，避免与 KeyboardHandler 的 20Hz 驱动冲突导致重复计算。
    """
    # 注册 writer
    register_sim_writer(writer)
    
    interval = 1.0 / STATE_HZ
    pipeline = _pipeline
    frame = 0
    try:
        while True:
            if pipeline:
                state = pipeline.snapshot()
                data = {
                    "joint_values": state.joint_values,
                    "ee_left": state.ee_left,
                    "ee_right": state.ee_right,
                    "target_left": state.target_left,
                    "target_right": state.target_right,
                    "target_mode": state.target_mode,
                    "dragging": state.dragging,
                    "left_ik_error": state.left_ik_error,
                    "right_ik_error": state.right_ik_error,
                    "base_x": state.base_x,
                    "base_y": state.base_y,
                    "base_yaw": state.base_yaw,
                    "wheel_speeds": state.wheel_speeds,
                    "motor_targets": _motor_targets_to_dict(state.motor_targets),
                }
            else:
                data = {"motor_targets": {}}
                if frame == 0:
                    print("[SIM] _ws_state_pusher: pipeline is NONE!")

            frame += 1

            payload = json.dumps(data, separators=(",", ":")).encode()
            writer.write(_encode_ws_frame(payload))
            await writer.drain()
            await asyncio.sleep(interval)
    except (ConnectionError, BrokenPipeError, RuntimeError):
        pass
    finally:
        # 注销 writer
        unregister_sim_writer(writer)


async def _run_ws(reader: asyncio.StreamReader, writer: asyncio.StreamWriter,
                  req: _HttpRequest) -> None:
    """WebSocket 连接 — 握手后纯推送，客户端消息仅处理 close/ping。"""
    ws_key = req.headers.get("sec-websocket-key", "")
    if not ws_key:
        writer.close()
        return

    accept = _ws_accept_key(ws_key)
    handshake = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
    ).encode()
    writer.write(handshake)
    await writer.drain()

    decoder = _WSDecoder()
    push_task = asyncio.ensure_future(_ws_state_pusher(writer))

    try:
        while True:
            chunk = await reader.read(4096)
            if not chunk:
                break

            for opcode, payload in decoder.feed(chunk):
                if opcode == OP_CLOSE:
                    return
                elif opcode == OP_PING:
                    writer.write(_encode_ws_frame(b"", OP_PONG))
                    await writer.drain()
                # 忽略所有 TEXT 消息 — 仿真不处理控制
    finally:
        push_task.cancel()
        try:
            await push_task
        except asyncio.CancelledError:
            pass


# ── TCP 入口 ───────────────────────────────────────────

async def _handle_client(reader: asyncio.StreamReader,
                         writer: asyncio.StreamWriter) -> None:
    try:
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = await reader.read(4096)
            if not chunk:
                writer.close()
                return
            head += chunk
            if len(head) > 65536:
                writer.close()
                return

        req = _HttpRequest.parse(head)
        if req is None:
            writer.close()
            return

        upgrade = req.headers.get("upgrade", "").lower()
        if upgrade == "websocket":
            await _run_ws(reader, writer, req)
        else:
            await _serve_file(writer, req.path)
    except (ConnectionError, asyncio.IncompleteReadError, BrokenPipeError):
        pass
    finally:
        try:
            writer.close()
        except Exception:
            pass


# ── 服务端封装 ─────────────────────────────────────────

class URDFViewerServer:
    """Asyncio 原生服务器: HTTP 静态文件 + WebSocket 纯推送。

    不包含任何控制逻辑，只做数据推送。
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8080, max_retry: int = 10) -> None:
        self.host = host
        self.port = port
        self._max_retry = max_retry
        self._server: Optional[asyncio.AbstractServer] = None
        self._actual_port: int = port

    @property
    def url(self) -> str:
        return f"http://localhost:{self._actual_port}/viewer.html"

    async def start(self) -> None:
        for offset in range(self._max_retry):
            try_port = self.port + offset
            try:
                self._server = await asyncio.start_server(
                    _handle_client, self.host, try_port,
                )
                self._actual_port = try_port
                break
            except OSError:
                continue

        if self._server is None:
            raise OSError(
                f"No port available in {self.port}-{self.port + self._max_retry - 1}"
            )

    async def stop(self) -> None:
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None


# ── 自测 ──────────────────────────────────────────────
if __name__ == "__main__":
    async def main() -> None:
        srv = URDFViewerServer()
        await srv.start()
        print(f"URDF Viewer + WebSocket: {srv.url}")
        await asyncio.Event().wait()

    asyncio.run(main())
