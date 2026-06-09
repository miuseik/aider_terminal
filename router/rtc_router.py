"""
RTC 路由 — 处理 WebSocket 的 rtc_command 指令，分发给 RTCController。

消息格式:
{
    "type": "rtc_command",
    "action": "start"  或  "stop"
}

对应原文件: aider_terminal/src/rtc_video.py（命令控制部分）
"""
import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from control.rtc_controller import RTCController

logger = logging.getLogger(__name__)


class RTCRouter:
    """RTC 命令路由 — 自注册到 CommandRouter."""

    def __init__(self, controller: "RTCController") -> None:
        self._ctrl = controller

    def register_with(self, cmd_router: "CommandRouter") -> None:
        """向 CommandRouter 注册本路由的所有处理器."""
        cmd_router.register("rtc_command", self.handle_command)
        cmd_router.register("reconnect", self.handle_reconnect)

    async def handle_command(self, payload: dict) -> None:
        """处理 rtc_command 消息。

        payload:
            action: "start" | "stop"
        """
        action = payload.get("action", "")
        if action == "start":
            await self._start()
        elif action == "stop":
            await self._stop()
        else:
            logger.warning("RTCRouter unhandled action=%s", action)

    async def handle_reconnect(self, payload: dict) -> None:
        """前端 WebRTC 重连/开始推流通知。"""
        logger.debug("RTCRouter: reconnect received → starting RTC")
        await self._start()

    # ── 内部 ──────────────────────────────────────────

    async def _start(self) -> None:
        if self._ctrl.is_running:
            logger.info("RTCRouter: RTC already running, skip")
            return
        await self._ctrl.start()
        logger.info("RTCRouter: start requested")

    async def _stop(self) -> None:
        if not self._ctrl.is_running:
            logger.info("RTCRouter: RTC not running, skip")
            return
        await self._ctrl.stop()
        logger.info("RTCRouter: stop requested")
