"""
命令路由 — 收上位机指令 → 解析 → 分发给对应控制器。
"""
import asyncio
import json
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class CommandRouter:
    """上位机指令分发中心。"""

    def __init__(self) -> None:
        self._handlers: dict[str, callable] = {}

    def register(self, cmd_type: str, handler: callable) -> None:
        """注册指令处理器。"""
        self._handlers[cmd_type] = handler

    async def handle(self, raw_message: str) -> None:
        """解析 JSON 指令并分发给对应 handler（传整个 dict）。"""
        try:
            msg: dict = json.loads(raw_message)
        except json.JSONDecodeError:
            logger.warning("CommandRouter invalid JSON: %s", raw_message[:100])
            return

        cmd_type = msg.get("type", "")
        handler = self._handlers.get(cmd_type)
        if handler is None:
            logger.warning("CommandRouter unhandled type=%s", cmd_type)
            return

        try:
            if asyncio.iscoroutinefunction(handler):
                await handler(msg)
            else:
                handler(msg)
        except Exception:
            logger.exception("CommandRouter handler error for type=%s", cmd_type)
