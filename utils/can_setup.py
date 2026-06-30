"""
CAN 接口硬件初始化工具。

确保 SocketCAN 驱动正常工作：加载内核模块、配置 bitrate、
txqueuelen 并将接口 UP。

参考 aider_go/src/app.py:_setup_can() 移植。
"""

import logging
import subprocess
from typing import Optional

logger = logging.getLogger(__name__)


def setup_can(
    can_if: str = "can0",
    bitrate: str = "1000000",
    txqlen: str = "1000",
) -> bool:
    """
    启动时确保 CAN 接口已正确配置。

    Args:
        can_if:   CAN 接口名，默认 can0
        bitrate:  波特率，默认 1000000
        txqlen:   发送队列长度，默认 1000

    Returns:
        True  配置成功或已就绪
        False 配置失败（硬件不存在或权限不足）
    """
    def _sudo(*args, timeout: float = 3.0) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["sudo", "-n", *args],
            capture_output=True, text=True, timeout=timeout,
        )

    # 1) 加载内核模块
    for mod in ("can", "can_raw", "gs_usb"):
        r = _sudo("modprobe", mod)
        if r.returncode != 0 and mod != "gs_usb":  # gs_usb 可能不存在
            logger.debug("modprobe %s: %s", mod, r.stderr.strip())

    # 2) 检查 can0 是否存在
    r = subprocess.run(
        ["ip", "link", "show", can_if],
        capture_output=True, text=True, timeout=2,
    )
    if r.returncode != 0:
        logger.info("can0 not present — skip CAN setup (no hardware)")
        return False

    # 3) 已是 UP 且 txqueuelen 足够 → 跳过
    state_r = subprocess.run(
        ["ip", "-br", "link", "show", can_if],
        capture_output=True, text=True, timeout=2,
    )
    if state_r.returncode == 0:
        parts = state_r.stdout.strip().split()
        if len(parts) >= 3 and parts[2] == "UP":
            try:
                with open(f"/sys/class/net/{can_if}/tx_queue_len") as f:
                    qlen = int(f.read().strip())
                if qlen >= 500:
                    logger.debug("can0 already UP with txqueuelen=%d — skip", qlen)
                    return True
            except Exception:
                pass

    # 4) 配置: down → bitrate → txqueuelen → up
    for cmd_args in (
        ("ip", "link", "set", can_if, "down"),
        ("ip", "link", "set", can_if, "type", "can", "bitrate", bitrate),
        ("ip", "link", "set", can_if, "txqueuelen", txqlen),
        ("ip", "link", "set", can_if, "up"),
    ):
        r = _sudo(*cmd_args)
        if r.returncode != 0:
            logger.warning(
                "CAN setup failed: sudo %s — %s",
                " ".join(cmd_args), r.stderr.strip(),
            )
            return False

    # 5) 验证最终状态
    state_r = subprocess.run(
        ["ip", "-br", "link", "show", can_if],
        capture_output=True, text=True, timeout=2,
    )
    if state_r.returncode == 0:
        logger.info("CAN setup done: %s", state_r.stdout.strip())
    else:
        logger.warning("CAN setup done but status unknown")

    return True
