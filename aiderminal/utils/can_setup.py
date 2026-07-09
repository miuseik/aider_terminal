"""
CAN 接口硬件初始化工具。

确保 SocketCAN 驱动正常工作：加载内核模块、配置 bitrate、
txqueuelen 并将接口 UP。

当 USB CAN 适配器固件卡死（gs_usb 无响应）时，自动通过 sysfs
authorized 切换进行 USB 软复位，然后重试配置。
"""

import logging
import os
import subprocess
import time as time_mod

logger = logging.getLogger(__name__)


def _usb_reset_for_can(can_if: str) -> bool:
    """
    USB CAN 适配器卡死时，通过 sysfs authorized 切换进行软复位。

    从 /sys/class/net/{can_if}/device 向上找到 USB 设备的
    authorized 文件，写 0 再写 1 触发 USB 重新枚举。

    Returns:
        True  复位成功
        False 找不到设备或写入失败
    """
    device_dir = f"/sys/class/net/{can_if}/device"
    if not os.path.isdir(device_dir):
        logger.warning("USB reset: %s 没有 device 目录", can_if)
        return False

    # 向上查找带 authorized 的父设备（USB 设备级）
    path = os.path.realpath(device_dir)
    for _ in range(5):
        parent = os.path.dirname(path)
        auth_file = os.path.join(parent, "authorized")
        if os.path.exists(auth_file):
            usb_dev = os.path.basename(parent)
            logger.warning(
                "USB CAN 适配器 %s 无响应，尝试 USB 软复位...", usb_dev,
            )
            try:
                with open(auth_file, "w") as f:
                    f.write("0")
                time_mod.sleep(1)
                with open(auth_file, "w") as f:
                    f.write("1")
                # 等待 CAN 接口重新出现（USB 枚举需要时间）
                for i in range(10):
                    if os.path.isdir(device_dir):
                        break
                    time_mod.sleep(0.5)
                else:
                    logger.error("USB 复位后 %s 未重新出现", can_if)
                    return False
                logger.info("USB 软复位完成: %s", usb_dev)
                return True
            except Exception as e:
                logger.error("USB 软复位失败 %s: %s", usb_dev, e)
                return False
        path = parent

    logger.warning("USB reset: 找不到 %s 的父 USB 设备", can_if)
    return False


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
        cmd = ["sudo", "-n", *args] if os.geteuid() != 0 else list(args)
        try:
            return subprocess.run(
                cmd,
                capture_output=True, text=True, timeout=timeout,
            )
        except subprocess.TimeoutExpired as e:
            logger.warning("CAN command timed out after %.0fs: %s", timeout, " ".join(cmd))
            # 返回一个 "失败" 的 CompletedProcess，避免异常上抛
            return subprocess.CompletedProcess(
                args=cmd, returncode=-1, stdout="", stderr=str(e),
            )

    def _config_sequence() -> tuple:
        """
        执行 CAN 配置序列: down → bitrate → (restart-ms) → txqueuelen → up。

        restart-ms 让控制器进 BUS-OFF 后内核自动重启；但**部分 CAN 控制器
        （某些 USB-CAN / SPI CAN）不支持该参数**，命令会报
        "Device doesn't support restart from Bus Off"。此时必须**忽略该错误继续
        up**，否则整条配置失败、接口永远 DOWN（表现为反复 "Network is down"、
        电机全掉线）。硬件不支持自动恢复时，BUS-OFF 改由 actuator_controller 的
        软件重连触发本函数的 down→up 来清除。
        Returns:
            (ok: bool, timed_out: bool)
        """
        # 1) down
        res = _sudo("ip", "link", "set", can_if, "down")
        if res.returncode != 0:
            logger.warning("CAN setup failed: sudo ip link set %s down — %s",
                           can_if, res.stderr.strip())
            return False, res.returncode == -1
        # 2) 设 bitrate（不带 restart-ms，避免不支持的硬件整条失败）
        res = _sudo("ip", "link", "set", can_if, "type", "can", "bitrate", bitrate)
        if res.returncode != 0:
            logger.warning("CAN setup failed: sudo ip link set %s type can bitrate — %s",
                           can_if, res.stderr.strip())
            return False, res.returncode == -1
        # 3) 尝试启用 BUS-OFF 自动恢复（硬件不支持则仅告警，不阻断 up）
        res = _sudo("ip", "link", "set", can_if, "type", "can", "restart-ms", "100")
        if res.returncode != 0:
            logger.warning(
                "CAN %s 不支持 BUS-OFF 自动恢复(restart-ms)，将依赖软件重连 down→up 清除 BUS-OFF — %s",
                can_if, res.stderr.strip(),
            )
        # 4) txqueuelen
        res = _sudo("ip", "link", "set", can_if, "txqueuelen", txqlen)
        if res.returncode != 0:
            logger.warning("CAN setup failed: sudo ip link set %s txqueuelen — %s",
                           can_if, res.stderr.strip())
            return False, res.returncode == -1
        # 5) up（必须执行，接口 UP 后电机才能通信）
        res = _sudo("ip", "link", "set", can_if, "up")
        if res.returncode != 0:
            logger.warning("CAN setup failed: sudo ip link set %s up — %s",
                           can_if, res.stderr.strip())
            return False, res.returncode == -1
        return True, False

    # 1) 加载内核模块
    for mod in ("can", "can_raw", "gs_usb"):
        try:
            r = _sudo("modprobe", mod)
            if r.returncode != 0 and mod != "gs_usb":
                logger.debug("modprobe %s: %s", mod, r.stderr.strip())
        except FileNotFoundError:
            logger.debug("modprobe not available — skip CAN module loading")
            break

    # 2) 检查 can 口是否存在
    r = subprocess.run(
        ["ip", "link", "show", can_if],
        capture_output=True, text=True, timeout=2,
    )
    if r.returncode != 0:
        logger.info("%s not present — skip CAN setup (no hardware)", can_if)
        return False

    # 3) 已是 UP、队列足够、总线健康 → 才跳过（不打断通信）。
    #    BUS-OFF / ERROR-PASSIVE / STOPPED 等故障态必须强制 down→up 清除，
    #    否则 BUS-OFF 会永久卡死（反复 "Network is down"）。
    #    注意：部分 CAN 控制器不支持 restart-ms（报 "Device doesn't support
    #    restart from Bus Off"），无法靠内核自动退出 BUS-OFF；此时 BUS-OFF
    #    由 actuator_controller 软件重连触发本函数的 down→up 清除，不强制要求 restart_ms>0。
    state_r = subprocess.run(
        ["ip", "-br", "link", "show", can_if],
        capture_output=True, text=True, timeout=2,
    )
    if state_r.returncode == 0:
        parts = state_r.stdout.strip().split()
        if len(parts) >= 3 and parts[2] == "UP":
            # 读总线状态与健康/重启参数
            try:
                with open(f"/sys/class/net/{can_if}/can_state") as f:
                    can_state = f.read().strip().upper()
            except OSError:
                can_state = "UNKNOWN"
            try:
                with open(f"/sys/class/net/{can_if}/tx_queue_len") as f:
                    qlen = int(f.read().strip())
            except OSError:
                qlen = 0
            try:
                with open(f"/sys/class/net/{can_if}/restart_ms") as f:
                    restart_ms = int(f.read().strip())
            except (OSError, ValueError):
                restart_ms = 0

            bus_healthy = can_state in ("ERROR-ACTIVE", "UNKNOWN", "DOWN")
            if bus_healthy and qlen >= 500:
                # 已 UP 且健康：硬件支持自动恢复则静默跳过；不支持则提示依赖软件重连。
                if restart_ms == 0:
                    logger.debug("%s already UP & healthy (硬件不支持 BUS-OFF 自动恢复，依赖软件重连)", can_if)
                else:
                    logger.debug("%s already UP, healthy, restart-ms=%d — skip", can_if, restart_ms)
                return True
            # 不自跳：总线故障态 → 强制 down→up 清 BUS-OFF
            if can_state in ("BUS-OFF", "ERROR-PASSIVE", "STOPPED"):
                logger.warning(
                    "CAN %s 总线异常(%s)，强制重启接口清除 BUS-OFF/ERROR-PASSIVE",
                    can_if, can_state,
                )

    # 4) 配置: down → bitrate → txqueuelen → up
    ok, timed_out = _config_sequence()
    if not ok and timed_out:
        # USB 适配器无响应 → 尝试软复位后重试一次
        if _usb_reset_for_can(can_if):
            ok, _ = _config_sequence()
            if not ok:
                logger.error("%s USB 复位后仍配置失败", can_if)
                return False
        else:
            return False
    elif not ok:
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
