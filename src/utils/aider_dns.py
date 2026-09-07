"""
aider_dns — 应用层域名解析，无需 root 权限

在 Python 进程中植入自定义 DNS 解析，将 `aider.local` 解析为 `.aider_host` 文件中
记录的当前 IP 地址。不影响其他域名的正常解析。

用法:
    import aider_dns
    aider_dns.patch()          # 进程内启用域名解析
    import socket
    socket.gethostbyname("aider.local")  # → 192.168.0.110

工作原理:
    1. patch() 后 monkey-patch socket.getaddrinfo / socket.gethostbyname
    2. 遇到 aider.local → 从 .aider_host 文件读取 IP
    3. 其他域名 → 原样走系统 DNS
"""

import socket as _socket
import os
import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# 域名和配置文件名
AIDER_DOMAIN = os.getenv("AIDER_DOMAIN", "aider.local")

# Host IP 文件查找顺序
# 优先级: 环境变量 > terminal 项目目录 > server 项目目录 > 项目根目录
_AIDER_HOST_FILE = os.getenv("AIDER_HOST_FILE", "")


def _find_host_file() -> Optional[Path]:
    """查找 .aider_host 文件"""
    # 1. 环境变量指定
    if _AIDER_HOST_FILE:
        p = Path(_AIDER_HOST_FILE)
        if p.exists():
            return p

    # 2. 项目根目录 (最常见)
    #    aider_terminal / aider_server 都在 /home/xxx/www/aider/ 下
    candidates = [
        Path("/app/.aider_host"),                  # Docker 内
        Path(__file__).resolve().parent.parent.parent / ".aider_host",  # 项目根
    ]

    for p in candidates:
        if p.exists():
            return p
    return None


def _read_host_ip() -> Optional[str]:
    """从 .aider_host 读取当前 IP"""
    host_file = _find_host_file()
    if host_file is None:
        return None
    try:
        content = host_file.read_text().strip()
        if content:
            return content.split("\n")[0].split("#")[0].strip()
    except Exception:
        pass
    return None


# ====== 保存原始函数 ======
_original_getaddrinfo = _socket.getaddrinfo
_original_gethostbyname = _socket.gethostbyname


def _patched_getaddrinfo(host, port, family=0, type=0, proto=0, flags=0):
    """Monkey-patched getaddrinfo: aider.local → 从 .aider_host 读取 IP"""
    if host == AIDER_DOMAIN:
        ip = _read_host_ip()
        if ip:
            # 根据 IP 格式推断地址族：IPv4 → AF_INET, IPv6 → AF_INET6
            if ':' in ip:
                resolved_family = _socket.AF_INET6
            else:
                resolved_family = _socket.AF_INET
            return [(resolved_family, type, proto, "", (ip, port))]
    return _original_getaddrinfo(host, port, family, type, proto, flags)


def _patched_gethostbyname(hostname):
    """Monkey-patched gethostbyname: aider.local → 从 .aider_host 读取 IP"""
    if hostname == AIDER_DOMAIN:
        ip = _read_host_ip()
        if ip:
            return ip
    return _original_gethostbyname(hostname)


def patch() -> bool:
    """启用 aider.local 域名解析。返回 True 表示配置成功"""
    ip = _read_host_ip()
    if ip is None:
        logger.warning(
            "aider_dns: 未找到 .aider_host 文件，%s 将无法解析。"
            "请先启动 aider_server 或手动创建 .aider_host 写入当前 IP。",
            AIDER_DOMAIN,
        )
    else:
        logger.info("aider_dns: %s → %s", AIDER_DOMAIN, ip)

    _socket.getaddrinfo = _patched_getaddrinfo
    _socket.gethostbyname = _patched_gethostbyname
    return ip is not None


def write_host_ip(ip: str, host_file: Optional[str] = None) -> bool:
    """写入当前 IP 到 .aider_host 文件（由 server 启动时调用）"""
    target = host_file or _AIDER_HOST_FILE
    if not target:
        # 默认写到项目根目录
        target = str(Path(__file__).resolve().parent.parent.parent / ".aider_host")
    try:
        Path(target).write_text(f"{ip}\n")
        logger.info("aider_dns: 已写入 %s → %s", target, ip)
        return True
    except Exception as e:
        logger.error("aider_dns: 写入 %s 失败: %s", target, e)
        return False
