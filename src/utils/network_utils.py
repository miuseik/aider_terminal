"""
网络信息工具模块
"""
import socket
import subprocess
import os
import logging
import re

logger = logging.getLogger(__name__)


def _find_wireless_interface() -> str:
    """查找当前活动的无线网卡接口名，找不到返回 None。"""
    # 方法 A: 读 /proc/net/wireless（无需任何外部命令）
    try:
        with open('/proc/net/wireless', 'r') as f:
            lines = f.readlines()
            if len(lines) >= 3:  # 前两行是表头
                # 第三行开始是接口数据，格式: wlan0: ...
                for line in lines[2:]:
                    parts = line.strip().split(':')
                    if parts:
                        iface = parts[0].strip()
                        if iface:
                            return iface
    except (FileNotFoundError, IOError, IndexError):
        pass

    # 方法 B: 遍历 /sys/class/net 找有 wireless 子目录的接口
    try:
        for name in os.listdir('/sys/class/net'):
            wireless_dir = os.path.join('/sys/class/net', name, 'wireless')
            phy_dir = os.path.join('/sys/class/net', name, 'phy80211')
            if os.path.exists(wireless_dir) or os.path.exists(phy_dir):
                return name
    except (FileNotFoundError, IOError):
        pass

    return None


def get_local_ip() -> str:
    """获取本机的局域网 IP 地址"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return socket.gethostbyname(socket.gethostname())


def get_wifi_ssid() -> str:
    """获取当前连接的 WiFi 名称 (SSID)"""
    # 方法 1: iwgetid (最轻量，Raspbian 预装)
    try:
        result = subprocess.run(
            ['iwgetid', '-r'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    except Exception:
        pass

    # 方法 2: nmcli (现代桌面系统)
    try:
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if line.startswith('yes:'):
                    return line.split(':', 1)[1]
    except Exception:
        pass

    # 方法 3: iw (推荐，大多数 Linux 都有)
    try:
        result = subprocess.run(
            ['iw', 'dev'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if line.strip().startswith('Interface '):
                    interface = line.strip().split()[1]
                    link_result = subprocess.run(
                        ['iw', 'dev', interface, 'link'],
                        capture_output=True,
                        text=True,
                        timeout=5
                    )
                    if link_result.returncode == 0:
                        for link_line in link_result.stdout.splitlines():
                            if link_line.strip().startswith('SSID:'):
                                ssid = link_line.strip().split(':', 1)[1].strip()
                                if ssid:
                                    return ssid
    except Exception:
        pass

    # 方法 4: iwconfig (旧系统)
    try:
        iface = _find_wireless_interface() or 'wlan0'
        result = subprocess.run(
            ['iwconfig', iface],
            capture_output=True,
            text=True,
            timeout=5
        )
        output = result.stdout
        if "ESSID:" in output:
            ssid = output.split('ESSID:"')[1].split('"')[0]
            return ssid if ssid else "Not Connected"
    except Exception:
        pass

    # 方法 5: 读 /sys/class/net/$iface/wireless/essid（内核暴露，无外部依赖）
    try:
        iface = _find_wireless_interface()
        if iface:
            essid_path = os.path.join('/sys/class/net', iface, 'wireless', 'essid')
            if os.path.exists(essid_path):
                with open(essid_path, 'rb') as f:
                    raw = f.read()
                    # 内核会以 null-padded bytes 返回
                    ssid = raw.rstrip(b'\x00').decode('utf-8', errors='replace').strip()
                    if ssid:
                        return ssid
    except Exception:
        pass

    return "Not Connected"


def get_network_info() -> dict:
    """获取完整的网络信息"""
    return {
        "ip": get_local_ip(),
        "ssid": get_wifi_ssid(),
        "hostname": socket.gethostname()
    }
