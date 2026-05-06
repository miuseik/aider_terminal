"""
网络信息工具模块
"""
import socket
import subprocess
import logging
import re

logger = logging.getLogger(__name__)


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
    # 方法 1: 尝试 iwgetid (最轻量，通常在 raspbian 中预装)
    try:
        result = subprocess.run(
            ['iwgetid', '-r'], 
            capture_output=True, 
            text=True,
            timeout=5
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    except Exception as e:
        print(f"iwgetid 失败: {e}")

    # 方法 2: 尝试 nmcli (现代桌面版系统)
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
    except Exception as e:
        print(f"nmcli 失败: {e}")
    
    # 方法 3: 尝试 iw (推荐，大多数 Linux 系统都有)
    try:
        # 先获取无线接口名称
        result = subprocess.run(
            ['iw', 'dev'], 
            capture_output=True, 
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            # 解析接口名称
            for line in result.stdout.splitlines():
                if line.strip().startswith('Interface '):
                    interface = line.strip().split()[1]
                    # 获取该接口的连接信息
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
    except Exception as e:
        print(f"iw 失败: {e}")
    
    # 方法 4: 尝试 iwconfig (旧系统)
    try:
        result = subprocess.run(
            ['iwconfig', 'wlan0'], 
            capture_output=True, 
            text=True,
            timeout=5
        )
        output = result.stdout
        if "ESSID:" in output:
            ssid = output.split('ESSID:"')[1].split('"')[0]
            return ssid if ssid else "Not Connected"
    except Exception as e:
        print(f"iwconfig 失败: {e}")
    
    print("所有 WiFi SSID 获取方法均失败")
    return "Not Connected"


def get_network_info() -> dict:
    """获取完整的网络信息"""
    return {
        "ip": get_local_ip(),
        "ssid": get_wifi_ssid(),
        "hostname": socket.gethostname()
    }
