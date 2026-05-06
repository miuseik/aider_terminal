"""
Server API 客户端
用于 Terminal 与 Server 通信，获取配置等信息
"""
import logging
from pathlib import Path
from typing import Optional, Dict

# 导入统一的地址获取函数
from telegrip.config import get_api_endpoint, config as telegrip_config

logger = logging.getLogger(__name__)


class ServerAPIClient:
    """Server API 客户端"""
    
    def __init__(self):
        """初始化 API 客户端，使用统一的地址解析逻辑"""
        self.api_host = get_api_endpoint()
        self.server_port = telegrip_config.websocket_port
        print(f"📡 Server API 客户端初始化 - API: {self.api_host}:{self.server_port}")
    

    
    def get_servo_ids_config(self) -> Optional[Dict]:
        """从 Server 获取舵机配置
        
        Returns:
            舵机配置字典，失败返回 None
        """
        try:
            import requests
            
            # 尝试 HTTPS，如果失败再尝试 HTTP
            url = f"https://{self.api_host}:{self.server_port}/api/get-servo-ids"
            print(f"🔍 从 Server 获取舵机配置: {url}")
            
            # 禁用 SSL 验证（如果是自签名证书）
            response = requests.post(url, timeout=5, verify=False)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('code') == 200 and data.get('data'):
                    print("✅ 成功获取舵机配置")
                    return data['data']
                else:
                    print(f"⚠️ Server 返回的配置数据无效: {data}")
                    return None
            else:
                print(f"❌ 从 Server 获取配置失败: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ 获取舵机配置异常: {e}")
            return None
