"""
Server API 客户端
用于 Terminal 与 Server 通信，获取配置等信息
"""
import logging
from pathlib import Path
from typing import Optional, Dict

logger = logging.getLogger(__name__)


class ServerAPIClient:
    """Server API 客户端"""
    
    def __init__(self, config_path: Optional[Path] = None):
        """初始化 API 客户端
        
        Args:
            config_path: config.yaml 路径，默认自动查找
        """
        if config_path is None:
            config_path = Path(__file__).parent.parent / 'config.yaml'
        
        self.server_host = 'localhost'
        self.server_port = 8000
        self._load_server_config(config_path)
    
    def _load_server_config(self, config_path: Path):
        """从 config.yaml 加载 Server 地址"""
        try:
            if config_path.exists():
                import yaml
                with open(config_path, 'r', encoding='utf-8') as f:
                    terminal_config = yaml.safe_load(f)
                    network_config = terminal_config.get('network', {})
                    self.server_host = network_config.get('server_host', 'localhost')
                    self.server_port = network_config.get('server_port', 8000)
                logger.info(f"📡 Server 地址: {self.server_host}:{self.server_port}")
            else:
                logger.warning(f"⚠️ 配置文件不存在: {config_path}，使用默认地址")
        except Exception as e:
            logger.error(f"❌ 加载 Server 配置失败: {e}")
    
    def get_servo_ids_config(self) -> Optional[Dict]:
        """从 Server 获取舵机配置
        
        Returns:
            舵机配置字典，失败返回 None
        """
        try:
            import requests
            
            url = f"http://{self.server_host}:{self.server_port}/api/get-servo-ids"
            logger.info(f"🔍 从 Server 获取舵机配置: {url}")
            
            response = requests.post(url, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('code') == 200 and data.get('data'):
                    logger.info("✅ 成功获取舵机配置")
                    return data['data']
                else:
                    logger.warning(f"⚠️ Server 返回的配置数据无效: {data}")
                    return None
            else:
                logger.error(f"❌ 从 Server 获取配置失败: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            logger.error(f"❌ 获取舵机配置异常: {e}")
            return None
