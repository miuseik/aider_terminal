"""
Server API 客户端
用于 Terminal 与 Server 通信，获取配置等信息

职责:
- 提供 HTTP/HTTPS API 接口调用
- 处理与 Server 的数据交换
- 统一管理 API 端点和认证
"""
import logging
from pathlib import Path
from typing import Optional, Dict, Any

# 导入统一的地址获取函数
from telegrip.config import get_api_endpoint, config as telegrip_config

logger = logging.getLogger(__name__)


class ServerAPIClient:
    """
    Server API 客户端
    
    提供与 Server 通信的所有 API 接口
    """
    
    def __init__(self):
        """初始化 API 客户端，使用统一的地址解析逻辑"""
        self.api_host = get_api_endpoint()
        self.server_port = telegrip_config.websocket_port
        self.base_url = f"https://{self.api_host}:{self.server_port}"
        print(f"📡 Server API 客户端初始化 - Base URL: {self.base_url}")
    
    def _make_request(self, endpoint: str, method: str = 'POST', data: Dict = None) -> Optional[Dict]:
        """
        通用请求方法
        
        Args:
            endpoint: API 端点路径
            method: HTTP 方法 ('GET', 'POST', 'PUT', 'DELETE')
            data: 请求数据
            
        Returns:
            响应数据字典，失败返回 None
        """
        try:
            import requests
            
            url = f"{self.base_url}{endpoint}"
            print(f"🔍 {method} {url}")
            
            # 禁用 SSL 验证（如果是自签名证书）
            if method.upper() == 'GET':
                response = requests.get(url, timeout=5, verify=False)
            elif method.upper() == 'POST':
                response = requests.post(url, json=data, timeout=5, verify=False)
            elif method.upper() == 'PUT':
                response = requests.put(url, json=data, timeout=5, verify=False)
            elif method.upper() == 'DELETE':
                response = requests.delete(url, timeout=5, verify=False)
            else:
                print(f"❌ 不支持的 HTTP 方法: {method}")
                return None
            
            if response.status_code == 200:
                data = response.json()
                if data.get('code') == 200:
                    return data.get('data')
                else:
                    print(f"⚠️ Server 返回错误: {data}")
                    return None
            else:
                print(f"❌ HTTP 错误: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ API 请求异常: {e}")
            return None
    
    # ==================== 配置管理 API ====================
    
    def get_servo_ids_config(self) -> Optional[Dict]:
        """
        从 Server 获取舵机配置
        
        Returns:
            舵机配置字典，失败返回 None
        """
        return self._make_request('/api/get-servo-ids', 'POST')
    
    def update_servo_config(self, config: Dict) -> bool:
        """
        更新舵机配置到 Server
        
        Args:
            config: 舵机配置字典
            
        Returns:
            bool: 是否成功
        """
        result = self._make_request('/api/update-servo-config', 'POST', {'config': config})
        return result is not None
    
    # ==================== 用户管理 API（预留）====================
    
    def login(self, username: str, password: str) -> Optional[Dict]:
        """
        用户登录（预留）
        
        Args:
            username: 用户名
            password: 密码
            
        Returns:
            用户信息/token，失败返回 None
        """
        # TODO: 实现登录逻辑
        return self._make_request('/api/login', 'POST', {
            'username': username,
            'password': password
        })
    
    def get_user_info(self) -> Optional[Dict]:
        """
        获取用户信息（预留）
        
        Returns:
            用户信息字典，失败返回 None
        """
        # TODO: 实现获取用户信息逻辑
        return self._make_request('/api/user/info', 'GET')
    
    # ==================== 数据同步 API（预留）====================
    
    def sync_robot_data(self, robot_id: str, data: Dict) -> bool:
        """
        同步机器人数据到 Server（预留）
        
        Args:
            robot_id: 机器人 ID
            data: 要同步的数据
            
        Returns:
            bool: 是否成功
        """
        # TODO: 实现数据同步逻辑
        result = self._make_request(f'/api/robot/{robot_id}/sync', 'POST', data)
        return result is not None
    
    def get_robot_config(self, robot_id: str) -> Optional[Dict]:
        """
        获取机器人配置（预留）
        
        Args:
            robot_id: 机器人 ID
            
        Returns:
            机器人配置字典，失败返回 None
        """
        # TODO: 实现获取机器人配置逻辑
        return self._make_request(f'/api/robot/{robot_id}/config', 'GET')
