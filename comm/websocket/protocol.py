"""WebSocket 消息协议 - 负责消息的编码和解码"""
import json
from typing import Dict, Any


def encode_message(data: Dict[str, Any]) -> str:
    """将字典编码为 JSON 字符串"""
    return json.dumps(data)


def decode_message(raw: str) -> Dict[str, Any]:
    """将 JSON 字符串解码为字典"""
    return json.loads(raw)
