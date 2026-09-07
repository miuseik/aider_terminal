"""WebSocket 消息协议 - 负责消息的编码和解码"""
import json
import math
from typing import Dict, Any, List


def _sanitize(obj):
    """递归把 NaN/Inf 换成 None (null)，避免生成非法 JSON (浏览器 JSON.parse 不认 NaN)。"""
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return obj
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    return obj


def encode_message(data: Dict[str, Any]) -> str:
    """将字典编码为 JSON 字符串 (清洗 NaN/Inf)"""
    return json.dumps(_sanitize(data))


def decode_message(raw: str) -> Dict[str, Any]:
    """将 JSON 字符串解码为字典"""
    return json.loads(raw)
