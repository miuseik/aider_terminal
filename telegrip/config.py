"""
统一遥操作系统的配置模块。
从 config.yaml 文件加载配置，并提供默认值回退。
"""

import os
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
from pathlib import Path
import logging
from .utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)

# 默认配置值（如果 YAML 文件不存在则使用）
DEFAULT_CONFIG = {
    "network": {
        "websocket_port": 8442,
        "host_ip": "0.0.0.0"
    },
    "robot": {
        "left_arm": {
            "name": "Left Arm",
            "port": "/dev/ttyACM0",
            "enabled": True
        },
        "right_arm": {
            "name": "Right Arm",
            "port": "/dev/ttyACM1",
            "enabled": True
        },
        "vr_to_robot_scale": 1.0,
        "send_interval": 0.05,
    },
    "control": {
        "keyboard": {
            "enabled": True,
            "pos_step": 0.01,
            "angle_step": 5.0,
            "gripper_step": 10.0
        },
        "vr": {
            "enabled": True
        },
        "pybullet": {
            "enabled": True
        }
    },
    "paths": {
        "urdf_path": "URDF/SO100/so100.urdf"
    },
    "gripper": {
        "open_angle": 0.0,
        "closed_angle": 45.0
    },
    "ik": {
        "use_reference_poses": True,
        "reference_poses_file": "reference_poses.json",
        "position_error_threshold": 0.001,
        "hysteresis_threshold": 0.01,
        "movement_penalty_weight": 0.01
    },
    "aloha": {
        "enabled": True,
        "initial_height": 0.3
    }
}

def load_config(config_path: str = "config.yaml") -> dict:
    """从 YAML 文件加载配置，回退到默认值。"""
    config = DEFAULT_CONFIG.copy()
    
    # 首先尝试从项目根目录加载（包安装目录）
    package_config_path = get_absolute_path(config_path)
    
    # 检查包目录中是否存在配置
    if package_config_path.exists():
        config_file_to_use = package_config_path
        logger.info(f"Loading config from package directory: {config_file_to_use}")
    # 回退到当前工作目录（用于用户提供的配置）
    elif os.path.exists(config_path):
        config_file_to_use = Path(config_path)
        logger.info(f"Loading config from current directory: {config_file_to_use}")
    else:
        logger.info(f"Config file {config_path} not found in package directory ({package_config_path}) or current directory, using defaults")
        return config
    
    try:
        with open(config_file_to_use, 'r', encoding='utf-8') as f:
            yaml_config = yaml.safe_load(f)
            if yaml_config:
                # 将 yaml 配置深度合并到默认配置中
                _deep_merge(config, yaml_config)
    except Exception as e:
        logger.warning(f"Could not load config from {config_file_to_use}: {e}")
        logger.info("Using default configuration")
    
    return config

def save_config(config: dict, config_path: str = "config.yaml"):
    """将配置保存到项目根目录的 YAML 文件。"""
    # 始终保存到项目根目录
    abs_config_path = get_absolute_path(config_path)
    try:
        with open(abs_config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Error saving config to {abs_config_path}: {e}")
        return False

def _deep_merge(base: dict, update: dict):
    """将 update 字典深度合并到 base 字典中。"""
    for key, value in update.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value

# 加载配置
_config_data = load_config()

# 提取值以保持向后兼容性
WEBSOCKET_PORT = _config_data["network"]["websocket_port"]
HOST_IP = _config_data["network"]["host_ip"]

VR_TO_ROBOT_SCALE = _config_data["robot"]["vr_to_robot_scale"]
SEND_INTERVAL = _config_data["robot"]["send_interval"]

POS_STEP = _config_data["control"]["keyboard"]["pos_step"]
ANGLE_STEP = _config_data["control"]["keyboard"]["angle_step"]
GRIPPER_STEP = _config_data["control"]["keyboard"]["gripper_step"]

URDF_PATH = _config_data["paths"]["urdf_path"]

GRIPPER_OPEN_ANGLE = _config_data["gripper"]["open_angle"]
GRIPPER_CLOSED_ANGLE = _config_data["gripper"]["closed_angle"]

# IK 配置
USE_REFERENCE_POSES = _config_data["ik"]["use_reference_poses"]
REFERENCE_POSES_FILE = _config_data["ik"]["reference_poses_file"]
IK_POSITION_ERROR_THRESHOLD = _config_data["ik"]["position_error_threshold"]
IK_HYSTERESIS_THRESHOLD = _config_data["ik"]["hysteresis_threshold"]
IK_MOVEMENT_PENALTY_WEIGHT = _config_data["ik"]["movement_penalty_weight"]

# Aloha 配置
ALOHA_ENABLED = _config_data["aloha"]["enabled"]
ALOHA_INITIAL_HEIGHT = _config_data["aloha"]["initial_height"]

# --- 关节配置 ---
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 3  # Use only first 3 joints for IK (Rotation, Pitch, Elbow)
WRIST_FLEX_INDEX = 3
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# SO100 电机配置
COMMON_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"], 
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# URDF 关节名称映射
URDF_TO_INTERNAL_NAME_MAP = {
    "1": "shoulder_pan",
    "2": "shoulder_lift",
    "3": "elbow_flex",
    "4": "wrist_flex",
    "5": "wrist_roll",
    "6": "gripper",
}

# --- PyBullet 配置 ---
END_EFFECTOR_LINK_NAME = "Fixed_Jaw_tip"

# --- 键盘控制 ---
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees

# --- 设备端口 ---
DEFAULT_FOLLOWER_PORTS = {
    "left": _config_data["robot"]["left_arm"]["port"],
    "right": _config_data["robot"]["right_arm"]["port"],
    "left_servo_type": _config_data["robot"]["left_arm"].get("servo_type", "st3215"),
    "left_baudrate": _config_data["robot"]["left_arm"].get("baudrate", 1000000),
    "right_servo_type": _config_data["robot"]["right_arm"].get("servo_type", "st3215"),
    "right_baudrate": _config_data["robot"]["right_arm"].get("baudrate", 1000000)
}

@dataclass
class TelegripConfig:
    """遥操作系统的主配置类。"""
    
    # 网络设置
    websocket_port: int = WEBSOCKET_PORT
    host_ip: str = HOST_IP
    server_host: str = os.getenv("TELEGRIP_SERVER_HOST", "ws.houqicg.com")  # Aider Server 地址
    enable_webrtc: bool = True  # 启用 WebRTC 视频推流
    video_source: str = "/dev/video0"  # 摄像头设备路径
    
    # 机器人设置
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    send_interval: float = SEND_INTERVAL
    
    # 设备端口
    follower_ports: Dict[str, str] = None
    
    # 控制标志
    enable_pybullet: bool = True
    enable_pybullet_gui: bool = True
    enable_robot: bool = True
    enable_vr: bool = True
    enable_keyboard: bool = True
    autoconnect: bool = False
    log_level: str = "warning"
    
    # 路径
    urdf_path: str = URDF_PATH
    webapp_dir: str = "webapp"
    
    # IK 设置
    use_reference_poses: bool = USE_REFERENCE_POSES
    reference_poses_file: str = REFERENCE_POSES_FILE
    ik_position_error_threshold: float = IK_POSITION_ERROR_THRESHOLD
    ik_hysteresis_threshold: float = IK_HYSTERESIS_THRESHOLD
    ik_movement_penalty_weight: float = IK_MOVEMENT_PENALTY_WEIGHT
    
    # 夹爪设置
    gripper_open_angle: float = GRIPPER_OPEN_ANGLE
    gripper_closed_angle: float = GRIPPER_CLOSED_ANGLE
    
    # 键盘控制
    pos_step: float = POS_STEP
    angle_step: float = ANGLE_STEP
    gripper_step: float = GRIPPER_STEP
    
    # Aloha 设置
    aloha_enabled: bool = ALOHA_ENABLED
    aloha_initial_height: float = ALOHA_INITIAL_HEIGHT
    
    def __post_init__(self):
        # 如果未设置则初始化 follower_ports
        if self.follower_ports is None:
            self.follower_ports = {
                "left": _config_data["robot"]["left_arm"]["port"],
                "right": _config_data["robot"]["right_arm"]["port"]
            }
        
        # 确保端口不为 None
        if self.follower_ports["left"] is None:
            self.follower_ports["left"] = "/dev/ttyACM0"
        if self.follower_ports["right"] is None:
            self.follower_ports["right"] = "/dev/ttyACM1"
    
    @property
    def urdf_exists(self) -> bool:
        """检查 URDF 文件是否存在。"""
        urdf_path = get_absolute_path(self.urdf_path)
        return urdf_path.exists()
    
    @property
    def webapp_exists(self) -> bool:
        """检查 webapp 目录是否存在。"""
        webapp_path = get_absolute_path(self.webapp_dir)
        return webapp_path.exists()
    
    def get_absolute_urdf_path(self) -> str:
        """获取 URDF 文件的绝对路径。"""
        return str(get_absolute_path(self.urdf_path))
    
    def get_absolute_reference_poses_path(self) -> str:
        """获取参考姿态文件的绝对路径。"""
        return str(get_absolute_path(self.reference_poses_file))

def get_config_data():
    """获取当前配置数据。"""
    return _config_data.copy()

def update_config_data(new_config: dict):
    """更新全局配置数据。"""
    global _config_data
    _config_data = new_config
    
    # 保存到文件
    save_config(_config_data)

# 全局配置实例
config = TelegripConfig() 
