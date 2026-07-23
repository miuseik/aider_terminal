"""
统一遥操作系统的配置模块。
从 config.yaml 文件加载配置，并提供默认值回退。
支持多种机器人类型: aider (8-DOF), aloha (6-DOF via SO100)。
"""

import os
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
from pathlib import Path
import logging
from aiderminal.utils.common_utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)


# =========================== .env 加载 ===========================

def _load_env_file() -> None:
    """根据 ENV 环境变量加载对应的 .env 文件，设置 os.environ。

    ENV=dev  → env/.env.development
    其他/未设 → env/.env.production

    在 TelegripConfig 定义前调用，确保 os.getenv() 读取到正确的值。
    不会覆盖已存在的环境变量（命令行设置的优先级最高）。
    """
    env_name = os.getenv("ENV", "pro")
    env_file_name = ".env.development" if env_name == "dev" else ".env.production"
    env_file = get_project_root() / "env" / env_file_name

    if not env_file.exists():
        logger.debug("未找到 .env 文件: %s，使用已有环境变量或默认值", env_file)
        return

    try:
        with open(env_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    continue
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                if key and key not in os.environ:
                    os.environ[key] = value
                    logger.debug("从 %s 加载: %s=%s", env_file_name, key, value)
    except Exception as e:
        logger.warning("加载 .env 文件失败 (%s): %s", env_file, e)


_load_env_file()

# =========================== 默认配置 ===========================

DEFAULT_CONFIG = {
    "network": {
        "websocket_port": 8442,
        "host_ip": "0.0.0.0",
        "enable_webrtc": True,
        "webrtc_room_id": "robot-camera",
        "video_source": "/dev/video0",
        "camera_width": 640,
        "camera_height": 480,
        "camera_fps": 25,
        "camera_fourcc": "MJPG",
        "audio_enabled": True,
        "audio_device": None,
        "audio_sample_rate": 16000,
    },
    "robot": {
        "type": "aider",  # 机器人类型: aider, aloha, openarmx, custom
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
        "send_interval": 0.02,
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
        "urdf_path": "URDF/aider/urdf/aider_pro.SLDASM.urdf",
        "aider_urdf": "URDF/aider/urdf/aider_pro.SLDASM.urdf",
        "aloha_urdf": "URDF/aloha/Aloha.urdf",
        "so100_urdf": "URDF/SO100/so100.urdf",
    },
    "gripper": {
        "open_angle": 0.0,
        "closed_angle": 45.0
    },
    "ik": {
        "use_reference_poses": True,
        "reference_poses_file": "aiderminal/config/reference_poses.json",
        "position_error_threshold": 0.001,
        "hysteresis_threshold": 0.01,
        "movement_penalty_weight": 0.01
    },
    "aloha": {
        "enabled": True,
        "initial_height": 0.3
    }
}

def load_config(config_path: str = "aiderminal/config/config.yaml") -> dict:
    """从 YAML 文件加载配置，回退到默认值。"""
    config = DEFAULT_CONFIG.copy()
    
    # 首先尝试从项目根目录加载（包安装目录）
    package_config_path = get_absolute_path(config_path)
    
    # 检查包目录中是否存在配置
    if package_config_path.exists():
        config_file_to_use = package_config_path
        print(f"Loading config from package directory: {config_file_to_use}")
    # 回退到当前工作目录（用于用户提供的配置）
    elif os.path.exists(config_path):
        config_file_to_use = Path(config_path)
        print(f"Loading config from current directory: {config_file_to_use}")
    else:
        print(f"Config file {config_path} not found in package directory ({package_config_path}) or current directory, using defaults")
        return config
    
    try:
        with open(config_file_to_use, 'r', encoding='utf-8') as f:
            yaml_config = yaml.safe_load(f)
            if yaml_config:
                # 将 yaml 配置深度合并到默认配置中
                _deep_merge(config, yaml_config)
    except Exception as e:
        print(f"Could not load config from {config_file_to_use}: {e}")
        print("Using default configuration")
    
    return config

def save_config(config: dict, config_path: str = "aiderminal/config/config.yaml"):
    """将配置保存到项目根目录的 YAML 文件。"""
    # 始终保存到项目根目录
    abs_config_path = get_absolute_path(config_path)
    try:
        with open(abs_config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        print(f"Error saving config to {abs_config_path}: {e}")
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

# =========================== 机器人类型相关常量 ===========================

# 当前活跃的机器人类型（由 set_robot_type() 设置）
_ROBOT_TYPE: str = _config_data.get("robot", {}).get("type", "aider")


def _get_robot_settings(robot_type: str = None):
    """动态导入机器人类型的私有 settings 模块。"""
    rt = robot_type or _ROBOT_TYPE
    return __import__(f"aiderminal.robots.{rt}.settings", fromlist=["*"])


def set_robot_type(robot_type: str) -> None:
    """切换机器人类型并更新所有模块级常量。

    必须在其他模块 import settings 的常量之前调用。
    典型用法: 在 main.py 中解析 --robot-type 后立即调用。
    """
    global _ROBOT_TYPE, \
        NUM_JOINTS, NUM_IK_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, WRIST_YAW_INDEX, GRIPPER_INDEX, \
        JOINT_NAMES, ARM_JOINT_NAMES_LEFT, ARM_JOINT_NAMES_RIGHT, \
        URDF_PATH, ALOHA_URDF_PATH, END_EFFECTOR_LINK_NAME, \
        COMMON_MOTORS, URDF_TO_INTERNAL_NAME_MAP

    robot_type = robot_type.lower()
    try:
        robot = _get_robot_settings(robot_type)
    except (ImportError, ModuleNotFoundError):
        print(f"⚠️ 未知机器人类型 '{robot_type}'，使用默认 'aider'")
        robot_type = "aider"
        robot = _get_robot_settings("aider")

    _ROBOT_TYPE = robot_type

    NUM_JOINTS = robot.NUM_JOINTS
    NUM_IK_JOINTS = getattr(robot, "NUM_IK_JOINTS", robot.NUM_JOINTS)
    WRIST_FLEX_INDEX = robot.WRIST_FLEX_INDEX
    WRIST_ROLL_INDEX = robot.WRIST_ROLL_INDEX
    WRIST_YAW_INDEX = getattr(robot, "WRIST_YAW_INDEX", 6)
    GRIPPER_INDEX = robot.GRIPPER_INDEX
    JOINT_NAMES = robot.JOINT_NAMES
    ARM_JOINT_NAMES_LEFT = robot.ARM_JOINT_NAMES_LEFT
    ARM_JOINT_NAMES_RIGHT = robot.ARM_JOINT_NAMES_RIGHT
    URDF_PATH = robot.URDF_PATH
    ALOHA_URDF_PATH = getattr(robot, "ALOHA_URDF_PATH", "URDF/aloha/Aloha.urdf")
    END_EFFECTOR_LINK_NAME = robot.END_EFFECTOR_LINK_NAME
    COMMON_MOTORS = robot.COMMON_MOTORS
    URDF_TO_INTERNAL_NAME_MAP = robot.URDF_TO_INTERNAL_NAME_MAP

    print(f"✅ 机器人类型已设为: {robot_type} (NUM_JOINTS={NUM_JOINTS}, IK={NUM_IK_JOINTS})")


def get_robot_type() -> str:
    return _ROBOT_TYPE


def get_robot_urdf_path(robot_type: str = None) -> str:
    """返回指定（或当前）机器人类型的 URDF 路径。"""
    robot = _get_robot_settings(robot_type)
    return robot.URDF_PATH


def get_robot_aloha_urdf_path() -> str:
    """返回 Aloha URDF 路径（需 aloha 机器人类型）。"""
    robot = _get_robot_settings("aloha")
    return getattr(robot, "ALOHA_URDF_PATH", "URDF/aloha/Aloha.urdf")


def get_robot_initial_arm(arm: str) -> list:
    robot = _get_robot_settings()
    return getattr(robot, f"INITIAL_{arm.upper()}_ARM")


def get_robot_poses() -> dict:
    """返回当前机器人类型的姿态预设字典 {pose_name: {left: [...], right: [...]}}。"""
    robot = _get_robot_settings()
    return getattr(robot, "POSES", {})


def get_default_pose_name() -> str:
    """返回默认启动姿态名（下拉框预设之一，单一数据源）。"""
    robot = _get_robot_settings()
    return getattr(robot, "DEFAULT_POSE_NAME", "default")


def get_joint_limits_deg() -> dict:
    """返回当前机器人类型的关节限位字典 {joint_name: {lower, upper}} (度)。"""
    robot = _get_robot_settings()
    return getattr(robot, "JOINT_LIMITS_DEG", {})


def get_body_joint_limits() -> dict:
    """返回身体关节限位字典。"""
    robot = _get_robot_settings()
    return getattr(robot, "BODY_JOINT_LIMITS", {})


# --- 默认初始化 (aider) ---
# 模块加载时设默认值，main.py/teminal_node.py 会调用 set_robot_type() 覆盖。
# 注意：不能用 __import__("robots/aider/settings")，因为 robots/aider/__init__.py
#   会 import adapter -> adapter import TelegripConfig from settings → 循环导入。
NUM_JOINTS = 8
NUM_IK_JOINTS = 8
WRIST_FLEX_INDEX = 5
WRIST_ROLL_INDEX = 4
WRIST_YAW_INDEX = 6
GRIPPER_INDEX = 7
JOINT_NAMES = ["arm1", "arm2", "arm3", "arm4", "arm5", "arm6", "arm7", "arm8"]
ARM_JOINT_NAMES_LEFT = ["left_arm1", "left_arm2", "left_arm3", "left_arm4",
                         "left_arm5", "left_arm6", "left_arm7", "left_arm8"]
ARM_JOINT_NAMES_RIGHT = ["right_arm1", "right_arm2", "right_arm3", "right_arm4",
                          "right_arm5", "right_arm6", "right_arm7", "right_arm8"]
URDF_PATH = _config_data.get("paths", {}).get("aider_urdf", "URDF/aider/urdf/aider_pro.SLDASM.urdf")
ALOHA_URDF_PATH = _config_data.get("paths", {}).get("aloha_urdf", "URDF/aloha/Aloha.urdf")
END_EFFECTOR_LINK_NAME = "left_arm8"
COMMON_MOTORS = {
    "arm1": [1, "sts3215"], "arm2": [2, "sts3215"], "arm3": [3, "sts3215"],
    "arm4": [4, "sts3215"], "arm5": [5, "sts3215"], "arm6": [6, "sts3215"],
    "arm7": [7, "sts3215"], "arm8": [8, "sts3215"],
}
URDF_TO_INTERNAL_NAME_MAP = {
    "left_arm1": "arm1", "left_arm2": "arm2", "left_arm3": "arm3",
    "left_arm4": "arm4", "left_arm5": "arm5", "left_arm6": "arm6",
    "left_arm7": "arm7", "left_arm8": "arm8",
    "right_arm1": "arm1", "right_arm2": "arm2", "right_arm3": "arm3",
    "right_arm4": "arm4", "right_arm5": "arm5", "right_arm6": "arm6",
    "right_arm7": "arm7", "right_arm8": "arm8",
}

# 夹爪通用配置
GRIPPER_OPEN_ANGLE = _config_data["gripper"]["open_angle"]
GRIPPER_CLOSED_ANGLE = _config_data["gripper"]["closed_angle"]

# IK 通用配置
USE_REFERENCE_POSES = _config_data["ik"]["use_reference_poses"]
REFERENCE_POSES_FILE = _config_data["ik"]["reference_poses_file"]
IK_POSITION_ERROR_THRESHOLD = _config_data["ik"]["position_error_threshold"]
IK_HYSTERESIS_THRESHOLD = _config_data["ik"]["hysteresis_threshold"]
IK_MOVEMENT_PENALTY_WEIGHT = _config_data["ik"]["movement_penalty_weight"]

# Aloha 通用配置
ALOHA_ENABLED = _config_data["aloha"]["enabled"]
ALOHA_INITIAL_HEIGHT = _config_data["aloha"]["initial_height"]

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
    server_host: str = os.getenv("TELEGRIP_SERVER_HOST", "ws.houqicg.com")
    api_host: str = os.getenv("TELEGRIP_API_HOST", "www.houqicg.com")
    enable_webrtc: bool = True
    webrtc_room_id: str = "robot-camera"
    ice_servers: list = None
    video_source: str = "/dev/video0"
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 25
    camera_fourcc: str = "MJPG"
    audio_enabled: bool = True
    audio_device: str = None
    audio_sample_rate: int = 16000
    audio_output_sample_rate: int = 48000
    
    # 机器人设置
    robot_type: str = _config_data.get("robot", {}).get("type", "aider")
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    send_interval: float = SEND_INTERVAL
    
    # 设备端口
    follower_ports: Dict[str, str] = None
    
    # 控制标志
    enable_pybullet: bool = True
    enable_pybullet_gui: bool = True
    enable_vr: bool = True
    enable_keyboard: bool = True
    autoconnect: bool = False
    log_level: str = "warning"
    
    # 路径
    urdf_path: str = URDF_PATH
    aloha_urdf_path: str = ALOHA_URDF_PATH
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
    
    # 键盘控制 (复用 IK 中的配置)
    
    # Aloha 设置
    aloha_enabled: bool = ALOHA_ENABLED
    aloha_initial_height: float = ALOHA_INITIAL_HEIGHT
    
    def __post_init__(self):
        # 从 config.yaml 读取 WebRTC / 摄像头配置
        net = _config_data.get("network", {})
        self.enable_webrtc = net.get("enable_webrtc", self.enable_webrtc)
        self.webrtc_room_id = net.get("webrtc_room_id", self.webrtc_room_id)
        self.ice_servers = net.get("ice_servers", [])
        self.video_source = net.get("video_source", self.video_source)
        self.camera_width = int(net.get("camera_width", self.camera_width))
        self.camera_height = int(net.get("camera_height", self.camera_height))
        self.camera_fps = int(net.get("camera_fps", self.camera_fps))
        self.camera_fourcc = net.get("camera_fourcc", self.camera_fourcc)
        self.audio_enabled = net.get("audio_enabled", self.audio_enabled)
        self.audio_device = net.get("audio_device", self.audio_device)
        self.audio_sample_rate = int(net.get("audio_sample_rate", self.audio_sample_rate))
        self.audio_output_sample_rate = int(net.get("audio_output_sample_rate", self.audio_output_sample_rate))

        if self.follower_ports is None:
            self.follower_ports = {
                "left": _config_data["robot"]["left_arm"]["port"],
                "right": _config_data["robot"]["right_arm"]["port"]
            }
        if self.follower_ports["left"] is None:
            self.follower_ports["left"] = "/dev/ttyACM0"
        if self.follower_ports["right"] is None:
            self.follower_ports["right"] = "/dev/ttyACM1"
    
    @property
    def urdf_exists(self) -> bool:
        urdf_path = get_absolute_path(self.urdf_path)
        return urdf_path.exists()
    
    @property
    def webapp_exists(self) -> bool:
        webapp_path = get_absolute_path(self.webapp_dir)
        return webapp_path.exists()
    
    def get_absolute_urdf_path(self) -> str:
        return str(get_absolute_path(self.urdf_path))
    
    def get_absolute_aloha_urdf_path(self) -> str:
        return str(get_absolute_path(self.aloha_urdf_path))
    
    def get_absolute_reference_poses_path(self) -> str:
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

def get_api_endpoint():
    """获取 API 地址，优先级：命令行参数 > 环境变量(.env) > 配置默认值"""
    import sys
    if '--api-host' in sys.argv:
        idx = sys.argv.index('--api-host')
        if idx + 1 < len(sys.argv):
            return sys.argv[idx + 1]
    return config.api_host 
