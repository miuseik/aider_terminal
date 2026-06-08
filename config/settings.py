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
from utils.common_utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)

# =========================== 默认配置 ===========================

DEFAULT_CONFIG = {
    "network": {
        "websocket_port": 8442,
        "host_ip": "0.0.0.0"
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
        "reference_poses_file": "config/reference_poses.json",
        "position_error_threshold": 0.001,
        "hysteresis_threshold": 0.01,
        "movement_penalty_weight": 0.01
    },
    "aloha": {
        "enabled": True,
        "initial_height": 0.3
    }
}

def load_config(config_path: str = "config/config.yaml") -> dict:
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

def save_config(config: dict, config_path: str = "config/config.yaml"):
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

# 各机器人类型的关节/URDF 配置
_ROBOT_TYPE_CONFIGS = {
    "aider": {
        "num_joints": 8,
        "num_ik_joints": 8,
        "wrist_flex_index": 5,
        "wrist_roll_index": 4,
        "wrist_yaw_index": 6,
        "gripper_index": 7,
        "joint_names": ["arm1", "arm2", "arm3", "arm4", "arm5", "arm6", "arm7", "arm8"],
        "arm_joint_names_left": [f"left_arm{i}" for i in range(1, 9)],
        "arm_joint_names_right": [f"right_arm{i}" for i in range(1, 9)],
        "urdf_path": _config_data.get("paths", {}).get("aider_urdf", "URDF/aider/urdf/aider_pro.SLDASM.urdf"),
        "end_effector_link_name": "left_arm8",
        "common_motors": {
            "arm1": [1, "sts3215"], "arm2": [2, "sts3215"], "arm3": [3, "sts3215"],
            "arm4": [4, "sts3215"], "arm5": [5, "sts3215"], "arm6": [6, "sts3215"],
            "arm7": [7, "sts3215"], "arm8": [8, "sts3215"],
        },
        "urdf_to_internal_name_map": {
            "left_arm1": "arm1", "left_arm2": "arm2", "left_arm3": "arm3",
            "left_arm4": "arm4", "left_arm5": "arm5", "left_arm6": "arm6",
            "left_arm7": "arm7", "left_arm8": "arm8",
            "right_arm1": "arm1", "right_arm2": "arm2", "right_arm3": "arm3",
            "right_arm4": "arm4", "right_arm5": "arm5", "right_arm6": "arm6",
            "right_arm7": "arm7", "right_arm8": "arm8",
        },
        "initial_left_arm": [0, -100, 100, 60, 0, 0, 0, 0],
        "initial_right_arm": [0, -100, 100, 60, 0, 0, 0, 0],
        # 关节限位 (度), 参考人体结构
        "joint_limits_deg": {
            # arm1: 肩部水平旋转 (X轴) — 人体外展/内收约 -90°~90°
            "arm1": {"lower": -90, "upper": 90},
            # arm2: 肩部上举 (Y轴) — 人体屈曲/伸展约 -150°~30°
            "arm2": {"lower": -150, "upper": 30},
            # arm3: 肘部弯曲 (Z轴) — 0°=直臂, 负=曲臂, 人体肘屈约 0°~145°
            "arm3": {"lower": -150, "upper": 30},
            # arm4: 前臂旋转 (X轴) — 人体旋前/旋后约 ±90°
            "arm4": {"lower": -90, "upper": 90},
            # arm5: 腕部翻滚 (Z轴) — 人体桡偏/尺偏约 -30°~40°, 机械放宽
            "arm5": {"lower": -45, "upper": 45},
            # arm6: 腕部俯仰 (X轴) — 人体腕屈/腕伸约 -80°~70°
            "arm6": {"lower": -70, "upper": 70},
            # arm7: 腕部偏航 (Y轴) — 人体腕部独立偏航范围小, 放宽至 ±45°
            "arm7": {"lower": -45, "upper": 45},
            # arm8: 夹爪 (X轴) — 张开=0°, 闭合=-90°
            "arm8": {"lower": -90, "upper": 0},
        },
        # 身体关节限位
        "body_joint_limits": {
            "waist_Link":  {"lower_deg": -90, "upper_deg": 90},
            "head_Link":   {"lower_deg": -60, "upper_deg": 60},
            "head_Link2":  {"lower_deg": -30, "upper_deg": 45},
            "lift_Link":   {"lower_m":    0.0, "upper_m":  0.5},
        },
    },
    "aloha": {
        "num_joints": 6,
        "num_ik_joints": 6,
        "wrist_flex_index": 3,
        "wrist_roll_index": 4,
        "wrist_yaw_index": 0,
        "gripper_index": 5,
        "joint_names": ["shoulder_pan", "shoulder_lift", "elbow_flex",
                       "wrist_flex", "wrist_roll", "gripper"],
        "arm_joint_names_left": ["1", "2", "3", "4", "5", "6"],
        "arm_joint_names_right": ["1", "2", "3", "4", "5", "6"],
        "urdf_path": _config_data.get("paths", {}).get("so100_urdf", "URDF/SO100/so100.urdf"),
        "aloha_urdf_path": _config_data.get("paths", {}).get("aloha_urdf", "URDF/aloha/Aloha.urdf"),
        "end_effector_link_name": "Fixed_Jaw_tip_joint",
        "common_motors": {
            "shoulder_pan": [1, "sts3215"], "shoulder_lift": [2, "sts3215"],
            "elbow_flex": [3, "sts3215"], "wrist_flex": [4, "sts3215"],
            "wrist_roll": [5, "sts3215"], "gripper": [6, "sts3215"],
        },
        "urdf_to_internal_name_map": {
            "1": "shoulder_pan", "2": "shoulder_lift",
            "3": "elbow_flex", "4": "wrist_flex",
            "5": "wrist_roll", "6": "gripper",
        },
        "initial_left_arm": [0, -100, 100, 60, 0, 0],
        "initial_right_arm": [0, -100, 100, 60, 0, 0],
    },
}

# 当前活跃的机器人类型（由 set_robot_type() 设置）
_ROBOT_TYPE: str = _config_data.get("robot", {}).get("type", "aider")


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
    if robot_type not in _ROBOT_TYPE_CONFIGS:
        print(f"⚠️ 未知机器人类型 '{robot_type}'，使用默认 'aider'")
        robot_type = "aider"

    _ROBOT_TYPE = robot_type
    cfg = _ROBOT_TYPE_CONFIGS[robot_type]

    NUM_JOINTS = cfg["num_joints"]
    NUM_IK_JOINTS = cfg["num_ik_joints"]
    WRIST_FLEX_INDEX = cfg["wrist_flex_index"]
    WRIST_ROLL_INDEX = cfg["wrist_roll_index"]
    WRIST_YAW_INDEX = cfg.get("wrist_yaw_index", 6)
    GRIPPER_INDEX = cfg["gripper_index"]
    JOINT_NAMES = cfg["joint_names"]
    ARM_JOINT_NAMES_LEFT = cfg["arm_joint_names_left"]
    ARM_JOINT_NAMES_RIGHT = cfg["arm_joint_names_right"]
    URDF_PATH = cfg["urdf_path"]
    ALOHA_URDF_PATH = cfg.get("aloha_urdf_path", "URDF/aloha/Aloha.urdf")
    END_EFFECTOR_LINK_NAME = cfg["end_effector_link_name"]
    COMMON_MOTORS = cfg["common_motors"]
    URDF_TO_INTERNAL_NAME_MAP = cfg["urdf_to_internal_name_map"]

    print(f"✅ 机器人类型已设为: {robot_type} (NUM_JOINTS={NUM_JOINTS}, IK={NUM_IK_JOINTS})")


def get_robot_type() -> str:
    return _ROBOT_TYPE


def get_robot_initial_arm(arm: str) -> list:
    cfg = _ROBOT_TYPE_CONFIGS.get(_ROBOT_TYPE, _ROBOT_TYPE_CONFIGS["aider"])
    return cfg[f"initial_{arm}_arm"]


def get_joint_limits_deg() -> dict:
    """返回当前机器人类型的关节限位字典 {joint_name: {lower, upper}} (度)。"""
    cfg = _ROBOT_TYPE_CONFIGS.get(_ROBOT_TYPE, _ROBOT_TYPE_CONFIGS["aider"])
    return cfg.get("joint_limits_deg", {})


def get_body_joint_limits() -> dict:
    """返回身体关节限位字典。"""
    cfg = _ROBOT_TYPE_CONFIGS.get(_ROBOT_TYPE, _ROBOT_TYPE_CONFIGS["aider"])
    return cfg.get("body_joint_limits", {})


# --- 默认初始化 (aider) ---
# 模块加载时设默认值，main.py 会调用 set_robot_type() 覆盖
NUM_JOINTS = _ROBOT_TYPE_CONFIGS["aider"]["num_joints"]
NUM_IK_JOINTS = _ROBOT_TYPE_CONFIGS["aider"]["num_ik_joints"]
WRIST_FLEX_INDEX = _ROBOT_TYPE_CONFIGS["aider"]["wrist_flex_index"]
WRIST_ROLL_INDEX = _ROBOT_TYPE_CONFIGS["aider"]["wrist_roll_index"]
WRIST_YAW_INDEX = _ROBOT_TYPE_CONFIGS["aider"].get("wrist_yaw_index", 6)
GRIPPER_INDEX = _ROBOT_TYPE_CONFIGS["aider"]["gripper_index"]
JOINT_NAMES = _ROBOT_TYPE_CONFIGS["aider"]["joint_names"]
ARM_JOINT_NAMES_LEFT = _ROBOT_TYPE_CONFIGS["aider"]["arm_joint_names_left"]
ARM_JOINT_NAMES_RIGHT = _ROBOT_TYPE_CONFIGS["aider"]["arm_joint_names_right"]
URDF_PATH = _ROBOT_TYPE_CONFIGS["aider"]["urdf_path"]
ALOHA_URDF_PATH = _ROBOT_TYPE_CONFIGS["aloha"].get("aloha_urdf_path", "URDF/aloha/Aloha.urdf")
END_EFFECTOR_LINK_NAME = _ROBOT_TYPE_CONFIGS["aider"]["end_effector_link_name"]
COMMON_MOTORS = _ROBOT_TYPE_CONFIGS["aider"]["common_motors"]
URDF_TO_INTERNAL_NAME_MAP = _ROBOT_TYPE_CONFIGS["aider"]["urdf_to_internal_name_map"]

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
    video_source: str = "/dev/video0"
    
    # 机器人设置
    robot_type: str = _config_data.get("robot", {}).get("type", "aider")
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
    """获取 API 地址，优先级：命令行参数 > 环境变量 > 配置默认值"""
    import sys
    # 1. 检查命令行参数
    if '--api-host' in sys.argv:
        idx = sys.argv.index('--api-host')
        if idx + 1 < len(sys.argv):
            return sys.argv[idx + 1]
    if '--env-dev' in sys.argv:
        return 'localhost'
    # 2. 返回配置中的值（支持环境变量覆盖）
    return config.api_host 
