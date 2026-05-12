"""
通用电机控制器 - 品牌无关的抽象层

职责:
1. 硬件发现与扫描（支持多品牌混搭）
2. 舵机信息管理（注册、查询）
3. 统一控制接口（位置、速度、扭矩）
4. 批量同步控制
5. 状态监控与诊断
6. 自动路由到对应品牌驱动

设计原则:
- 不关心具体品牌（Feetech/Robstride/Damiao）
- 不关心具体关节名称
- 不关心串口号
- 只做数据处理和分发
- 具体实现由各品牌驱动负责
"""

import time
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, asdict


@dataclass
class ServoInfo:
    """舵机信息"""
    port: str                    # 串口号
    servo_id: int                # 舵机ID
    brand: str                   # 品牌: 'feetech', 'robstride', 'damiao'
    model: str = ""              # 型号
    joint_name: str = ""         # 关节名称（可选）
    is_online: bool = True       # 是否在线
    firmware_version: str = ""   # 固件版本


@dataclass
class ServoStatus:
    """舵机状态"""
    position: float = 0.0        # 当前位置（度）
    velocity: float = 0.0        # 当前速度（度/秒）
    current: float = 0.0         # 当前电流（A）
    temperature: float = 0.0     # 温度（°C）
    voltage: float = 0.0         # 电压（V）
    load: float = 0.0           # 负载（%）
    moving: bool = False         # 是否在运动


class MotorController:
    """
    通用电机控制器
    
    特点:
    - 品牌无关：支持 Feetech/Robstride/Damiao 混搭
    - 自动路由：根据端口和品牌自动选择驱动
    - 统一管理：提供一致的控制接口
    """
    
    def __init__(self, config=None, robot_interface=None):
        """
        初始化电机控制器
        
        Args:
            config: 配置信息(可选，兼容旧代码)
            robot_interface: 机器人接口实例(兼容旧代码)
        """
        # 驱动实例池: {port: driver_instance}
        self.drivers = {}
        
        # 舵机注册表: {(port, servo_id): ServoInfo}
        self.servo_registry = {}
        
        # 关节名称映射: {joint_name: (port, servo_id)}
        self.joint_map = {}
        
        # 兼容旧代码的参数
        self.config = config
        self.robot_interface = robot_interface
        
        # 电机名称到索引的映射（兼容旧代码）
        self.motor_index_map = {
            'shoulder_pan': 0,
            'shoulder_lift': 1,
            'elbow_flex': 2,
            'wrist_flex': 3,
            'wrist_roll': 4,
            'gripper': 5
        }
        
        # 缓存从 Server 获取的舵机配置（避免重复 API 调用）
        self._servo_config_cache = None
        self._load_servo_config()
        
        # 初始化底层驱动（如果传入配置，兼容旧代码）
        self.driver = None
        if config:
            self._initialize_driver(config)
        
        print("✅ 通用电机控制器初始化完成")
    
    def _load_servo_config(self):
        """
        从 Server API 加载舵机配置（只调用一次，缓存结果）
        """
        try:
            from api.server_api_client import ServerAPIClient
            api_client = ServerAPIClient()
            self._servo_config_cache = api_client.get_servo_ids_config()
            
            if self._servo_config_cache:
                print(f"✅ 已从 Server 加载舵机配置（缓存）")
            else:
                print(f"⚠️ 未能从 Server 获取配置，将使用默认偏移量")
        except Exception as e:
            print(f"⚠️ 加载配置失败: {e}，将使用默认偏移量")
            self._servo_config_cache = None
    
    # ==================== 1. 硬件发现与扫描 ====================
    
    def scan_available_ports(self) -> List[str]:
        """
        扫描所有可用的 USB 串口和 CAN 接口（自动启动 CAN 接口）
        
        Returns:
            List[str]: 可用端口列表 ['/dev/ttyACM0', '/dev/ttyUSB0', 'can0', ...]
        """
        import serial.tools.list_ports
        import subprocess
        
        # ✅ 自动启动 CAN 接口（如果存在但 DOWN）
        self._ensure_can_interface_up()
        
        # ✅ 使用 pyserial 的 list_ports，更准确
        ports = []
        for p in serial.tools.list_ports.comports():
            # 只保留 USB 相关的串口
            if 'USB' in p.device or 'ACM' in p.device or 'ttyUSB' in p.device:
                ports.append(p.device)
        
        # ✅ 添加 CAN 接口
        try:
            result = subprocess.run(['ip', '-o', 'link', 'show'], capture_output=True, text=True, timeout=2)
            for line in result.stdout.split('\n'):
                if 'can0' in line and 'can0' not in ports:
                    ports.insert(0, 'can0')  # 放在最前面
                    break
        except Exception as e:
            print(f"⚠️ 检查 CAN 接口失败: {e}")
        
        print(f"🔍 发现 {len(ports)} 个端口: {ports}")
        return ports
    
    def _ensure_can_interface_up(self):
        """确保 CAN 接口处于 UP 状态（全自动处理）"""
        import subprocess
        try:
            result = subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0 and 'state UP' not in result.stdout:
                subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '500000'],
                             capture_output=True, text=True, timeout=5)
        except Exception:
            pass
    
    def scan_servos_on_port(self, port: str, start_id: int = 1, end_id: int = 253) -> List[ServoInfo]:
        """在指定串口/CAN接口扫描电机（自动识别品牌）"""
        found_servos = []
        brands_to_try = ['robstride'] if 'can' in port.lower() else ['feetech']
        
        for brand in brands_to_try:
            try:
                driver = self._get_or_create_driver(port, brand)
                if not driver or not driver.is_connected:
                    continue
                
                if brand.lower() == 'robstride':
                    # ✅ 调用驱动的扫描方法
                    found_ids = driver.scan_motors(start_id, end_id)
                    for motor_id in found_ids:
                        servo_info = ServoInfo(port=port, servo_id=motor_id, brand='robstride', model='RS-00', is_online=True)
                        found_servos.append(servo_info)
                        self.servo_registry[(port, motor_id)] = servo_info
                else:
                    for servo_id in range(start_id, end_id + 1):
                        if driver.ping(servo_id):
                            model = self._get_servo_model(driver, servo_id)
                            servo_info = ServoInfo(port=port, servo_id=servo_id, brand=brand, model=model, is_online=True)
                            found_servos.append(servo_info)
                
                if found_servos:
                    break
            except Exception:
                continue
        
        return found_servos
    
    def scan_all_servos(self) -> Dict[str, List[ServoInfo]]:
        """
        全局扫描（所有串口）
        
        Returns:
            Dict[str, List[ServoInfo]]: {port: [servo_infos]}
        """
        ports = self.scan_available_ports()
        result = {}
        
        for port in ports:
            servos = self.scan_servos_on_port(port)
            if servos:
                result[port] = servos
        
        return result
    
    def discover_ports_by_ids(self, target_ids: set) -> Dict[int, str]:
        """
        根据舵机 ID 自动发现所在的端口
        
        Args:
            target_ids: 需要查找的舵机 ID 集合
            
        Returns:
            Dict[int, str]: {servo_id: port} 映射
        """
        available_ports = self.scan_available_ports()
        if not available_ports:
            return {}
        
        # ✅ 只扫描目标 ID，而不是 1-253
        id_to_port = {}
        
        for port in available_ports:
            try:
                driver = self._get_or_create_driver(port, 'feetech')
                if driver and hasattr(driver, 'connect'):
                    if driver.connect():
                        for servo_id in sorted(target_ids):
                            try:
                                if driver.ping(servo_id):
                                    id_to_port[servo_id] = port
                            except Exception:
                                pass
                        driver.disconnect()
            except Exception:
                continue
        
        return id_to_port
    
    def auto_discover_ports_from_config(self, servo_config: dict) -> tuple[dict, bool]:
        """
        根据舵机配置自动发现并更新端口
        
        Args:
            servo_config: 舵机配置字典（格式同 servo_ids.yaml）
            
        Returns:
            (updated_config, has_changes): 更新后的配置和是否有修改
        """
        # 1. 收集所有需要查找的舵机 ID
        target_ids = set()
        for bus_name, bus_config in servo_config.items():
            if not isinstance(bus_config, dict):
                continue
            for part_name, part_config in bus_config.items():
                if part_name == 'port' or not isinstance(part_config, dict):
                    continue
                for joint_name, joint_info in part_config.items():
                    if isinstance(joint_info, dict) and 'id' in joint_info:
                        target_ids.add(joint_info['id'])
        
        if not target_ids:
            return servo_config, False
        
        # 2. 发现 ID → Port 映射
        id_to_port = self.discover_ports_by_ids(target_ids)
        
        if not id_to_port:
            print("⚠️ 未检测到任何舵机，使用配置文件中的端口")
            return servo_config, False
        
        # 3. 更新配置中的端口
        updated = False
        for bus_name, bus_config in servo_config.items():
            if not isinstance(bus_config, dict):
                continue
            
            # 找出该总线中所有 ID 所在的端口
            ports_in_bus = set()
            for part_name, part_config in bus_config.items():
                if part_name == 'port' or not isinstance(part_config, dict):
                    continue
                for joint_name, joint_info in part_config.items():
                    if isinstance(joint_info, dict) and 'id' in joint_info:
                        sid = joint_info['id']
                        if sid in id_to_port:
                            ports_in_bus.add(id_to_port[sid])
            
            # 如果该总线的所有 ID 都在同一个端口，更新配置
            if len(ports_in_bus) == 1:
                detected_port = ports_in_bus.pop()
                old_port = bus_config.get('port')
                if old_port != detected_port:
                    bus_config['port'] = detected_port
                    print(f"🔄 {bus_name}: {old_port} → {detected_port}")
                    updated = True
        
        if updated:
            print("✅ 端口配置已自动更新")
        
        return servo_config, updated
    
    # ==================== 2. 舵机信息管理 ====================
    
    def register_servo(self, port: str, servo_id: int, brand: str, joint_name: str = ""):
        """
        注册舵机（建立映射关系）
        
        Args:
            port: 串口号
            servo_id: 舵机ID
            brand: 品牌
            joint_name: 关节名称（可选）
        """
        key = (port, servo_id)
        
        servo_info = ServoInfo(
            port=port,
            servo_id=servo_id,
            brand=brand,
            joint_name=joint_name
        )
        
        self.servo_registry[key] = servo_info
        
        if joint_name:
            self.joint_map[joint_name] = key
        
        print(f"✅ 注册舵机: {brand} @ {port} ID={servo_id}, 关节={joint_name}")
    
    def get_servo_info(self, port: str, servo_id: int) -> Optional[ServoInfo]:
        """获取舵机信息"""
        key = (port, servo_id)
        return self.servo_registry.get(key)
    
    def list_registered_servos(self) -> List[ServoInfo]:
        """列出所有已注册的舵机"""
        return list(self.servo_registry.values())
    
    # ==================== 3. ID 管理 ====================
    
    def change_servo_id(self, port: str, old_id: int, new_id: int) -> bool:
        """
        修改舵机ID（自动路由到对应品牌驱动）
        
        Args:
            port: 串口号
            old_id: 当前ID
            new_id: 新ID
            
        Returns:
            bool: 是否成功
        """
        print(f"🔄 修改舵机ID: {port} ID {old_id} → {new_id}")
        
        # ✅ 根据端口判断品牌
        brand = 'robstride' if 'can' in port.lower() else 'feetech'
        
        # 获取驱动
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        
        # ✅ 调用驱动的 set_id 方法
        success = False
        try:
            if hasattr(driver, 'set_id'):
                import time
                start_time = time.time()
                result = driver.set_id(old_id, new_id)
                elapsed = time.time() - start_time
                print(f"⏱️  ID 修改耗时: {elapsed:.2f}秒")
                
                if result is not None:
                    success = True
                    print(f"✅ ID修改成功")
                else:
                    print(f"❌ ID修改失败")
        except Exception as e:
            print(f"❌ 修改ID失败: {e}")
            return False
        
        if success:
            # ✅ 清除驱动缓存，下次使用时会重新连接
            if port in self.drivers:
                try:
                    self.drivers[port].disconnect()
                except:
                    pass
                del self.drivers[port]
            
            # 更新注册表
            old_key = (port, old_id)
            new_key = (port, new_id)
            
            if old_key in self.servo_registry:
                info = self.servo_registry.pop(old_key)
                info.servo_id = new_id
                self.servo_registry[new_key] = info
            
            print(f"✅ ID 修改成功")
        else:
            print(f"❌ ID 修改失败")
        
        return success
    
    # ==================== 4. 位置控制 ====================
    
    def set_servo_angle(self, port: str, servo_id: int, angle_deg: float, time_ms: int = 500) -> bool:
        """单个舵机角度控制（自动识别品牌）"""
        brand = self._get_servo_brand(port, servo_id) or 'feetech'
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        
        try:
            # ✅ 统一调用驱动的标准接口，品牌特定逻辑由驱动自己处理
            if hasattr(driver, 'set_angle'):
                return driver.set_angle(servo_id, angle_deg, time_ms)
            elif hasattr(driver, 'move_to_angle'):
                return driver.move_to_angle(servo_id, angle_deg, time_ms)
            return False
        except Exception:
            return False
    
    def set_servos_angles(self, port: str, targets: Dict[int, float], time_ms: int = 500) -> bool:
        """多个舵机角度控制"""
        success_count = sum(1 for servo_id, angle in targets.items() 
                          if self.set_servo_angle(port, servo_id, angle, time_ms))
        return success_count > 0
    
    def set_joint_angle(self, joint_name: str, angle_deg: float, time_ms: int = 500) -> bool:
        """按关节名称控制"""
        if joint_name not in self.joint_map:
            return False
        port, servo_id = self.joint_map[joint_name]
        return self.set_servo_angle(port, servo_id, angle_deg, time_ms)
    
    # ==================== 5. 速度控制 ====================
    
    def set_velocity_mode(self, port: str, servo_id: int) -> bool:
        """切换到速度模式（轮式模式，连续旋转）"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        if not driver or not hasattr(driver, 'set_velocity_mode'):
            return False
        return driver.set_velocity_mode(servo_id)
    
    def set_position_mode(self, port: str, servo_id: int) -> bool:
        """切换到位置模式（默认）"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        if not driver or not hasattr(driver, 'set_position_mode'):
            return False
        return driver.set_position_mode(servo_id)
    
    def set_servo_speed(self, port: str, servo_id: int, speed: int) -> bool:
        """设置速度（连续旋转）"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        return driver.set_speed(servo_id, speed)
    
    def stop_servo(self, port: str, servo_id: int) -> bool:
        """停止舵机"""
        return self.set_servo_speed(port, servo_id, 0)
    
    # ==================== 6. 批量同步控制 ====================
    
    def sync_write_positions(self, port: str, targets: Dict[int, float], time_ms: int = 500) -> bool:
        """同步位置控制（所有舵机同时开始运动）"""
        first_id = list(targets.keys())[0]
        brand = self._get_servo_brand(port, first_id)
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        if hasattr(driver, 'sync_write_positions'):
            return driver.sync_write_positions(targets, time_ms)
        return self.set_servos_angles(port, targets, time_ms)
    
    def sync_write_speeds(self, port: str, targets: Dict[int, int]) -> bool:
        """同步速度控制"""
        first_id = list(targets.keys())[0]
        brand = self._get_servo_brand(port, first_id)
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        if hasattr(driver, 'sync_write_speeds'):
            return driver.sync_write_speeds(targets)
        success_count = sum(1 for servo_id, speed in targets.items() 
                          if self.set_servo_speed(port, servo_id, speed))
        return success_count > 0
    
    # ==================== 7. 状态读取 ====================
    
    def get_servo_info(self, servo_id: int, port: str = '/dev/ttyACM0') -> Optional[Dict]:
        """获取舵机详细信息（统一接口，用于 API 返回）"""
        brand = self._get_servo_brand(port, servo_id) or 'feetech'
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return None
        
        try:
            # ✅ 统一调用驱动的标准接口，品牌特定逻辑由驱动自己处理
            if hasattr(driver, 'get_status'):
                return driver.get_status(servo_id)
            return None
        except Exception:
            return None
    
    # ==================== 8. 扭矩控制 ====================
    
    def set_torque(self, port: str, servo_id: int, enable: bool) -> bool:
        """启用/禁用扭矩"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            return False
        
        return driver.set_torque(servo_id, enable)
    
    def disable_all_torques(self, port: str) -> bool:
        """全部禁用（紧急停止）"""
        for (p, sid), info in self.servo_registry.items():
            if p == port:
                self.set_torque(port, sid, False)
        return True
    
    # ==================== 9. 辅助方法 ====================
    
    def _get_or_create_driver(self, port: str, brand: str):
        """获取或创建驱动实例（带自动重连）"""
        if port not in self.drivers:
            try:
                if brand.lower() == 'feetech':
                    from drivers.feetech.st3215_driver import ST3215Driver
                    driver = ST3215Driver(port=port, baudrate=1000000, servo_config=self._servo_config_cache)
                    if driver.connect():
                        self.drivers[port] = driver
                        return driver
                elif brand.lower() == 'robstride':
                    from drivers.robStride import RobStrideOfficialDriver
                    driver = RobStrideOfficialDriver(can_interface='can0')
                    # ✅ 不预先注册电机，让扫描时动态添加
                    if driver.connect():
                        self.drivers[port] = driver
                        return driver
            except Exception:
                pass
            return None
        
        driver = self.drivers.get(port)
        if driver and not driver.is_connected:
            if driver.connect():
                return driver
            return None
        return driver
    
    def _get_servo_brand(self, port: str, servo_id: int) -> Optional[str]:
        """获取舵机品牌"""
        key = (port, servo_id)
        info = self.servo_registry.get(key)
        
        if info:
            return info.brand
        
        # ✅ 根据端口类型自动判断品牌
        if 'can' in port.lower():
            return 'robstride'
        
        # 尝试检测
        return self._detect_brand(port, servo_id)
    
    def _detect_brand(self, port: str, servo_id: int) -> Optional[str]:
        """检测舵机品牌"""
        for brand in ['feetech']:
            try:
                driver = self._get_or_create_driver(port, brand)
                if driver and driver.ping(servo_id):
                    return brand
            except:
                continue
        
        return None
    
    def _get_servo_model(self, driver, servo_id: int) -> str:
        """获取舵机型号"""
        if hasattr(driver, 'get_model'):
            return driver.get_model(servo_id)
        return "Unknown"
    
    def cleanup(self):
        """清理资源"""
        for driver in self.drivers.values():
            try:
                driver.disconnect()
            except:
                pass
        self.drivers.clear()
    
# 全局实例
motor_controller = MotorController()
