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
        扫描所有可用的 USB 串口（过滤掉虚拟终端等）
        
        Returns:
            List[str]: 可用串口列表 ['/dev/ttyACM0', '/dev/ttyUSB0', ...]
        """
        import serial.tools.list_ports
        
        # ✅ 使用 pyserial 的 list_ports，更准确
        ports = []
        for p in serial.tools.list_ports.comports():
            # 只保留 USB 相关的串口
            if 'USB' in p.device or 'ACM' in p.device or 'ttyUSB' in p.device:
                ports.append(p.device)
        
        print(f"🔍 发现 {len(ports)} 个 USB 串口: {ports}")
        return ports
    
    def scan_servos_on_port(self, port: str, start_id: int = 1, end_id: int = 253) -> List[ServoInfo]:
        """
        在指定串口/CAN接口扫描电机（自动识别品牌）
        
        Args:
            port: 串口号或 CAN 接口 (can0)
            start_id: 起始ID
            end_id: 结束ID
            
        Returns:
            List[ServoInfo]: 发现的舵机/电机列表
        """
        print(f"🔍 扫描 {port} (ID: {start_id}-{end_id})...")
        
        found_servos = []
        
        # ✅ 根据端口类型决定扫描哪些品牌
        if 'can' in port.lower():
            # CAN 接口：扫描 RobStride
            brands_to_try = ['robstride']
        else:
            # 串口：扫描飞特等
            brands_to_try = ['feetech']
        
        for brand in brands_to_try:
            try:
                driver = self._get_or_create_driver(port, brand)
                
                if not driver or not driver.is_connected:
                    continue
                
                # RobStride 特殊处理：遍历 ID 范围
                if brand.lower() == 'robstride':
                    # ✅ 使用 ping 快速检测，不需要使能/禁用
                    for motor_id in range(start_id, end_id + 1):
                        try:
                            if driver.ping(motor_id):
                                # ✅ Ping 成功，读取状态获取位置信息
                                state = driver.get_feedback(motor_id)
                                model = 'RS-00'  # TODO: 从配置或 UUID 识别
                                servo_info = ServoInfo(
                                    port=port,
                                    servo_id=motor_id,
                                    brand='robstride',
                                    model=model,
                                    is_online=True
                                )
                                found_servos.append(servo_info)
                                self.servo_registry[(port, motor_id)] = servo_info
                                pos_str = f", 位置={state.position:.3f} rad" if state else ""
                                print(f"  ✅ 发现 RobStride 电机 ID={motor_id}, 型号={model}{pos_str}")
                            else:
                                print(f"  ⚠️  ID={motor_id} 未响应")
                        except Exception as e:
                            # ID 不存在时会有异常，这是正常的
                            pass

                else:
                    # 飞特等其他品牌：逐个 ping
                    for servo_id in range(start_id, end_id + 1):
                        if driver.ping(servo_id):
                            # 获取型号信息
                            model = self._get_servo_model(driver, servo_id)
                            
                            servo_info = ServoInfo(
                                port=port,
                                servo_id=servo_id,
                                brand=brand,
                                model=model,
                                is_online=True
                            )
                            
                            found_servos.append(servo_info)
                            print(f"  ✅ 发现 {brand} 舵机 ID={servo_id}, 型号={model}")
                
                # 如果找到了设备，就停止尝试其他品牌
                if found_servos:
                    break
                    
            except Exception as e:
                print(f"⚠️ 尝试品牌 {brand} 时出错: {e}")
                continue
        
        print(f"✅ 扫描完成，找到 {len(found_servos)} 个设备")
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
        
        # 获取品牌信息
        servo_info = self.get_servo_info(port, old_id)
        brand = servo_info.brand if servo_info else self._detect_brand(port, old_id)
        
        if not brand:
            print(f"❌ 无法检测舵机品牌")
            return False
        
        # 获取驱动
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        
        # 调用驱动的 set_id 方法
        success = driver.set_id(old_id, new_id)
        
        if success:
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
        """
        单个舵机角度控制（自动识别品牌）
        
        Args:
            port: 串口号/CAN接口
            servo_id: 舵机/电机ID
            angle_deg: 目标角度（度）- 飞特用；RobStride 会转换为弧度
            time_ms: 到达时间（毫秒）- 飞特用；RobStride 忽略
            
        Returns:
            bool: 是否成功
        """
        # 尝试获取品牌，如果未识别则默认为 feetech
        brand = self._get_servo_brand(port, servo_id)
        if not brand:
            print(f"⚠️ 未识别舵机 {servo_id} 的品牌，默认使用 feetech")
            brand = 'feetech'
        
        # 获取驱动
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            print(f"❌ 无法创建舵机 {servo_id} 的驱动")
            return False
        
        # 根据品牌调用不同方法
        if brand.lower() == 'robstride':
            # RobStride 使用 MIT 模式控制
            try:
                # ✅ 先使能电机
                if not driver.is_motor_enabled(servo_id):
                    print(f"  🔌 使能 RobStride 电机 {servo_id}...")
                    driver.enable_motor(servo_id)
                    import time
                    time.sleep(0.1)  # 等待使能完成
                
                # ✅ 关键修复：获取当前位置，避免大幅度跳变
                current_state = None
                for retry in range(3):  # 最多重试3次
                    current_state = driver.get_feedback(servo_id)
                    if current_state:
                        break
                    import time
                    time.sleep(0.05)  # 等待50ms后重试
                
                if current_state:
                    current_pos = current_state.position
                    
                    # 角度转弧度
                    target_rad = angle_deg * 3.14159265 / 180.0
                    
                    # ✅ 将目标位置调整到当前位置附近（处理多圈问题）
                    # RobStride 位置范围是 ±12.5 rad
                    # 尝试多个等效位置，选择离当前最近且在合法范围内的
                    
                    best_position = None
                    best_distance = float('inf')
                    
                    for offset in [-2, -1, 0, 1, 2]:
                        candidate = target_rad + offset * 2 * 3.14159265
                        
                        if -12.5 <= candidate <= 12.5:
                            distance = abs(candidate - current_pos)
                            if distance < best_distance:
                                best_distance = distance
                                best_position = candidate
                    
                    if best_position is None:
                        print(f"  ⚠️ RobStride {servo_id}: 无法找到合法位置")
                        return False
                    
                    position_rad = best_position
                    
                    print(f"  📍 RobStride {servo_id}: 当前={current_pos:.3f} rad, 目标={position_rad:.3f} rad ({angle_deg}°)")
                else:
                    # ⚠️ 严重警告：无法获取状态，禁止发送指令
                    print(f"  ❌ RobStride {servo_id}: 无法获取电机状态，拒绝发送指令（防止超转）")
                    return False
                
                success = driver.send_motion_control(
                    motor_id=servo_id,
                    position=position_rad,
                    kp=150.0,  # ✅ 更高Kp，更快响应
                    kd=3.0,    # ✅ 适中Kd，平衡稳定性和速度
                    torque=0.0
                )
                if success:
                    print(f"✅ RobStride 电机 {servo_id} 位置设置为 {position_rad:.3f} rad ({angle_deg}°)")
                else:
                    print(f"❌ RobStride 电机 {servo_id} 位置设置失败")
                return success
            except Exception as e:
                print(f"❌ RobStride 电机控制异常: {e}")
                return False
        else:
            # 飞特等其他品牌使用角度控制
            try:
                success = driver.move_to_angle(servo_id, angle_deg, time_ms)
                if success:
                    print(f"✅ 舵机 {servo_id} 角度设置为 {angle_deg}°")
                else:
                    print(f"❌ 舵机 {servo_id} 角度设置失败")
                return success
            except Exception as e:
                print(f"❌ 舵机 {servo_id} 控制异常: {e}")
                return False
    
    def set_servos_angles(self, port: str, targets: Dict[int, float], time_ms: int = 500) -> bool:
        """
        多个舵机角度控制
        
        Args:
            port: 串口号
            targets: {servo_id: angle_deg}
            time_ms: 到达时间
            
        Returns:
            bool: 是否成功
        """
        print(f"📤 批量设置角度: {len(targets)} 个舵机")
        
        success_count = 0
        for servo_id, angle in targets.items():
            if self.set_servo_angle(port, servo_id, angle, time_ms):
                success_count += 1
        
        print(f"✅ 成功 {success_count}/{len(targets)} 个")
        return success_count > 0
    
    def set_joint_angle(self, joint_name: str, angle_deg: float, time_ms: int = 500) -> bool:
        """
        按关节名称控制
        
        Args:
            joint_name: 关节名称
            angle_deg: 目标角度
            time_ms: 到达时间
        """
        if joint_name not in self.joint_map:
            print(f"❌ 未知关节: {joint_name}")
            return False
        
        port, servo_id = self.joint_map[joint_name]
        return self.set_servo_angle(port, servo_id, angle_deg, time_ms)
    
    # ==================== 5. 速度控制 ====================
    
    def set_velocity_mode(self, port: str, servo_id: int) -> bool:
        """切换到速度模式（轮式模式，连续旋转）"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            print(f"❌ 无法获取舵机 {servo_id} 的驱动")
            return False
        
        if hasattr(driver, 'set_velocity_mode'):
            success = driver.set_velocity_mode(servo_id)
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到速度模式")
            else:
                print(f"❌ 舵机 {servo_id} 切换速度模式失败")
            return success
        else:
            print(f"⚠️ 驱动不支持 set_velocity_mode 方法")
            return False
    
    def set_position_mode(self, port: str, servo_id: int) -> bool:
        """切换到位置模式（默认）"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            print(f"❌ 无法获取舵机 {servo_id} 的驱动")
            return False
        
        if hasattr(driver, 'set_position_mode'):
            success = driver.set_position_mode(servo_id)
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到位置模式")
            else:
                print(f"❌ 舵机 {servo_id} 切换位置模式失败")
            return success
        else:
            print(f"⚠️ 驱动不支持 set_position_mode 方法")
            return False
    
    def set_servo_speed(self, port: str, servo_id: int, speed: int) -> bool:
        """
        设置速度（连续旋转）
        
        Args:
            speed: -1000 ~ 1000, 正=顺时针, 负=逆时针
        """
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
        """
        同步位置控制（所有舵机同时开始运动）
        
        Args:
            targets: {servo_id: angle_deg}
        """
        # print(f"⚡ 同步位置控制: {len(targets)} 个舵机")
        
        # 获取第一个舵机的品牌（假设同一端口的舵机品牌相同）
        first_id = list(targets.keys())[0]
        brand = self._get_servo_brand(port, first_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            return False
        
        # 调用驱动的同步写入方法
        if hasattr(driver, 'sync_write_positions'):
            return driver.sync_write_positions(targets, time_ms)
        else:
            # 降级为逐个发送
            return self.set_servos_angles(port, targets, time_ms)
    
    def sync_write_speeds(self, port: str, targets: Dict[int, int]) -> bool:
        """
        同步速度控制
        
        Args:
            targets: {servo_id: speed}
        """
        first_id = list(targets.keys())[0]
        brand = self._get_servo_brand(port, first_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            return False
        
        if hasattr(driver, 'sync_write_speeds'):
            return driver.sync_write_speeds(targets)
        else:
            # 降级为逐个发送
            success_count = 0
            for servo_id, speed in targets.items():
                if self.set_servo_speed(port, servo_id, speed):
                    success_count += 1
            return success_count > 0
    
    # ==================== 7. 状态读取 ====================
    
    def get_servo_info(self, servo_id: int, port: str = '/dev/ttyACM0') -> Optional[Dict]:
        """
        获取舵机详细信息（统一接口，用于 API 返回）
        
        Args:
            servo_id: 舵机ID
            port: 串口号
            
        Returns:
            Dict: 包含位置、角度、电压、温度等信息
        """
        brand = self._get_servo_brand(port, servo_id)
        if not brand:
            print(f"⚠️ 未识别舵机 {servo_id} 的品牌，默认使用 feetech")
            brand = 'feetech'
        
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            print(f"❌ 无法获取舵机 {servo_id} 的驱动")
            return None
        
        try:
            # ✅ 根据品牌使用不同的读取方法
            if brand.lower() == 'robstride':
                # RobStride 使用 get_feedback
                state = driver.get_feedback(servo_id)
                if state:
                    # ✅ 将弧度转换为角度，并归一化到 -180° ~ 180°
                    angle_raw = state.position * 180.0 / 3.14159265
                    # 归一化：将任意角度映射到 [-180, 180)
                    angle_normalized = ((angle_raw + 180) % 360) - 180
                    
                    return {
                        'servo_id': servo_id,
                        'port': port,
                        'position': state.position,
                        'angle': round(angle_normalized, 2),  # ✅ 使用归一化后的角度
                        'voltage': 0,  # RobStride 不直接提供电压
                        'temperature': state.temperature,
                        'current': state.torque,  # 扭矩当作电流近似
                        'speed': state.velocity,
                        'load': 0,
                        'mode': 'position',
                        'torque_enabled': state.enabled
                    }
            elif hasattr(driver.controller, 'read_status'):
                # 飞特等其他品牌
                status = driver.controller.read_status(servo_id)
                if status:
                    position = status.get('position', 0)
                    angle = (position / 4095.0) * 360.0 - 180.0
                    
                    return {
                        'servo_id': servo_id,
                        'port': port,
                        'position': position,
                        'angle': round(angle, 2),
                        'voltage': status.get('voltage', 0),
                        'temperature': status.get('temperature', 0),
                        'current': status.get('present_current', 0),
                        'speed': status.get('speed', 0),
                        'load': status.get('load', 0),
                        'mode': 'position',
                        'torque_enabled': True
                    }
            
            print(f"❌ 舵机 {servo_id} 不支持 read_status 方法")
            return None
        except Exception as e:
            print(f"❌ 获取舵机信息失败: {e}")
            import traceback
            print(traceback.format_exc())
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
        print(f"🛑 紧急停止：禁用所有舵机扭矩")
        
        success_count = 0
        for (p, sid), info in self.servo_registry.items():
            if p == port:
                if self.set_torque(port, sid, False):
                    success_count += 1
        
        print(f"✅ 已禁用 {success_count} 个舵机")
        return True
    
    # ==================== 9. 辅助方法 ====================
    
    def _get_or_create_driver(self, port: str, brand: str):
        """获取或创建驱动实例（带自动重连）"""
        if port not in self.drivers:
            try:
                # 使用 feetech-servo-sdk (官方 SDK)
                if brand.lower() == 'feetech':
                    from drivers.feetech.st3215_driver import ST3215Driver
                    driver = ST3215Driver(port=port, baudrate=1000000, servo_config=self._servo_config_cache)
                    if driver.connect():
                        self.drivers[port] = driver
                        print(f"✅ 创建 ST3215 驱动: {brand} @ {port}")
                        return driver
                    else:
                        print(f"❌ ST3215 连接失败")
                        return None
                
                # RobStride 电机（使用官方 SDK）
                if brand.lower() == 'robstride':
                    try:
                        from drivers.robStride import RobStrideOfficialDriver
                        # RobStride 使用 CAN 接口
                        driver = RobStrideOfficialDriver(can_interface='can0')
                        
                        # ✅ 添加电机 ID 127 (使用 rs-02 型号)
                        driver.add_motor(motor_id=127, model='rs-02')
                        
                        if driver.connect():
                            self.drivers[port] = driver
                            print(f"✅ 创建 RobStride 驱动: {brand} @ {port}")
                            return driver
                        else:
                            print(f"❌ RobStride 连接失败")
                            return None
                    except Exception as e:
                        print(f"❌ 创建 RobStride 驱动失败: {e}")
                        return None
                
                elif brand.lower() == 'damiao':
                    print(f"⚠️ Damiao(LX16A) 驱动暂未实现")
                    return None
                else:
                    print(f"❌ 不支持的品牌: {brand}")
                    return None
                    
            except Exception as e:
                print(f"❌ 创建驱动失败: {e}")
                return None
        
        # ✅ 驱动已存在，检查连接状态
        driver = self.drivers.get(port)
        if driver and not driver.is_connected:
            print(f"⚠️ 驱动 {port} 已断开，尝试重新连接...")
            if driver.connect():
                print(f"✅ 重新连接成功: {port}")
            else:
                print(f"❌ 重新连接失败: {port}")
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
        print("🧹 资源清理完成")
    
# 全局实例
motor_controller = MotorController()
