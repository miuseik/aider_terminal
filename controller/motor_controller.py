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
        
        # 初始化底层驱动（如果传入配置，兼容旧代码）
        self.driver = None
        if config:
            self._initialize_driver(config)
        
        print("✅ 通用电机控制器初始化完成")
    
    # ==================== 1. 硬件发现与扫描 ====================
    
    def scan_available_ports(self) -> List[str]:
        """
        扫描所有可用的串口
        
        Returns:
            List[str]: 可用串口列表 ['/dev/ttyACM0', '/dev/ttyUSB0', ...]
        """
        import glob
        import sys
        
        ports = []
        
        if sys.platform.startswith('linux'):
            ports = glob.glob('/dev/tty[A-Z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        elif sys.platform.startswith('win'):
            import serial.tools.list_ports
            ports = [p.device for p in serial.tools.list_ports.comports()]
        
        print(f"🔍 发现 {len(ports)} 个串口: {ports}")
        return ports
    
    def scan_servos_on_port(self, port: str, start_id: int = 1, end_id: int = 253) -> List[ServoInfo]:
        """
        在指定串口扫描舵机（自动识别品牌）
        
        Args:
            port: 串口号
            start_id: 起始ID
            end_id: 结束ID
            
        Returns:
            List[ServoInfo]: 发现的舵机列表
        """
        print(f"🔍 扫描串口 {port} (ID: {start_id}-{end_id})...")
        
        found_servos = []
        
        # 尝试不同品牌的驱动
        brands_to_try = ['feetech', 'robstride', 'damiao']
        
        for brand in brands_to_try:
            try:
                driver = self._get_or_create_driver(port, brand)
                
                if not driver or not driver.is_connected:
                    continue
                
                # 扫描该品牌的舵机
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
                
                # 如果找到了舵机，就停止尝试其他品牌
                if found_servos:
                    break
                    
            except Exception as e:
                print(f"⚠️ 尝试品牌 {brand} 时出错: {e}")
                continue
        
        print(f"✅ 扫描完成，找到 {len(found_servos)} 个舵机")
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
        单个舵机角度控制
        
        Args:
            port: 串口号
            servo_id: 舵机ID
            angle_deg: 目标角度（度）
            time_ms: 到达时间（毫秒）
            
        Returns:
            bool: 是否成功
        """
        # 获取品牌
        brand = self._get_servo_brand(port, servo_id)
        if not brand:
            return False
        
        # 获取驱动
        driver = self._get_or_create_driver(port, brand)
        if not driver:
            return False
        
        # 调用驱动的位置控制
        return driver.set_position(servo_id, angle_deg, time_ms)
    
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
        """切换到速度模式"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            return False
        
        return driver.set_velocity_mode(servo_id)
    
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
        print(f"⚡ 同步位置控制: {len(targets)} 个舵机")
        
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
        print(f"⚡ 同步速度控制: {len(targets)} 个舵机")
        
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
    
    def read_servo_status(self, port: str, servo_id: int) -> Optional[ServoStatus]:
        """读取单个舵机状态"""
        brand = self._get_servo_brand(port, servo_id)
        driver = self._get_or_create_driver(port, brand)
        
        if not driver:
            return None
        
        # 调用驱动的状态读取
        if hasattr(driver, 'read_status'):
            status_dict = driver.read_status(servo_id)
            if status_dict:
                return ServoStatus(**status_dict)
        
        return None
    
    def read_all_servos_status(self, port: str) -> Dict[int, ServoStatus]:
        """读取所有舵机状态"""
        result = {}
        
        # 找到该端口的所有舵机
        for (p, sid), info in self.servo_registry.items():
            if p == port:
                status = self.read_servo_status(port, sid)
                if status:
                    result[sid] = status
        
        return result
    
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
        """获取或创建驱动实例"""
        if port not in self.drivers:
            try:
                # 优先使用 feetech_servo 目录下的驱动
                if brand.lower() == 'feetech':
                    # 尝试使用 rustypot (高性能)
                    try:
                        from drivers.feetech_servo import RustypotDriver
                        driver = RustypotDriver(port=port, baudrate=1000000)
                        if driver.connect():
                            self.drivers[port] = driver
                            print(f"✅ 创建 Rustypot 驱动: {brand} @ {port}")
                            return driver
                        else:
                            print(f"⚠️ Rustypot 连接失败，尝试 Pypot")
                    except ImportError as e:
                        print(f"⚠️ rustypot 未安装，尝试 Pypot: {e}")
                    
                    # 降级到 pypot
                    try:
                        from drivers.feetech_servo import PypotDriver
                        driver = PypotDriver(port=port, baudrate=1000000)
                        if driver.connect():
                            self.drivers[port] = driver
                            print(f"✅ 创建 Pypot 驱动: {brand} @ {port}")
                            return driver
                        else:
                            print(f"❌ Pypot 连接失败")
                            return None
                    except ImportError as e:
                        print(f"❌ pypot 未安装: {e}")
                        return None
                
                # 其他品牌使用原有的驱动工厂
                from drivers.bus_servo_driver import create_servo_driver, ServoType
                
                # 映射品牌到枚举
                brand_map = {
                    'robstride': ServoType.RS00,
                    'damiao': ServoType.LX16A,
                }
                
                servo_type = brand_map.get(brand.lower())
                if not servo_type:
                    print(f"❌ 不支持的品牌: {brand}")
                    return None
                
                driver = create_servo_driver(
                    servo_type=servo_type,
                    port=port,
                    baudrate=1000000
                )
                
                if driver.connect():
                    self.drivers[port] = driver
                    print(f"✅ 创建驱动: {brand} @ {port}")
                else:
                    return None
                    
            except Exception as e:
                print(f"❌ 创建驱动失败: {e}")
                return None
        
        return self.drivers.get(port)
    
    def _get_servo_brand(self, port: str, servo_id: int) -> Optional[str]:
        """获取舵机品牌"""
        key = (port, servo_id)
        info = self.servo_registry.get(key)
        
        if info:
            return info.brand
        
        # 尝试检测
        return self._detect_brand(port, servo_id)
    
    def _detect_brand(self, port: str, servo_id: int) -> Optional[str]:
        """检测舵机品牌"""
        for brand in ['feetech', 'robstride', 'damiao']:
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
    
    # ==================== 兼容旧版 API ====================
    
    def _initialize_driver(self, config: Dict):
        """根据配置初始化底层驱动（兼容旧代码）"""
        try:
            servo_type = config.get('servo_type', '').lower()
            port = config.get('port')
            baudrate = config.get('baudrate', 115200)
            
            if not port:
                print("⚠️ 配置中缺少串口号，跳过驱动初始化")
                return
            
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            # 映射舵机类型
            if servo_type == 'lx16a':
                servo_type_enum = ServoType.LX16A
                print(f"🔧 初始化 Hiwonder LX-16A 驱动")
            elif servo_type == 'st3215':
                servo_type_enum = ServoType.ST3215
                print(f"🔧 初始化 Feetech ST3215 驱动")
            elif servo_type in ['robstride', 'rs00']:
                # 灵足 Robstride 电机
                from drivers.robstride.robstride_driver import RobstrideDriver
                self.driver = RobstrideDriver(port=port, baudrate=baudrate)
                print(f"🔧 初始化 Robstride 电机驱动")
                if self.driver.connect():
                    print(f"✅ Robstride 驱动连接成功")
                else:
                    print(f"❌ Robstride 驱动连接失败")
                return
            else:
                print(f"❌ 不支持的舵机类型: {servo_type}")
                return
            
            # 创建驱动实例
            self.driver = create_servo_driver(
                servo_type=servo_type_enum,
                port=port,
                baudrate=baudrate
            )
            
            # 连接驱动
            if self.driver.connect():
                print(f"✅ 驱动初始化成功: {servo_type} @ {port}")
            else:
                print(f"❌ 驱动连接失败: {servo_type} @ {port}")
                self.driver = None
                
        except Exception as e:
            print(f"❌ 驱动初始化异常: {e}")
            import traceback
            print(traceback.format_exc())
            self.driver = None
    
    def control_motor(self, arm: str, motor_name: str, angle: float) -> bool:
        """
        控制单个电机角度（兼容旧代码）
        
        Args:
            arm: 'left' 或 'right'
            motor_name: 电机名称
            angle: 目标角度(度)
        """
        if not self.robot_interface or not self.robot_interface.is_connected or not self.robot_interface.is_engaged:
            print("⚠️ 机器人未连接或未使能")
            return False
        
        try:
            if motor_name not in self.motor_index_map:
                print(f"❌ 未知的电机名称: {motor_name}")
                return False
            
            index = self.motor_index_map[motor_name]
            
            # 更新对应机械臂的角度
            if arm == 'left' and self.robot_interface.left_robot and self.robot_interface.left_arm_connected:
                self.robot_interface.left_arm_angles[index] = angle
                print(f"✅ 左臂 {motor_name} 设置为 {angle}°")
                return True
            elif arm == 'right' and self.robot_interface.right_robot and self.robot_interface.right_arm_connected:
                self.robot_interface.right_arm_angles[index] = angle
                print(f"✅ 右臂 {motor_name} 设置为 {angle}°")
                return True
            else:
                print(f"❌ {arm} 臂未连接")
                return False
        except Exception as e:
            print(f"❌ 控制电机异常: {e}")
            return False
    
    def calibrate_motor(self, arm: str, motor_name: str, target_zero: float = 0.0) -> bool:
        """校准电机零点（兼容旧代码）"""
        if not self.robot_interface or not self.robot_interface.is_connected or not self.robot_interface.is_engaged:
            print("⚠️ 机器人未连接或未使能")
            return False
        
        try:
            current_angles = self.robot_interface.get_actual_arm_angles(arm)
            
            if motor_name not in self.motor_index_map:
                print(f"❌ 未知的电机名称: {motor_name}")
                return False
            
            index = self.motor_index_map[motor_name]
            current_position = current_angles[index]
            
            print(f"🎯 校准 {arm} 臂 {motor_name}: 当前位置={current_position}°, 目标零点={target_zero}°")
            print(f"⚠️ 注意: 需要在电机固件层面实现零点校准功能")
            
            if arm == 'left' and self.robot_interface.left_robot and self.robot_interface.left_arm_connected:
                self.robot_interface.left_arm_angles[index] = target_zero
                print(f"✅ 左臂 {motor_name} 零点已设置为 {target_zero}°")
                return True
            elif arm == 'right' and self.robot_interface.right_robot and self.robot_interface.right_arm_connected:
                self.robot_interface.right_arm_angles[index] = target_zero
                print(f"✅ 右臂 {motor_name} 零点已设置为 {target_zero}°")
                return True
            else:
                print(f"❌ {arm} 臂未连接")
                return False
        except Exception as e:
            print(f"❌ 校准电机异常: {e}")
            return False
    
    def send_arm_command(self, arm: str, angles: Dict[str, float]) -> bool:
        """发送机械臂角度指令到真机（兼容旧代码）"""
        if not self.robot_interface or not self.robot_interface.is_connected:
            print("⚠️ 机器人未连接")
            return False
        
        try:
            for joint_name, angle in angles.items():
                if joint_name in self.motor_index_map:
                    index = self.motor_index_map[joint_name]
                    if arm == 'left':
                        self.robot_interface.left_arm_angles[index] = angle
                    elif arm == 'right':
                        self.robot_interface.right_arm_angles[index] = angle
            
            print(f"📤 {arm}臂角度指令已准备: {angles}")
            return True
        except Exception as e:
            print(f"❌ 发送{arm}臂指令失败: {e}")
            return False
    
    def set_motor_id(self, port: str, servo_type: str, old_id: int, new_id: int, baudrate: int = 115200) -> bool:
        """设置电机ID（兼容旧代码）"""
        if not (1 <= old_id <= 253) or not (1 <= new_id <= 253):
            print(f"❌ ID超出范围: old_id={old_id}, new_id={new_id}")
            return False
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            if servo_type.lower() == 'lx16a':
                servo_type_enum = ServoType.LX16A
            elif servo_type.lower() == 'st3215':
                servo_type_enum = ServoType.ST3215
            else:
                print(f"❌ 不支持的舵机类型: {servo_type}")
                return False
            
            driver = create_servo_driver(
                servo_type=servo_type_enum,
                port=port,
                baudrate=baudrate
            )
            
            if not driver.connect():
                print(f"❌ 无法连接到 {port}")
                return False
            
            success = driver.set_id(old_id, new_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 电机ID设置成功: {port} ID {old_id} → {new_id}")
            else:
                print(f"❌ 电机ID设置失败: {port} ID {old_id} → {new_id}")
            
            return success
                
        except Exception as e:
            print(f"❌ 设置电机ID异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def scan_servos(self, port: str, servo_type: str, start_id: int = 1, end_id: int = 20, baudrate: int = 1000000) -> list:
        """扫描指定范围内的在线舵机（兼容旧代码）"""
        found_servos = []
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            if servo_type.lower() == 'lx16a':
                servo_type_enum = ServoType.LX16A
            elif servo_type.lower() == 'st3215':
                servo_type_enum = ServoType.ST3215
            else:
                print(f"❌ 不支持的舵机类型: {servo_type}")
                return []
            
            driver = create_servo_driver(
                servo_type=servo_type_enum,
                port=port,
                baudrate=baudrate
            )
            
            if not driver.connect():
                print(f"❌ 无法连接到 {port}")
                return []
            
            print(f"🔍 开始扫描舵机: {port} ({servo_type}) ID范围 {start_id}-{end_id}")
            
            for servo_id in range(start_id, end_id + 1):
                if driver.ping(servo_id):
                    found_servos.append({
                        'id': servo_id,
                        'online': True
                    })
                    print(f"✅ 发现舵机 ID={servo_id}")
                else:
                    print(f"⚪ ID={servo_id} 离线")
                
                import time
                time.sleep(0.02)
            
            driver.disconnect()
            
            print(f"✅ 扫描完成，找到 {len(found_servos)} 个在线舵机")
            return found_servos
                
        except Exception as e:
            print(f"❌ 扫描舵机异常: {e}")
            import traceback
            print(traceback.format_exc())
            return []
    
    def read_sensor_data(self, arm: str, motor_name: str):
        """读取电机传感器数据（兼容旧代码）"""
        try:
            # 优先从底层驱动读取
            if self.driver and hasattr(self.driver, 'get_observation'):
                observation = self.driver.get_observation()
                
                if motor_name not in self.motor_index_map:
                    print(f"❌ 未知的电机名称: {motor_name}")
                    return None
                
                index = self.motor_index_map[motor_name]
                joint_names = list(observation.keys())
                
                if index < len(joint_names):
                    joint_name = joint_names[index]
                    position = observation.get(joint_name, 0.0)
                    
                    return {
                        'position': float(position),
                        'velocity': 0.0,
                        'current': 0.0,
                        'temperature': 0.0
                    }
            
            # 降级方案: 从robot_interface读取
            if self.robot_interface:
                actual_angles = self.robot_interface.get_actual_arm_angles(arm)
                
                if motor_name not in self.motor_index_map:
                    print(f"❌ 未知的电机名称: {motor_name}")
                    return None
                
                index = self.motor_index_map[motor_name]
                
                return {
                    'position': float(actual_angles[index]),
                    'velocity': 0.0,
                    'current': 0.0,
                    'temperature': 0.0
                }
            
            print("⚠️ 无法读取传感器数据：驱动和robot_interface均未初始化")
            return None
            
        except Exception as e:
            print(f"❌ 读取传感器数据失败: {e}")
            import traceback
            print(traceback.format_exc())
            return None


# 全局实例
motor_controller = MotorController()
