"""
Motor命令路由器
处理从Server接收的WebSocket Motor控制命令
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class MotorRouter:
    """API命令路由器 - 根据action字段路由到对应的处理方法"""
    
    # 类级别的全局 ws_client 引用
    _ws_client = None
    
    @classmethod
    def set_ws_client(cls, ws_client):
        """设置全局 WebSocket 客户端引用"""
        cls._ws_client = ws_client
    
    def __init__(self, control_loop=None):
        """
        初始化Motor命令路由器
        
        Args:
            control_loop: ControlLoop 实例（从中动态获取 motor_controller）
        """
        self.control_loop = control_loop
    
    @property
    def motor_controller(self):
        """动态获取 motor_controller"""
        if self.control_loop and hasattr(self.control_loop, 'motor_controller'):
            return self.control_loop.motor_controller
        return None
    
    def route(self, command: Dict[str, Any]) -> bool:
        """
        路由单个API命令到对应的处理器
        
        Args:
            command: 命令字典,包含action和其他参数
            
        Returns:
            bool: 是否成功处理
        """
        action = command.get('action', '')
        
        if not action:
            print("⚠️ 命令缺少action字段")
            return False
        
        # 路由表: action → 处理方法
        route_map = {
            # === 机械臂控制 ===
            'control_motor': self._route_control_motor,
            'calibrate_motor': self._route_calibrate_motor,
            
            # === 传感器读取 ===
            'read_sensor': self._route_read_sensor,
            
            # === 电机硬件控制 ===
            'set_motor_id': self._route_set_motor_id,
            'edit_motor_id': self._route_set_motor_id,  # 别名，兼容前端调用
            'scan_servos': self._route_scan_servos,  # 扫描在线舵机
            'list_ports': self._route_list_ports,  # 获取串口列表
            'set_operation_mode': self._route_set_mode,
            'set_velocity': self._route_set_velocity,
            'set_torque': self._route_set_torque,
            
            # === 底盘和升降轴 ===
            'control_chassis': self._route_control_chassis,
            'control_lift': self._route_control_lift,
            
            # === 校准管理 ===
            'save_calibration': self._route_save_calibration,
            'load_calibration': self._route_load_calibration,
            
            # === 配置查询 ===
            'get_servo_ids': self._route_get_servo_ids,
            
            # === 舵机控制 ===
            'set_servo_angle': self._route_set_servo_angle,
            'reset_servo': self._route_reset_servo,
            'set_servo_id': self._route_set_servo_id,
            'set_position_mode': self._route_set_position_mode,
            'set_speed_mode': self._route_set_speed_mode,
            'set_servo_speed': self._route_set_servo_speed,
            'get_servo_info': self._route_get_servo_info,
        }
        
        handler = route_map.get(action)
        if handler:
            return handler(command)
        else:
            print(f"⚠️ 未知的命令: {action}")
            return False
    
    def _route_control_motor(self, command: Dict[str, Any]) -> bool:
        """路由电机角度控制命令"""
        arm = command.get('arm')
        motor_name = command.get('motor')
        angle = command.get('angle')
        
        if not all([arm, motor_name, angle is not None]):
            print("❌ control_motor 命令缺少必要参数")
            return False
        
        print(f"🦾 控制电机: {arm}臂, {motor_name}, 角度={angle}°")
        
        if not self.motor_controller:
            print("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.control_motor(arm, motor_name, float(angle))
        if not success:
            print(f"❌ 控制电机{motor_name}失败")
        
        return success
    
    def _route_calibrate_motor(self, command: Dict[str, Any]) -> bool:
        """路由电机校准命令"""
        arm = command.get('arm')
        motor_name = command.get('motor')
        target_zero = command.get('target_zero', 0.0)
        
        if not all([arm, motor_name]):
            print("❌ calibrate_motor 命令缺少必要参数")
            return False
        
        print(f"🎯 校准电机: {arm}臂, {motor_name}, 目标零点={target_zero}°")
        
        if not self.motor_controller:
            print("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.calibrate_motor(arm, motor_name, float(target_zero))
        if not success:
            print(f"❌ 校准电机{motor_name}失败")
        
        return success
    
    def _route_control_chassis(self, command: Dict[str, Any]) -> bool:
        """路由底盘控制命令"""
        wheel = command.get('wheel')
        speed = command.get('speed')
        
        if not all([wheel, speed is not None]):
            print("❌ control_chassis 命令缺少必要参数")
            return False
        
        print(f"🚗 控制底盘: {wheel}轮, 速度={speed}%")
        
        if not self.motor_controller:
            print("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.control_chassis(wheel, float(speed))
        if not success:
            print(f"❌ 控制{wheel}轮失败")
        
        return success
    
    def _route_control_lift(self, command: Dict[str, Any]) -> bool:
        """路由升降轴控制命令"""
        speed = command.get('speed')
        
        if speed is None:
            print("❌ control_lift 命令缺少speed参数")
            return False
        
        print(f"⬆️ 控制升降轴: 速度={speed}%")
        
        if not self.motor_controller:
            print("⚠️ 电机控制器未初始化")
            return False
        
        success = self.motor_controller.control_lift(float(speed))
        if not success:
            print(f"❌ 控制升降轴失败")
        
        return success
    
    def _route_read_sensor(self, command: Dict[str, Any]) -> bool:
        """路由传感器读取命令"""
        arm = command.get('arm')
        motor_name = command.get('motor')
        
        if not all([arm, motor_name]):
            print("❌ read_sensor 命令缺少必要参数")
            return False
        
        print(f"📖 读取传感器: {arm}臂, {motor_name}")
        
        if not self.motor_controller:
            print("⚠️ 电机控制器未初始化")
            return False
        
        sensor_data = self.motor_controller.read_sensor_data(arm, motor_name)
        if sensor_data:
            print(f"✅ 传感器数据: {sensor_data}")
            # TODO: 将数据返回给前端
            return True
        else:
            print(f"❌ 读取{motor_name}传感器失败")
            return False
    
    def _route_set_motor_id(self, command: Dict[str, Any]) -> bool:
        """路由设置电机ID命令（纯硬件操作）
        
        支持两种调用方式：
        1. 直接指定串口和舵机类型（推荐）：port, servo_type, old_id, new_id
        2. 通过机械臂信息推断：arm, motor, current_id, new_id
        """
        # 方式1：直接指定串口和舵机类型
        port = command.get('port')
        servo_type = command.get('servo_type')
        old_id = command.get('old_id') or command.get('current_id')
        new_id = command.get('new_id')
        baudrate = command.get('baudrate', 115200)
        
        # 方式2：通过机械臂信息推断（从配置中获取）
        if not port or not servo_type:
            arm = command.get('arm')
            motor_name = command.get('motor')
            
            if arm and motor_name:
                print(f"🔧 通过机械臂信息推断配置: {arm}臂 {motor_name}")
                
                # 从 telegrip 配置中获取串口和舵机类型
                try:
                    from telegrip.config import config, get_config_data
                    config_data = get_config_data()
                    
                    if arm == 'left':
                        port = config.follower_ports.get('left', '/dev/ttyACM0')
                        servo_type = config_data.get('robot', {}).get('left_arm', {}).get('servo_type', 'st3215')
                        baudrate = config_data.get('robot', {}).get('left_arm', {}).get('baudrate', 1000000)
                    elif arm == 'right':
                        port = config.follower_ports.get('right', '/dev/ttyACM1')
                        servo_type = config_data.get('robot', {}).get('right_arm', {}).get('servo_type', 'st3215')
                        baudrate = config_data.get('robot', {}).get('right_arm', {}).get('baudrate', 1000000)
                    else:
                        print(f"❌ 无效的机械臂: {arm}")
                        return False
                    
                    print(f"✅ 配置推断成功: port={port}, servo_type={servo_type}, baudrate={baudrate}")
                    
                except Exception as e:
                    print(f"❌ 配置推断失败: {e}")
                    import traceback
                    print(traceback.format_exc())
                    return False
            else:
                print("❌ set_motor_id 命令缺少必要参数")
                print(f"   方式1需要: port, servo_type, old_id/current_id, new_id")
                print(f"   方式2需要: arm, motor, current_id, new_id")
                print(f"   收到: {command}")
                return False
        
        if not all([port, servo_type, old_id is not None, new_id is not None]):
            print("❌ set_motor_id 命令缺少必要参数")
            print(f"   需要: port, servo_type, old_id/current_id, new_id")
            print(f"   收到: {command}")
            return False
        
        print(f"🔧 设置电机ID: {port} ({servo_type}) ID {old_id} → {new_id}")
        
        # 调用 motor_controller 的设置ID方法（纯硬件操作）
        if hasattr(self.motor_controller, 'set_motor_id'):
            success = self.motor_controller.set_motor_id(
                port=port,
                servo_type=servo_type,
                old_id=old_id,
                new_id=new_id,
                baudrate=baudrate
            )
            if success:
                print(f"✅ 电机ID设置成功: {port} {old_id} → {new_id}")
            else:
                print(f"❌ 电机ID设置失败: {port} {old_id} → {new_id}")
            return success
        else:
            print("❌ motor_controller 没有 set_motor_id 方法")
            return False
    
    def _route_set_mode(self, command: Dict[str, Any]) -> bool:
        """路由设置电机模式命令"""
        motor_id = command.get('motor_id')
        mode = command.get('mode')  # 'position' | 'velocity' | 'torque'
        
        if not all([motor_id is not None, mode]):
            print("❌ set_operation_mode 命令缺少必要参数")
            return False
        
        valid_modes = ['position', 'velocity', 'torque']
        if mode not in valid_modes:
            print(f"❌ 无效的模式: {mode}, 可选: {valid_modes}")
            return False
        
        print(f"🔧 设置电机{motor_id} 模式: {mode}")
        
        # TODO: 调用底层驱动设置模式
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_operation_mode(motor_id, mode)
        
        print("⚠️ 设置电机模式功能待实现")
        return False
    
    def _route_set_velocity(self, command: Dict[str, Any]) -> bool:
        """路由设置电机转速命令"""
        motor_id = command.get('motor_id')
        velocity = command.get('velocity')  # rpm
        
        if not all([motor_id is not None, velocity is not None]):
            print("❌ set_velocity 命令缺少必要参数")
            return False
        
        print(f"🔄 设置电机{motor_id} 转速: {velocity} rpm")
        
        # TODO: 调用底层驱动设置转速
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_velocity(motor_id, velocity)
        
        print("⚠️ 设置电机转速功能待实现")
        return False
    
    def _route_set_torque(self, command: Dict[str, Any]) -> bool:
        """路由设置电机力矩命令"""
        motor_id = command.get('motor_id')
        torque = command.get('torque')  # 0-100%
        
        if not all([motor_id is not None, torque is not None]):
            print("❌ set_torque 命令缺少必要参数")
            return False
        
        print(f"⚡ 设置电机{motor_id} 力矩: {torque}%")
        
        # TODO: 调用底层驱动设置力矩
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_torque(motor_id, torque)
        
        print("⚠️ 设置电机力矩功能待实现")
        return False
    
    def _route_scan_servos(self, command: Dict[str, Any]) -> bool:
        """路由扫描舵机命令"""
        port = command.get('port', '/dev/ttyACM0')
        servo_type = command.get('servo_type', 'st3215')
        start_id = command.get('start_id', 1)
        end_id = command.get('end_id', 20)
        baudrate = command.get('baudrate', 1000000)
        
        print(f"🔍 扫描舵机: {port} ({servo_type}) ID范围 {start_id}-{end_id}")
        
        # 调用 controller 的扫描方法
        if hasattr(self.motor_controller, 'scan_servos'):
            found_servos = self.motor_controller.scan_servos(
                port=port,
                servo_type=servo_type,
                start_id=start_id,
                end_id=end_id,
                baudrate=baudrate
            )
            
            print(f"✅ 扫描完成，找到 {len(found_servos)} 个舵机")
            print(f"📋 舵机列表: {found_servos}")
            
            # 通过 WebSocket 返回结果（异步）
            import asyncio
            print(f"📤 准备发送扫描结果到 Server...")
            print(f"🔌 ws_client 状态: {MotorRouter._ws_client is not None}")
            if MotorRouter._ws_client:
                print(f"🔌 transport 状态: {hasattr(MotorRouter._ws_client, 'transport')}")
            asyncio.create_task(self._send_scan_result(found_servos))
            return True
        else:
            print("❌ motor_controller 没有 scan_servos 方法")
            return False
    
    async def _send_scan_result(self, servos: list):
        """发送扫描结果到 Server"""
        try:
            import json
            result_message = {
                'type': 'scan_servos_response',
                'result': servos
            }
            print(f"📤 准备发送扫描结果: {len(servos)} 个舵机")
            print(f"🔌 ws_client: {MotorRouter._ws_client}")
            
            # 通过全局 ws_client 发送到 server
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                print(f"📡 发送消息: {result_message}")
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 扫描结果已发送到 Server")
            else:
                print(f"⚠️ WebSocket client 未初始化，结果无法返回")
                print(f"   _ws_client: {MotorRouter._ws_client}")
                if MotorRouter._ws_client:
                    print(f"   属性列表: {dir(MotorRouter._ws_client)}")
        except Exception as e:
            print(f"❌ 发送扫描结果失败: {e}")
            import traceback
            traceback.print_exc()
    
    def _route_list_ports(self, command: Dict[str, Any]) -> bool:
        """路由获取串口列表命令"""
        print("🔍 获取串口列表")
        
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            # 过滤出真实的 USB 串口设备
            port_list = [
                port.device for port in ports 
                if 'USB' in port.device or 'ACM' in port.device or 'ttyUSB' in port.device or 'ttyACM' in port.device
            ]
            
            print(f"✅ 发现 {len(port_list)} 个串口: {port_list}")
            
            # 通过 WebSocket 返回结果（异步）
            import asyncio
            asyncio.create_task(self._send_ports_result(port_list))
            return True
        except Exception as e:
            print(f"❌ 获取串口列表失败: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    async def _send_ports_result(self, ports: list):
        """发送串口列表结果到 Server"""
        try:
            result_message = {
                'type': 'list_ports_response',
                'ports': ports
            }
            print(f"📤 准备发送串口列表: {len(ports)} 个端口")
            
            # 通过全局 ws_client 发送到 server
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 串口列表已发送到 Server")
            else:
                print("⚠️ WebSocket client 未初始化，结果无法返回")
        except Exception as e:
            print(f"❌ 发送串口列表失败: {e}")
    
    def _route_save_calibration(self, command: Dict[str, Any]) -> bool:
        """路由保存校准配置命令"""
        filepath = command.get('filepath', 'calibration.json')
        
        print(f"💾 保存校准配置: {filepath}")
        
        # TODO: 实现校准配置保存功能
        print("⚠️ 校准配置保存功能待实现")
        return False
    
    def _route_load_calibration(self, command: Dict[str, Any]) -> bool:
        """路由加载校准配置命令"""
        filepath = command.get('filepath', 'calibration.json')
        
        print(f"📂 加载校准配置: {filepath}")
        
        # TODO: 实现校准配置加载功能
        print("⚠️ 校准配置加载功能待实现")
        return False
    
    def _route_get_servo_ids(self, command: Dict[str, Any]) -> bool:
        """路由获取舵机 ID 配置命令"""
        print("🔧 收到获取舵机 ID 配置命令")
        
        # 从 robot_interface 获取配置
        if self.control_loop and hasattr(self.control_loop, 'robot_interface'):
            servo_config = self.control_loop.robot_interface.servo_ids
            
            # 通过 WebSocket 返回配置（异步）
            import asyncio
            asyncio.create_task(self._send_servo_ids_result(servo_config))
            return True
        else:
            print("⚠️ Robot interface not initialized")
            return False
    
    async def _send_servo_ids_result(self, servo_config: dict):
        """发送舵机 ID 配置到 Server"""
        try:
            result_message = {
                'type': 'servo_ids_response',
                'data': servo_config
            }
            print(f"📤 准备发送舵机 ID 配置")
            
            # 通过全局 ws_client 发送到 server
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 舵机 ID 配置已发送到 Server")
            else:
                print("⚠️ WebSocket client 未初始化，结果无法返回")
        except Exception as e:
            print(f"❌ 发送舵机 ID 配置失败: {e}")
    
    def _route_set_servo_angle(self, command: Dict[str, Any]) -> bool:
        """路由设置舵机角度命令"""
        servo_id = command.get('servo_id')
        angle = command.get('angle')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"🎯 设置舵机 ID={servo_id} 角度={angle}° (端口: {port})")
        
        # 调用 motor_controller 设置舵机角度
        if hasattr(self.motor_controller, 'set_servo_angle'):
            success = self.motor_controller.set_servo_angle(
                port=port,
                servo_id=servo_id,
                angle=angle
            )
            if success:
                print(f"✅ 舵机 {servo_id} 角度设置成功")
                return True
            else:
                print(f"❌ 舵机 {servo_id} 角度设置失败")
                return False
        else:
            print("⚠️ motor_controller 没有 set_servo_angle 方法")
            return False
    
    def _route_reset_servo(self, command: Dict[str, Any]) -> bool:
        """路由重置舵机命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"🔄 重置舵机 ID={servo_id} (端口: {port})")
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            # 临时创建驱动
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            # 调用重置方法
            success = driver.reset_servo(servo_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 重置成功，请断电重启")
            else:
                print(f"❌ 舵机 {servo_id} 重置失败")
            
            return success
        except Exception as e:
            print(f"❌ 重置舵机异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def _route_set_servo_id(self, command: Dict[str, Any]) -> bool:
        """路由设置舵机ID命令"""
        old_id = command.get('old_id')
        new_id = command.get('new_id')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"🔢 修改舵机 ID: {old_id} → {new_id} (端口: {port})")
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            # 临时创建驱动
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            # 调用设置ID方法
            success = driver.set_id(old_id, new_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 ID 从 {old_id} 改为 {new_id}，请断电重启")
            else:
                print(f"❌ 舵机 ID 修改失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置舵机ID异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def _route_set_position_mode(self, command: Dict[str, Any]) -> bool:
        """路由设置位置模式命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"📍 设置舵机 ID={servo_id} 为位置模式 (端口: {port})")
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_position_mode(servo_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到位置模式")
            else:
                print(f"❌ 舵机 {servo_id} 模式切换失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置位置模式异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def _route_set_speed_mode(self, command: Dict[str, Any]) -> bool:
        """路由设置速度模式命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"⚡ 设置舵机 ID={servo_id} 为速度模式 (端口: {port})")
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_velocity_mode(servo_id)
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 已切换到速度模式")
            else:
                print(f"❌ 舵机 {servo_id} 模式切换失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置速度模式异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def _route_set_servo_speed(self, command: Dict[str, Any]) -> bool:
        """路由设置舵机速度命令"""
        servo_id = command.get('servo_id')
        speed = command.get('speed')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"🚀 设置舵机 ID={servo_id} 速度={speed} (端口: {port})")
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            success = driver.set_speed(servo_id, int(speed))
            driver.disconnect()
            
            if success:
                print(f"✅ 舵机 {servo_id} 速度设置为 {speed}")
            else:
                print(f"❌ 舵机 {servo_id} 速度设置失败")
            
            return success
        except Exception as e:
            print(f"❌ 设置舵机速度异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def _route_get_servo_info(self, command: Dict[str, Any]) -> bool:
        """路由获取舵机信息命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"📊 获取舵机 ID={servo_id} 信息 (端口: {port})")
        
        try:
            from drivers.bus_servo_driver import ServoType, create_servo_driver
            
            driver = create_servo_driver(
                servo_type=ServoType.ST3215,
                port=port,
                baudrate=1000000
            )
            
            if not driver.connect():
                print(f"❌ 驱动连接失败: {port}")
                return False
            
            # 读取舵机寄存器
            position = driver.controller.read_register(servo_id, 44, 2)  # Present Position
            voltage = driver.controller.read_register(servo_id, 58, 2)   # Present Voltage
            temperature = driver.controller.read_register(servo_id, 59, 1)  # Present Temperature
            current = driver.controller.read_register(servo_id, 60, 2)   # Present Current
            mode = driver.controller.read_register(servo_id, 33, 1)      # Work Mode
            torque = driver.controller.read_register(servo_id, 40, 1)    # Torque Enable
            
            driver.disconnect()
            
            # 转换数据
            if position is None:
                position = 0
                angle = 0.0
            else:
                angle = (position / 4095.0 * 360.0) - 180.0
            
            if voltage is None:
                voltage = 0.0
            else:
                voltage = voltage / 10.0  # 转换为伏特
            
            if temperature is None:
                temperature = 0
            
            if current is None:
                current = 0
            
            if mode is None:
                mode = 0
            
            if torque is None:
                torque = 0
            
            # 通过 WebSocket 返回结果
            info_data = {
                "type": "servo_info_response",
                "servo_id": servo_id,
                "position": position,
                "angle": round(angle, 1),
                "voltage": round(voltage, 1),
                "temperature": temperature,
                "current": current,
                "mode": mode,
                "torque_enabled": torque == 1
            }
            
            print(f"✅ 舵机 {servo_id} 信息: {info_data}")
            
            # 发送到 Server
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                import asyncio
                asyncio.create_task(
                    MotorRouter._ws_client.transport.send_raw(encode_message(info_data))
                )
            
            return True
        except Exception as e:
            print(f"❌ 获取舵机信息异常: {e}")
            import traceback
            print(traceback.format_exc())
            return False
