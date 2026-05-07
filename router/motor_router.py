"""
Motor命令路由器 - 轻量级路由层

职责:
- 接收 WebSocket/API 命令
- 根据 action 字段路由到 MotorController 的对应方法
- 处理结果返回（通过 WebSocket）
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class MotorRouter:
    """API命令路由器 - 根据action字段路由到对应的处理器"""
    
    # 类级别的全局 ws_client 引用
    _ws_client = None
    
    @classmethod
    def set_ws_client(cls, ws_client):
        """设置全局 WebSocket 客户端引用"""
        cls._ws_client = ws_client
    
    def __init__(self, control_loop=None):
        """初始化Motor命令路由器"""
        self.control_loop = control_loop
        # 不再需要单独的 Handler，直接使用 motor_controller
    
    def _get_motor_controller(self):
        """动态获取 motor_controller"""
        if self.control_loop and hasattr(self.control_loop, 'motor_controller'):
            return self.control_loop.motor_controller
        return None
    
    @staticmethod
    def _safe_call(method, *args, **kwargs):
        """安全调用 motor_controller 方法"""
        if method is None:
            print("❌ MotorController 未初始化")
            return False
        try:
            return method(*args, **kwargs)
        except Exception as e:
            print(f"❌ 调用失败: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    def route(self, command: Dict[str, Any]) -> bool:
        """路由单个API命令到对应的处理器"""
        action = command.get('action', '')
        
        if not action:
            print("⚠️ 命令缺少action字段")
            return False
        
        # 获取 motor_controller
        mc = self._get_motor_controller()
        
        # 路由表: action → 处理方法
        route_map = {
            # === 舵机控制（直接调用 MotorController）===
            'set_servo_angle': lambda: self._safe_call(
                mc.set_servo_angle,
                command.get('port', '/dev/ttyACM0'),
                command.get('servo_id'),
                command.get('angle'),
                command.get('time_ms', 500)
            ),
            'set_servo_id': lambda: self._safe_call(
                mc.change_servo_id,
                command.get('port', '/dev/ttyACM0'),
                command.get('old_id'),
                command.get('new_id')
            ),
            'set_servo_speed': lambda: self._safe_call(
                mc.set_servo_speed,
                command.get('port', '/dev/ttyACM0'),
                command.get('servo_id'),
                command.get('speed')
            ),
            'set_speed_mode': lambda: self._handle_set_speed_mode(command),
            'set_position_mode': lambda: self._handle_set_position_mode(command),
            'scan_servos': lambda: self._handle_scan_servos(command),
            'list_ports': lambda: self._handle_list_ports(),
            'get_network_info': lambda: self._handle_get_network_info(),
            'ping_servo': lambda: self._route_ping_servo(command),
            'get_servo_info': lambda: self._handle_get_servo_info(command),
        }
        
        handler = route_map.get(action)
        if handler:
            return handler()
        else:
            print(f"⚠️ 未知的命令: {action}")
            return False
    
    def _handle_get_servo_info(self, command: Dict[str, Any]) -> bool:
        """处理获取舵机信息命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        print(f"🔍 请求获取舵机信息: ID={servo_id}, Port={port}")
        
        mc = self._get_motor_controller()
        if not mc:
            print("❌ MotorController 未初始化")
            return False
        
        # ✅ 使用新的 get_servo_info 方法（基于 ST3215Driver）
        info_data = mc.get_servo_info(servo_id, port)
        
        if info_data:
            print(f"✅ 成功获取舵机信息: {info_data}")
            # 通过 WebSocket 返回结果
            import asyncio
            asyncio.create_task(self._send_servo_info(info_data))
            return True
        else:
            print(f"❌ 获取舵机信息失败: ID={servo_id}")
            return False
    
    def _handle_set_speed_mode(self, command: Dict[str, Any]) -> bool:
        """处理设置速度模式命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        if not servo_id:
            print("❌ set_speed_mode 命令缺少 servo_id 参数")
            return False
        
        print(f"🔄 设置舵机 {servo_id} 为速度模式 (Port={port})")
        
        mc = self._get_motor_controller()
        if not mc:
            print("❌ MotorController 未初始化")
            return False
        
        # 调用 motor_controller 的 set_velocity_mode 方法
        success = self._safe_call(
            mc.set_velocity_mode,
            port,
            servo_id
        )
        
        if success:
            print(f"✅ 舵机 {servo_id} 已切换到速度模式")
        else:
            print(f"❌ 舵机 {servo_id} 切换速度模式失败")
        
        return success
    
    def _handle_set_position_mode(self, command: Dict[str, Any]) -> bool:
        """处理设置位置模式命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        if not servo_id:
            print("❌ set_position_mode 命令缺少 servo_id 参数")
            return False
        
        print(f"🔄 设置舵机 {servo_id} 为位置模式 (Port={port})")
        
        mc = self._get_motor_controller()
        if not mc:
            print("❌ MotorController 未初始化")
            return False
        
        # 需要先添加 set_position_mode 方法到 motor_controller
        # 暂时直接调用驱动
        brand = 'feetech'  # 默认飞特
        driver = mc._get_or_create_driver(port, brand)
        
        if not driver:
            print(f"❌ 无法获取驱动")
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
    
    async def _send_servo_info(self, info_data: Dict[str, Any]):
        """发送舵机信息"""
        try:
            result_message = {
                'type': 'servo_info_response',
                **info_data
            }
            
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 舵机信息已发送到 Server")
            else:
                print("⚠️ WebSocket client 未初始化")
        except Exception as e:
            print(f"❌ 发送舵机信息失败: {e}")
    

    def _route_ping_servo(self, command: Dict[str, Any]) -> bool:
        """Ping 舵机（小幅度摆动 3 秒）"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        if not servo_id:
            print("❌ ping_servo 命令缺少 servo_id 参数")
            return False
        
        print(f"📡 Ping 舵机: ID={servo_id}, Port={port}")
        
        # 异步执行摆动
        import asyncio
        asyncio.create_task(self._execute_ping(servo_id, port, command))
        return True
    
    async def _execute_ping(self, servo_id: int, port: str, command: Dict[str, Any]):
        """执行舵机摆动"""
        try:
            import time
            import math
            
            mc = self._get_motor_controller()
            if not mc:
                print("❌ MotorController 未初始化")
                return
            
            # ✅ 使用新的 get_servo_info 方法获取当前位置
            info_data = mc.get_servo_info(servo_id, port)
            if not info_data:
                print(f"❌ 无法获取舵机 {servo_id} 信息")
                return
            
            current_angle = info_data['angle']
            duration = 3.0  # 3 秒
            start_time = time.time()
            
            print(f"🔄 开始 Ping 舵机 {servo_id}，持续 {duration} 秒")
            
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                progress = elapsed / duration
                
                # 正弦波摆动：4 个周期，±10 度
                offset = math.sin(progress * math.pi * 4) * 10
                target_angle = current_angle + offset
                
                # 设置角度
                mc.set_servo_angle(
                    port,
                    servo_id,
                    target_angle,
                    50  # 快速响应
                )
                
                time.sleep(0.05)  # 50ms 更新一次
            
            # 恢复到原位
            mc.set_servo_angle(
                port,
                servo_id,
                current_angle,
                500  # 平滑恢复
            )
            
            print(f"✅ Ping 完成：舵机 {servo_id}")
            
        except Exception as e:
            print(f"❌ Ping 舵机失败: {e}")
    
    async def _send_servo_ids_result(self, servo_config: dict):
        """发送舵机ID配置"""
        try:
            result_message = {
                'type': 'servo_ids_response',
                'data': servo_config
            }
            
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 舵机 ID 配置已发送到 Server")
            else:
                print("⚠️ WebSocket client 未初始化")
        except Exception as e:
            print(f"❌ 发送舵机 ID 配置失败: {e}")

    def _handle_get_network_info(self) -> bool:
        """处理获取网络信息命令"""
        try:
            from utils.network_utils import get_network_info
            info = get_network_info()
            print(f"🌐 获取网络信息: {info}")
            
            import asyncio
            asyncio.create_task(self._send_network_info(info))
            return True
        except Exception as e:
            print(f"❌ 获取网络信息失败: {e}")
            return False

    async def _send_network_info(self, info: dict):
        """发送网络信息到 Server"""
        try:
            result_message = {
                'type': 'network_info_response',
                **info
            }
            
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 网络信息已发送到 Server")
            else:
                print("⚠️ WebSocket client 未初始化")
        except Exception as e:
            print(f"❌ 发送网络信息失败: {e}")
    
    def _handle_scan_servos(self, command: Dict[str, Any]) -> bool:
        """处理扫描舵机命令"""
        port = command.get('port', '/dev/ttyACM0')
        start_id = command.get('start_id', 1)
        end_id = command.get('end_id', 253)  # 扩大默认范围
        
        print(f"🔍 扫描舵机: {port} ID范围 {start_id}-{end_id}（自动识别品牌）")
        
        mc = self._get_motor_controller()
        if not mc:
            print("❌ MotorController 未初始化")
            return False
        
        # ✅ 使用新的自动识别品牌的扫描方法
        found_servos = mc.scan_servos_on_port(port, start_id, end_id)
        
        # 转换为字典格式（保持 API 兼容性）
        result = [
            {
                'id': servo.servo_id,
                'port': servo.port,
                'brand': servo.brand,
                'model': servo.model,
                'online': servo.is_online
            }
            for servo in found_servos
        ]
        
        print(f"✅ 扫描完成，找到 {len(result)} 个舵机")
        print(f"📋 舵机列表: {result}")
        
        # 通过 WebSocket 返回结果
        import asyncio
        asyncio.create_task(self._send_scan_result(result))
        return True
    
    async def _send_scan_result(self, servos: list):
        """发送扫描结果"""
        try:
            result_message = {
                'type': 'scan_servos_response',
                'result': servos
            }
            
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 扫描结果已发送到 Server")
            else:
                print(f"⚠️ WebSocket client 未初始化")
        except Exception as e:
            print(f"❌ 发送扫描结果失败: {e}")
    
    def _handle_list_ports(self) -> bool:
        """处理获取串口列表命令"""
        print("🔍 收到 list_ports 命令")
        
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            port_list = [
                port.device for port in ports 
                if 'USB' in port.device or 'ACM' in port.device or 'ttyUSB' in port.device or 'ttyACM' in port.device
            ]
            
            print(f"✅ 发现 {len(port_list)} 个串口: {port_list}")
            
            import asyncio
            asyncio.create_task(self._send_ports_result(port_list))
            return True
        except Exception as e:
            print(f"❌ 获取串口列表失败: {e}")
            import traceback
            print(traceback.format_exc())
            return False
    
    async def _send_ports_result(self, ports: list):
        """发送串口列表"""
        try:
            result_message = {
                'type': 'list_ports_response',
                'ports': ports
            }
            
            if MotorRouter._ws_client and hasattr(MotorRouter._ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await MotorRouter._ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 串口列表已发送到 Server")
            else:
                print("⚠️ WebSocket client 未初始化")
        except Exception as e:
            print(f"❌ 发送串口列表失败: {e}")
