"""
Motor命令路由器 - 轻量级路由层
"""

import logging
from typing import Dict, Any

from router.servo_handler import ServoHandler
from router.motor_control_handler import MotorControlHandler
from router.hardware_handler import HardwareHandler

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
        
        # 初始化处理器
        motor_controller = self._get_motor_controller()
        self.servo_handler = ServoHandler()
        self.motor_control_handler = MotorControlHandler(motor_controller)
        self.hardware_handler = HardwareHandler(motor_controller)
    
    def _get_motor_controller(self):
        """动态获取 motor_controller"""
        if self.control_loop and hasattr(self.control_loop, 'motor_controller'):
            return self.control_loop.motor_controller
        return None
    
    def route(self, command: Dict[str, Any]) -> bool:
        """路由单个API命令到对应的处理器"""
        action = command.get('action', '')
        
        if not action:
            logger.warning("⚠️ 命令缺少action字段")
            return False
        
        # 路由表: action → 处理方法
        route_map = {
            # === 机械臂控制 ===
            'control_motor': lambda: self.motor_control_handler.control_motor(
                command.get('arm'), command.get('motor'), command.get('angle')
            ),
            'calibrate_motor': lambda: self.motor_control_handler.calibrate_motor(
                command.get('arm'), command.get('motor'), command.get('target_zero', 0.0)
            ),
            
            # === 传感器读取 ===
            'read_sensor': lambda: self.motor_control_handler.read_sensor(
                command.get('arm'), command.get('motor')
            ),
            
            # === 电机硬件控制 ===
            'set_motor_id': lambda: self.hardware_handler.set_motor_id(command),
            'edit_motor_id': lambda: self.hardware_handler.set_motor_id(command),
            'scan_servos': lambda: self.hardware_handler.scan_servos(command, MotorRouter._ws_client),
            'list_ports': lambda: self.hardware_handler.list_ports(MotorRouter._ws_client),
            
            # === 底盘和升降轴 ===
            'control_chassis': lambda: self.motor_control_handler.control_chassis(
                command.get('wheel'), command.get('speed')
            ),
            'control_lift': lambda: self.motor_control_handler.control_lift(
                command.get('speed')
            ),
            
            # === 舵机控制 ===
            'set_servo_angle': lambda: self.servo_handler.set_angle(
                self._get_motor_controller(),
                command.get('servo_id'),
                command.get('angle'),
                command.get('port', '/dev/ttyACM0')
            ),
            'reset_servo': lambda: self.servo_handler.reset(
                command.get('servo_id'),
                command.get('port', '/dev/ttyACM0')
            ),
            'set_servo_id': lambda: self.servo_handler.set_id(
                command.get('old_id'),
                command.get('new_id'),
                command.get('port', '/dev/ttyACM0')
            ),
            'set_position_mode': lambda: self.servo_handler.set_position_mode(
                command.get('servo_id'),
                command.get('port', '/dev/ttyACM0')
            ),
            'set_speed_mode': lambda: self.servo_handler.set_speed_mode(
                command.get('servo_id'),
                command.get('port', '/dev/ttyACM0')
            ),
            'set_servo_speed': lambda: self.servo_handler.set_speed(
                command.get('servo_id'),
                command.get('speed'),
                command.get('port', '/dev/ttyACM0')
            ),
            'get_servo_info': lambda: self._handle_get_servo_info(command),
        }
        
        handler = route_map.get(action)
        if handler:
            return handler()
        else:
            logger.warning(f"⚠️ 未知的命令: {action}")
            return False
    
    def _handle_get_servo_info(self, command: Dict[str, Any]) -> bool:
        """处理获取舵机信息命令"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        info_data = self.servo_handler.get_info(servo_id, port)
        
        if info_data:
            # 通过 WebSocket 返回结果
            import asyncio
            asyncio.create_task(self._send_servo_info(info_data))
            return True
        else:
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
                logger.info(f"✅ 舵机信息已发送到 Server")
            else:
                logger.warning("⚠️ WebSocket client 未初始化")
        except Exception as e:
            logger.error(f"❌ 发送舵机信息失败: {e}")
