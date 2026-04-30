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
        
        # 初始化处理器（不传入 motor_controller，使用时动态获取）
        self.servo_handler = ServoHandler()
        self.motor_control_handler = None  # 延迟初始化
        self.hardware_handler = None  # 延迟初始化
    
    def _get_motor_controller(self):
        """动态获取 motor_controller"""
        if self.control_loop and hasattr(self.control_loop, 'motor_controller'):
            return self.control_loop.motor_controller
        return None
    
    def _get_motor_control_handler(self):
        """延迟获取 motor_control_handler（每次检查 motor_controller 是否有效）"""
        motor_controller = self._get_motor_controller()
        
        # 如果 motor_controller 为 None，返回错误
        if motor_controller is None:
            logger.error("❌ motor_controller 未初始化")
            return None
        
        # 如果 handler 不存在或 motor_controller 已变化，重新创建
        if self.motor_control_handler is None or self.motor_control_handler.motor_controller != motor_controller:
            self.motor_control_handler = MotorControlHandler(motor_controller)
        
        return self.motor_control_handler
    
    def _get_hardware_handler(self):
        """延迟获取 hardware_handler（每次检查 motor_controller 是否有效）"""
        motor_controller = self._get_motor_controller()
        
        # 如果 motor_controller 为 None，返回错误
        if motor_controller is None:
            logger.error("❌ motor_controller 未初始化")
            return None
        
        # 如果 handler 不存在或 motor_controller 已变化，重新创建
        if self.hardware_handler is None or self.hardware_handler.motor_controller != motor_controller:
            self.hardware_handler = HardwareHandler(motor_controller)
        
        return self.hardware_handler
    
    @staticmethod
    def _safe_call(func, handler):
        """安全调用 handler 方法，如果 handler 为 None 则返回 False"""
        if handler is None:
            return False
        return func(handler)
    
    def route(self, command: Dict[str, Any]) -> bool:
        """路由单个API命令到对应的处理器"""
        action = command.get('action', '')
        
        if not action:
            logger.warning("⚠️ 命令缺少action字段")
            return False
        
        # 路由表: action → 处理方法
        route_map = {
            # === 机械臂控制 ===
            'control_motor': lambda: self._safe_call(
                lambda h: h.control_motor(command.get('arm'), command.get('motor'), command.get('angle')),
                self._get_motor_control_handler()
            ),
            'calibrate_motor': lambda: self._safe_call(
                lambda h: h.calibrate_motor(command.get('arm'), command.get('motor'), command.get('target_zero', 0.0)),
                self._get_motor_control_handler()
            ),
            
            # === 传感器读取 ===
            'read_sensor': lambda: self._safe_call(
                lambda h: h.read_sensor(command.get('arm'), command.get('motor')),
                self._get_motor_control_handler()
            ),
            
            # === 电机硬件控制 ===
            'set_motor_id': lambda: self._safe_call(
                lambda h: h.set_motor_id(command),
                self._get_hardware_handler()
            ),
            'edit_motor_id': lambda: self._safe_call(
                lambda h: h.set_motor_id(command),
                self._get_hardware_handler()
            ),
            'scan_servos': lambda: self._safe_call(
                lambda h: h.scan_servos(command, MotorRouter._ws_client),
                self._get_hardware_handler()
            ),
            'list_ports': lambda: self._safe_call(
                lambda h: h.list_ports(MotorRouter._ws_client),
                self._get_hardware_handler()
            ),
            'set_operation_mode': lambda: self._route_set_mode(command),
            'set_velocity': lambda: self._route_set_velocity(command),
            'set_torque': lambda: self._route_set_torque(command),
            
            # === 底盘和升降轴 ===
            'control_chassis': lambda: self._safe_call(
                lambda h: h.control_chassis(command.get('wheel'), command.get('speed')),
                self._get_motor_control_handler()
            ),
            'control_lift': lambda: self._safe_call(
                lambda h: h.control_lift(command.get('speed')),
                self._get_motor_control_handler()
            ),
            
            # === 校准管理 ===
            'save_calibration': lambda: self._route_save_calibration(command),
            'load_calibration': lambda: self._route_load_calibration(command),
            
            # === 配置管理 ===
            'reload_servo_config': lambda: self._route_reload_servo_config(),
            
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
            'ping_servo': lambda: self._route_ping_servo(command),
            'calibrate_servo_zero': lambda: self._route_calibrate_servo_zero(command),
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
        
        logger.info(f"🔍 请求获取舵机信息: ID={servo_id}, Port={port}")
        info_data = self.servo_handler.get_info(servo_id, port)
        
        if info_data:
            logger.info(f"✅ 成功获取舵机信息: {info_data}")
            # 通过 WebSocket 返回结果
            import asyncio
            asyncio.create_task(self._send_servo_info(info_data))
            return True
        else:
            logger.error(f"❌ 获取舵机信息失败: ID={servo_id}")
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
    
    def _route_set_mode(self, command: Dict[str, Any]) -> bool:
        """路由设置电机模式命令"""
        motor_id = command.get('motor_id')
        mode = command.get('mode')  # 'position' | 'velocity' | 'torque'
        
        if not all([motor_id is not None, mode]):
            logger.error("❌ set_operation_mode 命令缺少必要参数")
            return False
        
        valid_modes = ['position', 'velocity', 'torque']
        if mode not in valid_modes:
            logger.error(f"❌ 无效的模式: {mode}, 可选: {valid_modes}")
            return False
        
        logger.info(f"🔧 设置电机{motor_id} 模式: {mode}")
        
        # TODO: 调用底层驱动设置模式
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_operation_mode(motor_id, mode)
        
        logger.warning("⚠️ 设置电机模式功能待实现")
        return False
    
    def _route_set_velocity(self, command: Dict[str, Any]) -> bool:
        """路由设置电机转速命令"""
        motor_id = command.get('motor_id')
        velocity = command.get('velocity')  # rpm
        
        if not all([motor_id is not None, velocity is not None]):
            logger.error("❌ set_velocity 命令缺少必要参数")
            return False
        
        logger.info(f"🔄 设置电机{motor_id} 转速: {velocity} rpm")
        
        # TODO: 调用底层驱动设置转速
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_velocity(motor_id, velocity)
        
        logger.warning("⚠️ 设置电机转速功能待实现")
        return False
    
    def _route_set_torque(self, command: Dict[str, Any]) -> bool:
        """路由设置电机力矩命令"""
        motor_id = command.get('motor_id')
        torque = command.get('torque')  # 0-100%
        
        if not all([motor_id is not None, torque is not None]):
            logger.error("❌ set_torque 命令缺少必要参数")
            return False
        
        logger.info(f"⚡ 设置电机{motor_id} 力矩: {torque}%")
        
        # TODO: 调用底层驱动设置力矩
        # if hasattr(self.motor_controller, 'driver'):
        #     return self.motor_controller.driver.set_torque(motor_id, torque)
        
        logger.warning("⚠️ 设置电机力矩功能待实现")
        return False
    
    def _route_save_calibration(self, command: Dict[str, Any]) -> bool:
        """路由保存校准配置命令"""
        filepath = command.get('filepath', 'calibration.json')
        
        logger.info(f"💾 保存校准配置: {filepath}")
        
        # TODO: 实现校准配置保存功能
        logger.warning("⚠️ 校准配置保存功能待实现")
        return False
    
    def _route_load_calibration(self, command: Dict[str, Any]) -> bool:
        """路由加载校准配置命令"""
        filepath = command.get('filepath', 'calibration.json')
        
        logger.info(f"📂 加载校准配置: {filepath}")
        
        # TODO: 实现校准配置加载功能
        logger.warning("⚠️ 校准配置加载功能待实现")
        return False
    
    def _route_reload_servo_config(self) -> bool:
        """重载舵机配置（从 Server 获取）"""
        logger.info("🔄 收到重载舵机配置命令")
        
        if self.control_loop and hasattr(self.control_loop, 'robot_interface'):
            from router.server_api_client import ServerAPIClient
            
            api_client = ServerAPIClient()
            config = api_client.get_servo_ids_config()
            
            if config:
                success = self.control_loop.robot_interface.set_servo_ids_config(config)
                if success:
                    logger.info("✅ 舵机配置已重载")
                    return True
                else:
                    logger.error("❌ 设置舵机配置失败")
                    return False
            else:
                logger.error("❌ 从 Server 获取配置失败")
                return False
        else:
            logger.warning("⚠️ Robot interface 未初始化")
            return False
    
    def _route_get_servo_ids(self) -> bool:
        """获取舵机ID配置"""
        logger.info("🔧 收到获取舵机 ID 配置命令")
        
        if self.control_loop and hasattr(self.control_loop, 'robot_interface'):
            servo_config = self.control_loop.robot_interface.servo_ids
            
            import asyncio
            asyncio.create_task(self._send_servo_ids_result(servo_config))
            return True
        else:
            logger.warning("⚠️ Robot interface not initialized")
            return False
    
    def _route_ping_servo(self, command: Dict[str, Any]) -> bool:
        """Ping 舵机（小幅度摆动 3 秒）"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        if not servo_id:
            logger.error("❌ ping_servo 命令缺少 servo_id 参数")
            return False
        
        logger.info(f"📡 Ping 舵机: ID={servo_id}, Port={port}")
        
        # 异步执行摆动
        import asyncio
        asyncio.create_task(self._execute_ping(servo_id, port))
        return True
    
    async def _execute_ping(self, servo_id: int, port: str):
        """执行舵机摆动"""
        try:
            import time
            import math
            
            # 获取当前位置
            info_data = self.servo_handler.get_info(servo_id, port)
            if not info_data:
                logger.error(f"❌ 无法获取舵机 {servo_id} 信息")
                return
            
            current_angle = info_data['angle']
            duration = 3.0  # 3 秒
            start_time = time.time()
            
            logger.info(f"🔄 开始 Ping 舵机 {servo_id}，持续 {duration} 秒")
            
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                progress = elapsed / duration
                
                # 正弦波摆动：4 个周期，±10 度
                offset = math.sin(progress * math.pi * 4) * 10
                target_angle = current_angle + offset
                
                # 设置角度
                self.servo_handler.set_angle(
                    self._get_motor_controller(),
                    servo_id,
                    target_angle,
                    port
                )
                
                time.sleep(0.05)  # 50ms 更新一次
            
            # 恢复到原位
            self.servo_handler.set_angle(
                self._get_motor_controller(),
                servo_id,
                current_angle,
                port
            )
            
            logger.info(f"✅ Ping 完成：舵机 {servo_id}")
            
        except Exception as e:
            logger.error(f"❌ Ping 舵机失败: {e}")
    
    def _route_calibrate_servo_zero(self, command: Dict[str, Any]) -> bool:
        """校准舵机零点（设置当前位置为 0 度）"""
        servo_id = command.get('servo_id')
        port = command.get('port', '/dev/ttyACM0')
        
        if not servo_id:
            logger.error("❌ calibrate_servo_zero 命令缺少 servo_id 参数")
            return False
        
        logger.info(f"⚙️ 校准舵机零点: ID={servo_id}, Port={port}")
        
        try:
            # 获取当前位置
            info_data = self.servo_handler.get_info(servo_id, port)
            if not info_data:
                logger.error(f"❌ 无法获取舵机 {servo_id} 信息")
                return False
            
            current_angle = info_data['angle']
            logger.info(f"📍 当前位置: {current_angle}°，设置为 0°")
            
            # TODO: 保存零点偏移到配置文件
            # 需要找到该舵机在配置中的位置，更新 zero_offset
            logger.warning("⚠️ 舵机零点校准功能待实现（需要保存零点偏移到配置文件）")
            
            return True
        except Exception as e:
            logger.error(f"❌ 校准舵机零点失败: {e}")
            return False
    
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
                logger.info(f"✅ 舵机 ID 配置已发送到 Server")
            else:
                logger.warning("⚠️ WebSocket client 未初始化")
        except Exception as e:
            logger.error(f"❌ 发送舵机 ID 配置失败: {e}")
