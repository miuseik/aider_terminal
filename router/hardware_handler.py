"""
硬件操作处理器 - 处理电机ID设置、扫描等底层操作
"""

import logging
from typing import Dict, Any, List

logger = logging.getLogger(__name__)


class HardwareHandler:
    """硬件操作处理器"""
    
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
    
    def set_motor_id(self, command: Dict[str, Any]) -> bool:
        """设置电机ID（支持两种调用方式）"""
        # 方式1：直接指定串口和舵机类型
        port = command.get('port')
        servo_type = command.get('servo_type')
        old_id = command.get('old_id') or command.get('current_id')
        new_id = command.get('new_id')
        baudrate = command.get('baudrate', 115200)
        
        # 方式2：通过机械臂信息推断
        if not port or not servo_type:
            arm = command.get('arm')
            motor_name = command.get('motor')
            
            if arm and motor_name:
                logger.info(f"🔧 通过机械臂信息推断配置: {arm}臂 {motor_name}")
                
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
                        logger.error(f"❌ 无效的机械臂: {arm}")
                        return False
                    
                    logger.info(f"✅ 配置推断成功: port={port}, servo_type={servo_type}, baudrate={baudrate}")
                    
                except Exception as e:
                    logger.error(f"❌ 配置推断失败: {e}")
                    import traceback
                    logger.error(traceback.format_exc())
                    return False
            else:
                logger.error("❌ set_motor_id 命令缺少必要参数")
                return False
        
        if not all([port, servo_type, old_id is not None, new_id is not None]):
            logger.error("❌ set_motor_id 命令缺少必要参数")
            return False
        
        logger.info(f"🔧 设置电机ID: {port} ({servo_type}) ID {old_id} → {new_id}")
        
        if hasattr(self.motor_controller, 'set_motor_id'):
            success = self.motor_controller.set_motor_id(
                port=port,
                servo_type=servo_type,
                old_id=old_id,
                new_id=new_id,
                baudrate=baudrate
            )
            if success:
                logger.info(f"✅ 电机ID设置成功: {port} {old_id} → {new_id}")
            else:
                logger.error(f"❌ 电机ID设置失败: {port} {old_id} → {new_id}")
            return success
        else:
            logger.error("❌ motor_controller 没有 set_motor_id 方法")
            return False
    
    def scan_servos(self, command: Dict[str, Any], ws_client=None) -> bool:
        """扫描舵机"""
        port = command.get('port', '/dev/ttyACM0')
        servo_type = command.get('servo_type', 'st3215')
        start_id = command.get('start_id', 1)
        end_id = command.get('end_id', 20)
        baudrate = command.get('baudrate', 1000000)
        
        print(f"🔍 扫描舵机: {port} ({servo_type}) ID范围 {start_id}-{end_id}")
        
        if not self.motor_controller:
            print("❌ motor_controller 未初始化")
            return False
        
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
            
            # 通过 WebSocket 返回结果
            import asyncio
            asyncio.create_task(self._send_scan_result(found_servos, ws_client))
            return True
        else:
            print("❌ motor_controller 没有 scan_servos 方法")
            return False
    
    async def _send_scan_result(self, servos: list, ws_client):
        """发送扫描结果"""
        try:
            result_message = {
                'type': 'scan_servos_response',
                'result': servos
            }
            
            if ws_client and hasattr(ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await ws_client.transport.send_raw(encode_message(result_message))
                print(f"✅ 扫描结果已发送到 Server")
            else:
                print(f"⚠️ WebSocket client 未初始化")
        except Exception as e:
            print(f"❌ 发送扫描结果失败: {e}")
    
    def list_ports(self, ws_client=None) -> bool:
        """获取串口列表"""
        logger.info("🔍 获取串口列表")
        
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            port_list = [
                port.device for port in ports 
                if 'USB' in port.device or 'ACM' in port.device or 'ttyUSB' in port.device or 'ttyACM' in port.device
            ]
            
            logger.info(f"✅ 发现 {len(port_list)} 个串口: {port_list}")
            
            import asyncio
            asyncio.create_task(self._send_ports_result(port_list, ws_client))
            return True
        except Exception as e:
            logger.error(f"❌ 获取串口列表失败: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    async def _send_ports_result(self, ports: list, ws_client):
        """发送串口列表"""
        try:
            result_message = {
                'type': 'list_ports_response',
                'ports': ports
            }
            
            if ws_client and hasattr(ws_client, 'transport'):
                from telegrip.inputs.socket.ws_protocol import encode_message
                await ws_client.transport.send_raw(encode_message(result_message))
                logger.info(f"✅ 串口列表已发送到 Server")
            else:
                logger.warning("⚠️ WebSocket client 未初始化")
        except Exception as e:
            logger.error(f"❌ 发送串口列表失败: {e}")
