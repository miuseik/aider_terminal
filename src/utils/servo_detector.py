"""
舵机自动探测工具
通过尝试不同协议来识别舵机品牌和型号
"""

import time
import logging
from typing import Optional, Dict, Tuple

logger = logging.getLogger(__name__)


class ServoDetector:
    """舵机探测器 - 自动识别舵机品牌和型号"""
    
    def __init__(self):
        self.detected_servos = []
    
    def detect_on_port(self, port: str, baudrates: list = None) -> Optional[Dict]:
        """
        在指定端口上探测舵机
        
        :param port: 串口号 (如 COM3, /dev/ttyUSB0)
        :param baudrates: 要尝试的波特率列表
        :return: 检测到的舵机信息字典，或None
        """
        if baudrates is None:
            baudrates = [115200, 1000000, 9600, 57600]
        
        print(f"🔍 开始在 {port} 上探测舵机...")
        
        # 尝试 LewanSoul LX-16A
        lx16a_result = self._try_lx16a(port, baudrates)
        if lx16a_result:
            return lx16a_result
        
        # 尝试 Feetech ST3215
        st3215_result = self._try_st3215(port, baudrates)
        if st3215_result:
            return st3215_result
        
        print(f"⚠️ 在 {port} 上未检测到已知型号的舵机")
        return None
    
    def _try_lx16a(self, port: str, baudrates: list) -> Optional[Dict]:
        """尝试 Hiwonder LX-16A 协议"""
        try:
            from src.drivers.Hiwonder.lx16a_driver import LX16ADriver
            
            for baudrate in baudrates:
                print(f"  尝试 LX-16A @ {baudrate}...")
                
                driver = LX16ADriver(port=port, baudrate=baudrate, timeout=0.5)
                
                if not driver.connect():
                    continue
                
                # 尝试ping常见的ID (1-10)
                for servo_id in range(1, 11):
                    if driver.ping(servo_id):
                        # 读取数据验证
                        position = driver.get_position(servo_id)
                        if position is not None:
                            print(f"✅ 检测到 Hiwonder LX-16A")
                            print(f"   端口: {port}")
                            print(f"   波特率: {baudrate}")
                            print(f"   ID: {servo_id}")
                            print(f"   当前位置: {position:.1f}°")
                            
                            result = {
                                'brand': 'Hiwonder',
                                'model': 'LX-16A',
                                'port': port,
                                'baudrate': baudrate,
                                'id': servo_id,
                                'position': position
                            }
                            
                            driver.disconnect()
                            return result
                
                driver.disconnect()
                time.sleep(0.1)
        
        except Exception as e:
            print(f"LX-16A 探测失败: {e}")
        
        return None
    
    def _try_st3215(self, port: str, baudrates: list) -> Optional[Dict]:
        """尝试 Feetech ST3215 协议"""
        try:
            from src.drivers.actuator.feetech import ST3215Driver
            
            for baudrate in baudrates:
                print(f"  尝试 ST3215 @ {baudrate}...")
                
                driver = ST3215Driver(port=port, baudrate=baudrate)
                
                if not driver.connect():
                    continue
                
                # 尝试ping常见的ID (1-10)
                for servo_id in range(1, 11):
                    if driver.ping(servo_id):
                        # 读取数据验证
                        position = driver.get_position(servo_id)
                        if position is not None:
                            print(f"✅ 检测到 Feetech ST3215")
                            print(f"   端口: {port}")
                            print(f"   波特率: {baudrate}")
                            print(f"   ID: {servo_id}")
                            print(f"   当前位置: {position:.1f}°")
                            
                            result = {
                                'brand': 'Feetech',
                                'model': 'ST3215',
                                'port': port,
                                'baudrate': baudrate,
                                'id': servo_id,
                                'position': position
                            }
                            
                            driver.disconnect()
                            return result
                
                driver.disconnect()
                time.sleep(0.1)
        
        except Exception as e:
            print(f"ST3215 探测失败: {e}")
        
        return None
    
    def scan_all_ports(self, ports: list = None) -> list:
        """
        扫描多个端口
        
        :param ports: 要扫描的端口列表
        :return: 检测到的所有舵机列表
        """
        if ports is None:
            # Windows 常见端口
            ports = [f'COM{i}' for i in range(1, 20)]
            # Linux 常见端口
            ports += [f'/dev/ttyUSB{i}' for i in range(4)]
            ports += [f'/dev/ttyACM{i}' for i in range(4)]
        
        detected = []
        
        for port in ports:
            result = self.detect_on_port(port)
            if result:
                detected.append(result)
        
        print(f"\n📊 扫描完成，共发现 {len(detected)} 个舵机")
        for servo in detected:
            print(f"  - {servo['brand']} {servo['model']} @ {servo['port']} "
                       f"(ID:{servo['id']}, {servo['baudrate']}bps)")
        
        return detected
    
    def generate_config(self, detected_servos: list) -> str:
        """
        根据检测结果生成配置文件片段
        
        :param detected_servos: 检测到的舵机列表
        :return: YAML 配置片段
        """
        if not detected_servos:
            return "# 未检测到舵机\n"
        
        config_lines = ["robot:"]
        
        for i, servo in enumerate(detected_servos):
            arm_name = "left_arm" if i == 0 else "right_arm"
            
            config_lines.append(f"  {arm_name}:")
            config_lines.append(f"    enabled: true")
            config_lines.append(f"    port: {servo['port']}")
            config_lines.append(f"    servo_type: {servo['model'].lower().replace('-', '')}")
            config_lines.append(f"    baudrate: {servo['baudrate']}")
            config_lines.append(f"    # 检测到: {servo['brand']} {servo['model']}, ID={servo['id']}")
            config_lines.append("")
        
        return "\n".join(config_lines)


def main():
    """主函数 - 命令行界面"""
    import sys
    
    print("=" * 60)
    print("舵机自动探测工具")
    print("=" * 60)
    
    detector = ServoDetector()
    
    # 如果提供了端口参数，只扫描该端口
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"\n正在扫描端口: {port}\n")
        result = detector.detect_on_port(port)
        
        if result:
            print("\n" + "=" * 60)
            print("检测结果:")
            print("=" * 60)
            print(f"品牌: {result['brand']}")
            print(f"型号: {result['model']}")
            print(f"端口: {result['port']}")
            print(f"波特率: {result['baudrate']}")
            print(f"ID: {result['id']}")
            print(f"当前位置: {result['position']:.1f}°")
            
            print("\n" + "=" * 60)
            print("建议的配置:")
            print("=" * 60)
            config = detector.generate_config([result])
            print(config)
        else:
            print(f"\n❌ 在 {port} 上未检测到舵机")
            print("\n可能的原因:")
            print("  1. 舵机未连接或供电不足")
            print("  2. 串口号不正确")
            print("  3. 波特率不匹配")
            print("  4. 舵机ID不在1-10范围内")
    else:
        # 扫描所有常见端口
        print("\n正在扫描所有常见端口...\n")
        detected = detector.scan_all_ports()
        
        if detected:
            print("\n" + "=" * 60)
            print("建议的配置文件:")
            print("=" * 60)
            config = detector.generate_config(detected)
            print(config)
        else:
            print("\n❌ 未检测到任何舵机")


if __name__ == "__main__":
    # 设置日志级别
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)s: %(message)s'
    )
    
    main()
