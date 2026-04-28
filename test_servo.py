"""
总线舵机驱动测试脚本
用于测试 LX-16A 和 ST3215 舵机的连接和控制
"""

import sys
import time
import logging
from drivers.bus_servo_driver import create_servo_driver, ServoType

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_lx16a(port="/dev/ttyUSB0"):
    """测试 LX-16A 舵机"""
    logger.info("=" * 60)
    logger.info("测试 LX-16A 舵机")
    logger.info("=" * 60)
    
    driver = create_servo_driver(
        servo_type=ServoType.LX16A,
        port=port,
        baudrate=115200
    )
    
    if not driver.connect():
        logger.error("❌ 连接失败")
        return False
    
    try:
        # 测试读取位置
        logger.info("📖 读取舵机位置...")
        observation = driver.get_observation()
        logger.info(f"当前位置: {observation}")
        
        # 测试移动到指定位置
        logger.info("🔄 测试移动舵机...")
        action = {
            'shoulder_pan.pos': 45.0,
            'shoulder_lift.pos': 30.0,
        }
        driver.send_action(action, time_ms=1000)
        time.sleep(1.5)
        
        # 再次读取位置
        observation = driver.get_observation()
        logger.info(f"移动后位置: {observation}")
        
        logger.info("✅ LX-16A 测试成功")
        return True
        
    except Exception as e:
        logger.error(f"❌ 测试失败: {e}")
        return False
    finally:
        driver.disconnect()


def test_st3215(port="/dev/ttyUSB0"):
    """测试 ST3215 舵机"""
    logger.info("=" * 60)
    logger.info("测试 ST3215 舵机")
    logger.info("=" * 60)
    
    driver = create_servo_driver(
        servo_type=ServoType.ST3215,
        port=port,
        baudrate=1000000
    )
    
    if not driver.connect():
        logger.error("❌ 连接失败")
        return False
    
    try:
        # 测试读取位置
        logger.info("📖 读取舵机位置...")
        observation = driver.get_observation()
        logger.info(f"当前位置: {observation}")
        
        # 测试移动到指定位置
        logger.info("🔄 测试移动舵机...")
        action = {
            'shoulder_pan.pos': 45.0,
            'shoulder_lift.pos': 30.0,
        }
        driver.send_action(action, time_ms=1000)
        time.sleep(1.5)
        
        # 再次读取位置
        observation = driver.get_observation()
        logger.info(f"移动后位置: {observation}")
        
        logger.info("✅ ST3215 测试成功")
        return True
        
    except Exception as e:
        logger.error(f"❌ 测试失败: {e}")
        return False
    finally:
        driver.disconnect()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python test_servo.py <lx16a|st3215> [端口]")
        print("示例:")
        print("  python test_servo.py lx16a /dev/ttyUSB0")
        print("  python test_servo.py st3215 COM3")
        sys.exit(1)
    
    servo_type = sys.argv[1].lower()
    port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
    
    if servo_type == "lx16a":
        test_lx16a(port)
    elif servo_type == "st3215":
        test_st3215(port)
    else:
        print(f"❌ 不支持的舵机类型: {servo_type}")
        print("支持的类型: lx16a, st3215")
