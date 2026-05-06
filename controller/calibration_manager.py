"""
校准管理器 - 管理所有电机的校准配置

参考lerobot的MotorCalibration设计
"""

import json
import logging
from typing import Dict, Optional
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)


@dataclass
class MotorCalibration:
    """电机校准配置"""
    motor_id: int
    drive_mode: int           # 0=正常, 1=反向
    homing_offset: int        # 零点偏移量
    range_min: int            # 最小角度
    range_max: int            # 最大角度


class CalibrationManager:
    """校准管理器"""
    
    def __init__(self, motor_controller):
        """
        初始化校准管理器
        
        Args:
            motor_controller: MotorController实例
        """
        self.motor_controller = motor_controller
        self.calibrations: Dict[str, MotorCalibration] = {}
    
    def calibrate_motor(self, motor_name: str, motor_id: int) -> bool:
        """
        校准单个电机(手动搬动到零点,自动记录)
        
        流程:
        1. 读取当前位置作为零点
        2. 写入homing_offset到电机固件
        3. 保存校准配置
        
        Args:
            motor_name: 电机名称 (如 "shoulder_pan")
            motor_id: 电机ID
            
        Returns:
            bool: 是否成功
        """
        try:
            # 1. 读取当前位置作为零点
            current_pos = self.motor_controller.driver.read_position(motor_id)
            if current_pos is None:
                print(f"❌ 无法读取电机{motor_id} 当前位置")
                return False
            
            # 2. 创建校准配置
            calibration = MotorCalibration(
                motor_id=motor_id,
                drive_mode=0,
                homing_offset=int(current_pos),
                range_min=-180,
                range_max=180
            )
            
            # 3. 写入电机固件
            success = self.motor_controller.driver.write_calibration(
                motor_id=motor_id,
                homing_offset=calibration.homing_offset,
                drive_mode=calibration.drive_mode,
                range_min=calibration.range_min,
                range_max=calibration.range_max
            )
            
            if not success:
                print(f"❌ 写入电机{motor_id} 校准参数失败")
                return False
            
            # 4. 保存到内存
            self.calibrations[motor_name] = calibration
            
            print(f"✅ 电机{motor_name}(ID:{motor_id}) 校准完成:")
            print(f"   - 零点偏移: {calibration.homing_offset}°")
            print(f"   - 范围: [{calibration.range_min}, {calibration.range_max}]°")
            
            return True
        except Exception as e:
            print(f"❌ 校准电机{motor_name} 失败: {e}")
            return False
    
    def save_calibration(self, filepath: str = "calibration.json") -> bool:
        """
        保存校准配置到文件
        
        Args:
            filepath: 保存路径
            
        Returns:
            bool: 是否成功
        """
        try:
            # 转换为字典
            calib_dict = {
                name: asdict(calib) 
                for name, calib in self.calibrations.items()
            }
            
            # 保存到JSON文件
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(calib_dict, f, indent=2, ensure_ascii=False)
            
            print(f"💾 校准配置已保存: {filepath}")
            print(f"   - 共{len(self.calibrations)}个电机")
            
            return True
        except Exception as e:
            print(f"❌ 保存校准配置失败: {e}")
            return False
    
    def load_calibration(self, filepath: str = "calibration.json") -> bool:
        """
        从文件加载校准配置
        
        Args:
            filepath: 配置文件路径
            
        Returns:
            bool: 是否成功
        """
        try:
            # 读取JSON文件
            with open(filepath, 'r', encoding='utf-8') as f:
                calib_dict = json.load(f)
            
            # 转换为MotorCalibration对象
            self.calibrations = {
                name: MotorCalibration(**calib)
                for name, calib in calib_dict.items()
            }
            
            print(f"📂 校准配置已加载: {filepath}")
            print(f"   - 共{len(self.calibrations)}个电机")
            
            return True
        except FileNotFoundError:
            print(f"⚠️ 校准文件不存在: {filepath}")
            return False
        except Exception as e:
            print(f"❌ 加载校准配置失败: {e}")
            return False
    
    def apply_calibration(self, motor_name: str) -> bool:
        """
        应用校准配置到指定电机
        
        Args:
            motor_name: 电机名称
            
        Returns:
            bool: 是否成功
        """
        if motor_name not in self.calibrations:
            print(f"⚠️ 未找到电机{motor_name} 的校准配置")
            return False
        
        calibration = self.calibrations[motor_name]
        
        try:
            # 写入电机固件
            success = self.motor_controller.driver.write_calibration(
                motor_id=calibration.motor_id,
                homing_offset=calibration.homing_offset,
                drive_mode=calibration.drive_mode,
                range_min=calibration.range_min,
                range_max=calibration.range_max
            )
            
            if success:
                print(f"✅ 电机{motor_name} 校准配置已应用")
            
            return success
        except Exception as e:
            print(f"❌ 应用校准配置失败: {e}")
            return False
    
    def get_calibration(self, motor_name: str) -> Optional[MotorCalibration]:
        """
        获取电机校准配置
        
        Args:
            motor_name: 电机名称
            
        Returns:
            MotorCalibration: 校准配置,不存在返回None
        """
        return self.calibrations.get(motor_name)
