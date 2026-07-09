"""硬件信息采集 — 从 RobotInterface 收集并格式化前端所需的硬件状态数据."""

from typing import List


async def push_robot_hardware_info(transport, robot_interface) -> None:
    """推送机器人硬件详情给前端（数据由 reporter 模块收集）。

    不依赖 ControlLoop，作为独立函数可放在 loop 和主函数之外。
    """
    if not transport or not transport.is_connected:
        return
    ri = robot_interface
    if not ri:
        return
    try:
        from aiderminal.comm.websocket.protocol import encode_message
        msg = collect_hardware_info(ri)
        await transport.send_raw(encode_message(msg))
    except Exception as e:
        import traceback
        print(f"❌ 推送机器人硬件详情失败: {e}")
        traceback.print_exc()


def collect_hardware_info(ri) -> dict:
    """收集完整硬件信息（舵机 ID/关节名/部位/品牌/角度/在线状态等）。

    供 push_robot_hardware_info 调用，返回 robot_hardware_info 消息字典。
    如果 ri.servo_ids 为空，会尝试从 Server 拉取配置。

    Args:
        ri: RobotInterface 实例

    Returns:
        dict: robot_hardware_info 消息体
    """
    if not ri.servo_ids:
        _fetch_servo_config_from_server(ri)

    part_driver = _build_part_driver_map(ri)
    all_servos = _collect_servos(ri, part_driver)

    msg = {
        'type': 'robot_hardware_info',
        'servos': all_servos,
        'robot_connected': ri.is_connected,
        'is_engaged': ri.is_engaged,
        'left_arm_connected': ri.left_arm_connected,
        'right_arm_connected': ri.right_arm_connected,
        'base_connected': ri.base_connected,
        'lift_connected': ri.lift_connected,
        'left_arm_angles': ri.left_arm_angles.tolist() if hasattr(ri, 'left_arm_angles') and ri.left_arm_angles is not None else [],
        'right_arm_angles': ri.right_arm_angles.tolist() if hasattr(ri, 'right_arm_angles') and ri.right_arm_angles is not None else [],
        'lift_height_mm': ri.lift_height_mm,
        'lost_multiturn': getattr(ri, 'lost_multiturn_motors', []),
    }

    _log_summary(all_servos, ri)
    return msg


# ---- 内部 helpers ----

def _fetch_servo_config_from_server(ri) -> None:
    print("⚡ servo_ids 为空，尝试从 Server 拉取舵机配置...")
    try:
        from aiderminal.comm.api.client import ServerAPIClient
        api_client = ServerAPIClient()
        servo_config = api_client.get_servo_ids_config()
        if servo_config:
            ri.set_servo_ids_config(servo_config)
            print(f"✅ 从 Server 加载了舵机配置: {list(servo_config.keys())}")
    except Exception as e:
        print(f"⚠️ 从 Server 拉取舵机配置失败: {e}")


def _build_part_driver_map(ri) -> dict:
    """构建 部位名 → 驱动器 的映射."""
    part_driver = {}
    if ri.left_robot and ri.left_arm_connected:
        part_driver['left_arm'] = ri.left_robot
    if ri.right_robot and ri.right_arm_connected:
        part_driver['right_arm'] = ri.right_robot
    if ri.base_robot and ri.base_connected:
        for part in ['base', 'lift_axis', 'neck']:
            part_driver[part] = ri.base_robot
    return part_driver


def _collect_servos(ri, part_driver: dict) -> List[dict]:
    """遍历所有部件收集舵机详情."""
    all_servos = []
    for part_name in ['left_arm', 'right_arm', 'base', 'lift_axis', 'neck']:
        part_config = ri.servo_ids.get(part_name, {})
        if not isinstance(part_config, dict):
            continue
        driver = part_driver.get(part_name)
        for joint_name, joint_info in part_config.items():
            if not isinstance(joint_info, dict):
                continue
            servo_id = joint_info.get('id')
            if not servo_id:
                continue
            online = servo_id in ri.online_servos
            angle = _read_angle(driver, servo_id, joint_info) if online else 0.0
            all_servos.append({
                'id': servo_id,
                'joint_name': joint_name,
                'part': part_name,
                'brand': joint_info.get('brand', ''),
                'motor_type': joint_info.get('motor_type', ''),
                'angle': angle,
                'online': online,
            })
    return all_servos


def _read_angle(driver, servo_id: int, joint_info: dict) -> float:
    """从驱动器读取单个舵机角度."""
    try:
        position = driver.get_position(servo_id)
        if position is not None:
            brand = (joint_info.get('brand', '') or '').lower()
            if 'feetech' in brand:
                return round((position / 4095.0) * 360.0 - 180.0, 2)
            return round(position, 2)
    except Exception:
        pass
    return 0.0


def _log_summary(all_servos: List[dict], ri) -> None:
    """打印硬件信息摘要日志."""
    online_ids = [s['id'] for s in all_servos if s['online']]
    offline_ids = [s['id'] for s in all_servos if not s['online']]
    print(f"📤 机器人硬件详情: {len(all_servos)} 个舵机, "
          f"在线({len(online_ids)}): {online_ids}, "
          f"离线({len(offline_ids)}): {offline_ids}, "
          f"online_servos: {sorted(ri.online_servos.keys())}, "
          f"左臂={ri.left_arm_connected} 右臂={ri.right_arm_connected} 底盘={ri.base_connected}")
