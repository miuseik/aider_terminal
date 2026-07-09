"""
腰电机朝向测试脚本
==================

目的: 帮你判断 servo_ids.yaml 的 waist_Link 要不要加 `direction: -1`。

原理:
- RobStride 驱动里 `logical = direction * motor_raw + offset`。
- 本脚本用**默认 direction=+1** 建驱动, 因此 "逻辑角 == 原始电机角",
  发出的角度就是电机真实接收的角度, 不受任何反转影响 —— 正好用来观察物理方向。
- 约定(见 settings.py): 腰 **逻辑正向(正值) = 往后仰**, 逻辑负向 = 往前弯(鞠躬)。
- 我们给电机发一个 +Δ°(原始正向), 看腰实际往哪边动:
    * 往后仰  -> 原始正向 == 逻辑正向 == 约定  -> 朝向正确, 不需要 direction
    * 往前弯  -> 原始正向 != 逻辑正向          -> 需要 direction: -1
- 同时脚本会先扫描 can0, 列出在线电机 ID, 顺带找出腰的真实 ID(替换占位的 8)。

用法 (在 aider_terminal 目录下, 用 aider_venv 运行):
    cd /home/miuseik/www/aider/aider_terminal
    ../aider_venv/bin/python test_waist_direction.py
    ../aider_venv/bin/python test_waist_direction.py 9        # 已知 ID=9, 跳过扫描
    ../aider_venv/bin/python test_waist_direction.py 9 can1   # 指定 CAN 口

注意: 脚本会让腰实际转动一个小角度(默认 15°), 请确保机器人周围安全、腰无遮挡。
"""
import sys
import time
import logging

# 屏蔽驱动内部噪声, 只看关键输出
logging.basicConfig(level=logging.ERROR)

from aiderminal.drivers.actuator.robStride.robstride_driver import RobStrideOfficialDriver
from aiderminal.drivers.actuator.robStride.robstride_dynamics.protocol import MotorType

TEST_DELTA_DEG = 15.0  # 测试转动的安全小角度


def deg2rad(d):
    return d * 3.141592653589793 / 180.0


def main():
    can_if = sys.argv[2] if len(sys.argv) > 2 else "can0"
    preset_id = int(sys.argv[1]) if len(sys.argv) > 1 else None

    print("=" * 60)
    print("  腰电机朝向测试  (CAN 接口 = %s)" % can_if)
    print("=" * 60)

    # direction 默认 +1 -> 逻辑角 == 原始电机角, 便于观察物理方向
    drv = RobStrideOfficialDriver(can_interface=can_if)

    # 1) 找出腰的真实 ID
    if preset_id is not None:
        wid = preset_id
        print("[跳过扫描] 使用指定 ID = %d" % wid)
    else:
        print("扫描 %s 在线电机 (ID 1~30) ..." % can_if)
        found = drv.scan_motors(1, 30)
        print("发现的电机 ID:", found)
        if not found:
            print("✗ 未找到任何电机。请检查 CAN 总线/供电/USB-CAN 适配器是否连接。")
            return
        try:
            wid = int(input("哪个 ID 是腰? 请输入数字: ").strip())
        except ValueError:
            print("✗ 输入无效")
            return
        if wid not in found:
            print("⚠ 警告: %d 不在扫描结果 %s 中, 仍尝试操作..." % (wid, found))

    # 2) 注册电机 (腰 = robstride_04 -> RS04)
    drv.add_motor(wid, motor_type=MotorType.RS04)

    # 3) 读初始原始角度
    p0 = drv.get_position(wid)
    if p0 is None:
        print("✗ 读不到初始角度。电机可能未上电 / 无反馈帧, 无法继续。")
        return
    print("腰初始原始角度 = %.2f°" % p0)

    # 4) 发 +Δ° (direction=+1 下等价于原始电机正向)
    input("\n准备让电机转 +%.0f° (原始电机正向)。\n按 Enter 执行, 然后观察腰往哪边动..."
          % TEST_DELTA_DEG)
    ok = drv.move_one_joint_mit(wid, deg2rad(p0 + TEST_DELTA_DEG))
    if not ok:
        print("✗ 移动指令发送失败 (可能电机未使能或通信异常)")
    time.sleep(2.0)
    p1 = drv.get_position(wid)
    print("移动后原始角度 = %s°" % ("%.2f" % p1 if p1 is not None else "N/A"))

    # 5) 询问物理方向
    print("\n腰实际往哪个方向动了?")
    print("  1 = 往后仰 (身体上部向后倾)")
    print("  2 = 往前弯 / 鞠躬 (身体上部向前倾)")
    ans = input("请输入 1 或 2: ").strip()

    # 6) 回到初始位置
    input("\n准备回到初始 %.2f°。按 Enter 执行..." % p0)
    drv.move_one_joint_mit(wid, deg2rad(p0))
    time.sleep(1.5)

    # 7) 结论 + 要贴的 YAML
    print("\n" + "=" * 60)
    print("  结论")
    print("=" * 60)
    if ans == "1":
        print("原始电机正向 = 往后仰 = 逻辑正向(约定)。朝向正确。")
        print("=> 不需要 direction 字段。servo_ids.yaml 的 waist_Link 保持默认即可。")
        print("\n建议的 waist_Link (把 id 改成真实值 %d):" % wid)
        print("  waist:")
        print("    waist_Link:")
        print("      brand: robstride_04")
        print("      id: %d" % wid)
        print("      joint_name: 腰")
        print("      default_angle: 0")
        print("      min_angle: -90")
        print("      max_angle: 0")
        print("      zero_offset: 0")
    elif ans == "2":
        print("原始电机正向 = 往前弯 ≠ 逻辑正向(约定)。需要反转。")
        print("=> 在 servo_ids.yaml 的 waist_Link 下加 `direction: -1`。")
        print("\n建议的 waist_Link (把 id 改成真实值 %d):" % wid)
        print("  waist:")
        print("    waist_Link:")
        print("      brand: robstride_04")
        print("      id: %d" % wid)
        print("      joint_name: 腰")
        print("      default_angle: 0")
        print("      min_angle: -90")
        print("      max_angle: 0")
        print("      zero_offset: 0")
        print("      direction: -1")
    else:
        print("未识别答案。自行判断: +角度使腰后仰 -> 不加 direction;")
        print("+角度使腰前弯 -> 加 direction: -1")


if __name__ == "__main__":
    main()
