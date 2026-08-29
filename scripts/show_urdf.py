#!/usr/bin/env python3
"""
最小启动：仅打开 PyBullet 仿真窗口显示 Aider URDF。

纯 pybullet 原生加载，不依赖 aider 的 visualizer 封装 / IK 链 / mesh 压缩，
用于最快确认 URDF 模型能正确显示（mesh 路径、关节树）。

用法:
    python3 scripts/show_urdf.py            # 开 GUI 窗口
    python3 scripts/show_urdf.py --headless # 无 GUI，仅验证加载
"""

import os
import sys
import re
import time
import argparse
import tempfile

# 项目根目录（脚本在 aider_terminal/scripts/ 下）
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(PROJECT_ROOT, "URDF", "aider", "urdf", "aider_pro.SLDASM.urdf")
MESH_DIR = os.path.join(PROJECT_ROOT, "URDF", "aider", "meshes")


def main():
    parser = argparse.ArgumentParser(description="打开 PyBullet 显示 Aider URDF")
    parser.add_argument("--headless", action="store_true", help="无 GUI 模式")
    args = parser.parse_args()

    if not os.path.exists(URDF_PATH):
        print(f"❌ URDF 不存在: {URDF_PATH}")
        sys.exit(1)
    print(f"加载 URDF: {URDF_PATH}")

    import pybullet as p
    import pybullet_data

    mode = p.GUI if not args.headless else p.DIRECT
    client = p.connect(mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # ---- 把 URDF 里 package://xxx/meshes/yyy.STL 重写成实际 mesh 路径 ----
    with open(URDF_PATH, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    def _resolve_mesh(match):
        filename = match.group(1)  # e.g. base_link.STL
        real = os.path.join(MESH_DIR, filename)
        return f'filename="{real}"'

    urdf_content = re.sub(
        r'filename="package://[^"]*/([^/"]+)"',
        _resolve_mesh,
        urdf_content
    )

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf',
                                      dir=os.path.dirname(URDF_PATH), delete=False)
    tmp.write(urdf_content)
    tmp.close()

    p.setAdditionalSearchPath(MESH_DIR)
    robot_id = p.loadURDF(tmp.name, useFixedBase=True)
    os.unlink(tmp.name)

    num_joints = p.getNumJoints(robot_id)
    print(f"✅ URDF 已加载: robot_id={robot_id}, 关节数={num_joints}")

    # 列出关节名，确认模型结构
    print("--- 关节列表 ---")
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode() if isinstance(info[1], bytes) else info[1]
        jtype = info[2]
        print(f"  [{i}] {name} (type={jtype})")

    print("提示: 按 Ctrl+C 退出。" if not args.headless else "headless 模式，10 秒后自动退出验证。")
    try:
        if args.headless:
            for _ in range(500):  # ~10s
                p.stepSimulation()
                time.sleep(0.02)
        else:
            while True:
                p.stepSimulation()
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n正在关闭 PyBullet...")
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
