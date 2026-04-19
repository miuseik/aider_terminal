#!/usr/bin/env python3
"""
Telegrip 机械臂遥操作系统启动入口
"""

import sys
from telegrip.main import main_cli

if __name__ == "__main__":
    # 默认添加 --no-robot 参数（仅可视化，不连接机械臂）
    if '--no-robot' not in sys.argv:
        sys.argv.append('--no-robot')
    # 默认使用 info 日志级别
    if '--log-level' not in sys.argv:
        sys.argv.extend(['--log-level', 'info'])
    main_cli()
