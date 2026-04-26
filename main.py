#!/usr/bin/env python3
"""
Telegrip Terminal 启动脚本。
直接运行此文件即可启动终端控制系统。

用法:
    python main.py [--no-robot] [--log-level LEVEL]

选项:
    --no-robot      无机器人模式(仅仿真)
    --log-level     日志级别 (debug/info/warning/error/critical)
"""

import sys
import os
import asyncio

# 禁用 CUDA,使用 CPU (避免 NCCL 库问题)
os.environ['CUDA_VISIBLE_DEVICES'] = ''

# 确保可以导入 telegrip 包
sys.path.insert(0, os.path.dirname(__file__))

from telegrip.main import main

if __name__ == "__main__":
    # 默认添加 --no-robot 参数（仅可视化，不连接机械臂）
    if '--no-robot' not in sys.argv:
        sys.argv.append('--no-robot')
    # 默认使用 info 日志级别
    if '--log-level' not in sys.argv:
        sys.argv.extend(['--log-level', 'info'])
    # 默认使用 localhost 作为服务器地址
    if '--server-host' not in sys.argv:
        # sys.argv.extend(['--server-host', 'ws.houqicg.com'])
        sys.argv.extend(['--server-host', 'localhost'])

    # 运行异步主函数
    asyncio.run(main())
