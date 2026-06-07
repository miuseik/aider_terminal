#!/usr/bin/env python3
"""
Telegrip Terminal 启动脚本。
直接运行此文件即可启动终端控制系统。

用法:
    python main.py [--no-robot] [--log-level LEVEL] [--robot-type TYPE]

选项:
    --no-robot      无机器人模式(仅仿真)
    --log-level     日志级别 (debug/info/warning/error/critical)
    --robot-type    机器人类型 (aider/aloha/openarmx/custom)

示例:
    python main.py --robot-type aider        # 控制 Aider 机器人
    python main.py --robot-type aloha        # 控制 Aloha 机器人
    python main.py --robot-type openarmx     # 控制 OpenArmX 机器人
"""

import sys
import os
import asyncio
from datetime import datetime

# ✅ 配置日志文件输出（带时间戳 + 按日期分文件 + 启动时清空）
# 使用相对于脚本的路径，避免硬编码用户目录
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f"log_{datetime.now().strftime('%Y-%m-%d')}.log")

# ✅ 程序启动时清空日志文件
with open(LOG_FILE, 'w', encoding='utf-8') as f:
    f.write(f"=== 程序启动于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n\n")

# 自定义日志输出类，同时输出到控制台+文件
class LoggerRedirect:
    def __init__(self, log_file_path):
        self.log_file = open(log_file_path, "a", encoding="utf-8")
        self.console = sys.__stdout__

    def write(self, message):
        # ✅ 允许纯换行符通过，不要过滤掉
        if not message:
            return

        # 加时间戳
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 精确到毫秒
        log_msg = f"[{now}] {message}"
        
        # 控制台打印
        self.console.write(log_msg)
        # 写入日志文件（确保每条日志后有换行符）
        if not log_msg.endswith('\n'):
            log_msg += '\n'
        self.log_file.write(log_msg)
        self.log_file.flush()

    def flush(self):
        self.console.flush()
        self.log_file.flush()

# 接管全局 print
# sys.stdout = LoggerRedirect(LOG_FILE)
# sys.stderr = LoggerRedirect(LOG_FILE)

# 禁用 CUDA,使用 CPU (避免 NCCL 库问题)
os.environ['CUDA_VISIBLE_DEVICES'] = ''

# 解决 PyTorch OpenMP 库冲突问题
# os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

# 确保可以导入 app 模块
sys.path.insert(0, os.path.dirname(__file__))

from app import main

if __name__ == "__main__":
    # 默认使用 info 日志级别
    # if '--log-level' not in sys.argv:
    #     sys.argv.extend(['--log-level', 'info'])
    
    # 默认服务器地址配置
    if '--env-dev' in sys.argv:
        # 开发环境：使用本地地址
        if '--server-host' not in sys.argv:
            sys.argv.extend(['--server-host', 'localhost'])
        if '--api-host' not in sys.argv:
            sys.argv.extend(['--api-host', 'localhost'])
    else:
        # 生产环境（默认）：使用远程地址
        if '--server-host' not in sys.argv:
            sys.argv.extend(['--server-host', 'ws.houqicg.com'])
        if '--api-host' not in sys.argv:
            sys.argv.extend(['--api-host', 'www.houqicg.com'])

    # 运行异步主函数
    asyncio.run(main())
