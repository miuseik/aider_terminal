#!/usr/bin/env python3
"""测试 Aloha Mini ZMQ 连接"""
import sys
sys.path.insert(0, '/home/zwz/www/lerobot/aider/aider_terminal')

from telegrip.robots.aloha_mini import AlohaMiniClient

def test_connection():
    """测试连接到 Aloha Mini"""
    print("🔌 测试 Aloha Mini 连接...")
    
    # 配置（根据实际情况修改）
    client = AlohaMiniClient(
        remote_ip="localhost",  # 改为实际 IP
        cmd_port=5555,
        obs_port=5556
    )
    
    try:
        # 连接
        print("正在连接...")
        client.connect()
        print("✅ 连接成功！")
        
        # 尝试获取观测数据
        print("\n📡 获取观测数据...")
        obs = client.get_observation(timeout_ms=1000)
        if obs:
            print(f"✅ 收到观测数据:")
            for key in list(obs.keys())[:5]:  # 只显示前5个字段
                print(f"  {key}: {obs[key]}")
        else:
            print("⚠️ 未收到观测数据")
        
        # 断开
        client.disconnect()
        print("\n✅ 测试完成")
        
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_connection()
