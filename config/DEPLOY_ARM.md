# ARM 机器人端部署指南（树莓派 / ARM 架构）

本指南用于在树莓派等 ARM Linux 设备上部署 `aider_terminal`（Telegrip 遥操作终端）。

---

## 架构说明

```
树莓派(内网) ──WSS──▶ ECS 服务器(公网) ──WSS──▶ 浏览器
    │                      │
    └── FRP 隧道(UDP) ────▶│
```

- **aider_terminal**：跑在树莓派上，采集摄像头 + 控制机械臂
- **frpc（FRP 客户端）**：打通与 ECS 之间的 UDP 隧道，让 WebRTC 视频能穿越内网

---

## 一、克隆代码

```bash
cd ~
git clone <你的仓库地址> aider
cd aider/aider_terminal
```

---

## 二、安装 Python 依赖

```bash
pip install -e .
```

> 也可以用 `python main.py` 启动，它会自动检测并安装缺失的包。

---

## 三、修改配置

编辑 `config/config.yaml`，确认以下几项：

```yaml
network:
  # WebRTC 信令地址 — 改成你的 ECS 域名
  webrtc_signaling_url: "wss://ws.houqicg.com:8442/ws/signaling"
  webrtc_room_id: "robot-camera"

  # 摄像头设备
  video_source: "/dev/video0"       # 用 ls /dev/video* 确认
  camera_width: 640
  camera_height: 480
  camera_fps: 25

robot:
  left_arm:
    port: "/dev/ttySO100blue"       # 用 ls /dev/tty* 确认
  right_arm:
    port: "/dev/ttySO100red"
```

> 可选：通过环境变量覆盖服务器地址
> ```bash
> export TELEGRIP_SERVER_HOST=ws.houqicg.com
> export TELEGRIP_API_HOST=www.houqicg.com
> ```

---

## 四、安装 FRP 客户端（内网穿透）

```bash
cd /opt

# ARM 版本
wget https://github.com/fatedier/frp/releases/download/v0.68.1/frp_0.68.1_linux_arm64.tar.gz
tar -zxvf frp_0.68.1_linux_arm64.tar.gz
mv frp_0.68.1_linux_arm64 frp

# 配置文件
sudo tee /opt/frp/frpc.toml << 'EOF'
serverAddr = "121.40.151.10"
serverPort = 7000
auth.token = "aider123456"

[[proxies]]
name = "webrtc_udp"
type = "udp"
localIP = "127.0.0.1"
localPort = 50000
remotePort = 50000
EOF

# 启动（日志写到 ~/frpc.log，避免 /var/log 权限问题）
nohup /opt/frp/frpc -c /opt/frp/frpc.toml > ~/frpc.log 2>&1 &

# 验证 — 应看到 "start proxy success" 和 "login to server success"
sleep 2 && tail ~/frpc.log
```

> **注意**：`serverAddr` 要改成你的 ECS 公网 IP。

---

## 五、启动机器人终端

```bash
cd ~/aider/aider_terminal

# 生产模式（连远程 ECS）
python main.py

# 开发模式（连本地 localhost）
python main.py --env-dev
```

启动日志中应看到：
```
WebRTC streamer connected, subscribed to room robot-camera
```

---

## 六、常用命令

```bash
# 查看 frpc 状态
ps aux | grep frpc
tail -f ~/frpc.log

# 查看摄像头
ls /dev/video*
v4l2-ctl --list-devices

# 查看串口
ls /dev/tty*
```

---

## 七、故障排查

| 问题 | 排查 |
|------|------|
| frpc 连不上 | 检查 ECS 安全组 TCP 7000 是否放行 |
| 摄像头打不开 | `ls /dev/video*` 确认编号，改 `video_source` |
| 信令连不上 | 确认 `webrtc_signaling_url` 域名可解析 |
| 视频黑屏 | 确认 ECS frps 在跑 + 安全组 UDP 50000 放行 |
