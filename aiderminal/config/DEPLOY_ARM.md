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
  # 启用纯 Python aiortc WebRTC 推流
  enable_webrtc: true

  # WebRTC 信令已复用 /ws/terminal 通道，无需独立 signaling URL
  webrtc_room_id: "robot-camera"

  # ICE 服务器（兜底：若 ECS 端服务器未响应 webrtc_joined 时降级使用）
  # 正常情况下服务器会返回 ICE 配置，此段仅作为超时兜底
  ice_servers:
    - urls: ["stun:121.40.151.10:3478"]
    - urls: ["turn:121.40.151.10:3478"]
      username: "aider"
      credential: "aider123456"
    - urls: ["turns:houqicg.com:5349"]
      username: "aider"
      credential: "aider123456"

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

> 配置说明：
> - **`ice_servers`**：ICE 服务器列表，包含 STUN/TURN/TURNS 地址。正常情况下由 ECS 服务器通过 `webrtc_joined` 响应下发；若服务器未部署 `webrtc_join` 处理代码，Terminal 会在 5 秒超时后降级使用此本地配置。
> - 信令已从独立 `/ws/signaling` 通道改为**复用 `/ws/terminal` 通道**，不再需要 `webrtc_signaling_url` 配置项。

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
已加入房间 robot-camera, ICE ×3
```
或（服务器未部署 webrtc_join 时降级）：
```
webrtc_joined 超时，降级使用本地 ICE 配置 ×3
```

---

## 六、常用命令

### 6.1 systemd 服务管理

Terminal 通过 systemd 实现开机自启，服务名为 `aider-terminal`。

```bash
# 查看服务状态（是否在运行、最近日志）
systemctl status aider-terminal

# 停止服务（部署/调试时先停下）
sudo systemctl stop aider-terminal

# 启动服务
sudo systemctl start aider-terminal

# 重启服务（先 stop 再 start）
sudo systemctl restart aider-terminal

# 实时查看服务日志
journalctl -u aider-terminal -f --no-pager

# 禁用/启用开机自启
sudo systemctl disable aider-terminal   # 禁止开机自启
sudo systemctl enable aider-terminal    # 启用开机自启

# 确认只有一个 terminal 进程在跑（避免重复连接）
ps aux | grep "python.*main.py" | grep -v grep
```

> **部署后重启流程**：
> ```bash
> sudo systemctl stop aider-terminal   # 停服务
> cd /www/aider/aider_terminal && git pull  # 拉代码
> sudo systemctl start aider-terminal  # 启服务
> journalctl -u aider-terminal -f      # 查看日志确认
> ```

### 6.2 其他常用命令

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
| WebRTC 推流超时 | ① 确认 ECS 端 `aider_server` 已部署 `webrtc_join` 处理代码（见 DEPLOY_ECS.md）；② 确认 `ice_servers` 兜底配置已填写 |
| 视频黑屏 | 确认 ECS frps 在跑 + 安全组 UDP 50000 放行 |
