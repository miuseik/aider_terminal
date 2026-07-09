# Aider Terminal

双臂机器人遥操作系统 —— PyBullet 仿真 · Pink IK · WebRTC 推流

---

## 命令速查

| 场景 | 命令 |
|------|------|
| 新电脑，没有 Docker | `./scripts/setup.sh` |
| 有 Docker，第一次 | `docker compose up --build -d` |
| 之后每次启动（生产） | `./start-pro.sh` |
| 之后每次启动（本地） | `./start-dev.sh` |
| 启动 Aloha（生产） | `ROBOT_TYPE=aloha ./start-pro.sh` |
| 启动 Aloha（本地） | `ROBOT_TYPE=aloha ENV=dev docker compose up -d` |
| 改代码后重启 | `docker compose restart` |
| 停止 | `docker compose down` |
| 查看日志 | `docker compose logs -f` |
| 重建镜像 | `docker compose down && docker compose build --no-cache && docker compose up -d` |
```bash
docker compose restart
```
```bash
docker compose logs -f
```
```bash
docker compose down
```
---

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_TYPE` | `aider` | `aider` / `aloha` |
| `NO_SIM` | `false` | 禁用仿真 |
| `NO_VIZ` | `false` | 无头模式 |

用法：`ROBOT_TYPE=aloha ./start-pro.sh`

---

## 项目结构

```
aider_terminal/
├── aiderminal/        # Python 源码
├── URDF/              # 机器人 URDF 模型
├── launch/            # ROS 2 launch 文件
├── scripts/           # 辅助脚本
├── start-dev.sh       # 启动（localhost）
├── start-pro.sh       # 启动（生产）
├── docker-compose.yml
└── Dockerfile
```

---

## 常见问题

```bash
xhost +local:docker                    # 仿真窗口弹不出来
sudo usermod -aG docker $USER          # docker 权限不够（执行后重新登录）
```

---

## 便捷操作（勿删）

```bash
git add ./;git commit -m "hotfix";git push
```

```bash
git pull
```

```bash
ssh gaoda@192.168.0.114

cd www/aider_terminal
密码：gaoda
```

```bash
ssh gaoda@192.168.0.110
#cd /www/aider/aider_terminal ;sudo git pull ; ./start-pro.sh
cd /www/aider/aider_terminal ;sudo git pull ; docker compose restart
密码：gaoda123
```

---

MIT
