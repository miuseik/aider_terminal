# 数据管理 (data/)

运行期产生与消费的数据统一存放位置，与源码分离，便于挂载/清理/备份。

## 目录约定

| 目录 | 用途 | 说明 |
|------|------|------|
| `bags/` | rosbag2 录制数据 | 传感器与状态回放 |
| `maps/` | SLAM 地图 | 供 `robot_localization` / `robot_navigation` 加载 |
| `models/` | 模型权重 | VLA / RL 策略，按模型名分目录 |
| `datasets/` | 训练数据集 | 演示轨迹、采集样本 |
| `checkpoints/` | 训练检查点 | 由 `aider_training` 产出 |
| `logs/` | 运行日志 | 落盘日志（容器内挂载到 `/ws/log`） |
| `calibration/` | 标定结果 | 相机内参、IMU 偏置、外骨骼校准 |

## 注意

- 本目录**默认不入库**（见 `.gitignore`），仅保留空目录占位 `.gitkeep`
- 容器内通过 volume 挂载，路径见 `docker/README.md`
- 大文件（模型、bag）建议走对象存储或外部卷，不直接提交
